#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot_dispatcher.h"
#include "robot_motion.h"
#include "robot_protocol.h"
#include "robot_transport.h"
#include "uart_ring_buffer.h"


typedef struct
{
    uint8_t *armed_destination;
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE];
    uint16_t wire_length;
    uint32_t arm_count;
    uint32_t write_count;
    uint32_t now_ms;
    uint32_t write_duration_ms;
    uint32_t last_timeout_ms;
    bool arm_result;
    bool write_result;
} FakeUart;


typedef struct
{
    uint32_t message_count;
    uint8_t last_opcode;
} MessageCapture;


static void write_u16_le(
    uint8_t *data,
    uint16_t value
)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}


static bool fake_arm_receive(
    uint8_t *destination,
    void *user_context
)
{
    FakeUart *uart = (FakeUart *)user_context;

    ++uart->arm_count;

    if (!uart->arm_result)
    {
        return false;
    }

    uart->armed_destination = destination;
    return true;
}


static bool fake_write(
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms,
    void *user_context
)
{
    FakeUart *uart = (FakeUart *)user_context;

    assert(length <= sizeof(uart->wire));
    memcpy(uart->wire, data, length);
    uart->wire_length = length;
    uart->last_timeout_ms = timeout_ms;
    ++uart->write_count;
    uart->now_ms += uart->write_duration_ms;

    return uart->write_result;
}


static uint32_t fake_clock(void *user_context)
{
    FakeUart *uart = (FakeUart *)user_context;
    return uart->now_ms;
}


static void capture_message(
    const RobotProtocolMessage *message,
    const uint8_t *raw,
    uint16_t raw_length,
    void *user_context
)
{
    MessageCapture *capture =
        (MessageCapture *)user_context;

    assert(message != NULL);
    assert(raw != NULL);
    assert(
        raw_length ==
        ROBOT_PROTOCOL_HEADER_SIZE +
            message->payload_length
    );

    ++capture->message_count;
    capture->last_opcode = message->opcode;
}


static void inject_byte(
    RobotTransport *transport,
    FakeUart *uart,
    uint8_t byte
)
{
    assert(uart->armed_destination != NULL);
    *uart->armed_destination = byte;
    uart->armed_destination = NULL;
    RobotTransport_OnRxCompleteFromIsr(
        transport
    );
}


static uint16_t build_stop_wire(uint8_t *wire)
{
    uint8_t message[
        ROBOT_PROTOCOL_HEADER_SIZE
    ] = {
        ROBOT_PROTOCOL_VERSION,
        0U,
        ROBOT_NODE_LINUX,
        ROBOT_NODE_STM32,
        ROBOT_SERVICE_MOTION,
        ROBOT_MOTION_STOP,
        0x01U,
        0x00U,
        0x00U,
        0x00U
    };
    uint8_t raw[
        ROBOT_PROTOCOL_HEADER_SIZE +
        ROBOT_PROTOCOL_CRC_SIZE
    ] = {0};
    uint16_t crc;
    uint16_t read_index = 0U;
    uint16_t write_index = 1U;
    uint16_t code_index = 0U;
    uint8_t code = 1U;

    memcpy(raw, message, sizeof(message));
    crc = RobotProtocol_Crc16CcittFalse(
        message,
        (uint16_t)sizeof(message)
    );
    raw[sizeof(message)] =
        (uint8_t)(crc & 0xFFU);
    raw[sizeof(message) + 1U] =
        (uint8_t)(crc >> 8U);

    while (read_index < sizeof(raw))
    {
        if (raw[read_index] == 0U)
        {
            wire[code_index] = code;
            code = 1U;
            code_index = write_index++;
            ++read_index;
        }
        else
        {
            wire[write_index++] =
                raw[read_index++];
            ++code;

            if (code == 0xFFU)
            {
                wire[code_index] = code;
                code = 1U;
                code_index = write_index++;
            }
        }
    }

    wire[code_index] = code;
    wire[write_index++] = 0U;
    return write_index;
}


static void test_ring_capacity_overflow_and_wrap(void)
{
    UartRingBuffer ring;
    uint16_t index;
    uint8_t byte;

    UartRing_Init(&ring);

    for (
        index = 0U;
        index < ROBOT_UART_RX_RING_CAPACITY;
        ++index
    )
    {
        assert(
            UartRing_PushFromIsr(
                &ring,
                (uint8_t)index
            )
        );
    }

    assert(
        UartRing_Count(&ring) ==
        ROBOT_UART_RX_RING_CAPACITY
    );
    assert(!UartRing_PushFromIsr(&ring, 0xEEU));
    assert(ring.overflow_count == 1U);

    for (index = 0U; index < 128U; ++index)
    {
        assert(UartRing_Pop(&ring, &byte));
        assert(byte == (uint8_t)index);
    }

    for (index = 0U; index < 128U; ++index)
    {
        assert(
            UartRing_PushFromIsr(
                &ring,
                (uint8_t)(0x80U + index)
            )
        );
    }

    assert(
        UartRing_Count(&ring) ==
        ROBOT_UART_RX_RING_CAPACITY
    );

    for (index = 128U; index < 256U; ++index)
    {
        assert(UartRing_Pop(&ring, &byte));
        assert(byte == (uint8_t)index);
    }

    for (index = 0U; index < 128U; ++index)
    {
        assert(UartRing_Pop(&ring, &byte));
        assert(
            byte ==
            (uint8_t)(0x80U + index)
        );
    }

    assert(!UartRing_Pop(&ring, &byte));

    /*
     * Exercise the monotonic 16-bit indices across their numeric wrap.
     */
    for (index = 0U; index < 300U; ++index)
    {
        uint16_t inner;

        for (inner = 0U; inner < 250U; ++inner)
        {
            assert(
                UartRing_PushFromIsr(
                    &ring,
                    (uint8_t)inner
                )
            );
        }

        for (inner = 0U; inner < 250U; ++inner)
        {
            assert(UartRing_Pop(&ring, &byte));
            assert(byte == (uint8_t)inner);
        }
    }

    assert(UartRing_Count(&ring) == 0U);
}


static void test_transport_split_sticky_and_pressure(void)
{
    RobotProtocolContext protocol;
    RobotTransport transport;
    MessageCapture capture = {0};
    FakeUart uart = {
        .arm_result = true,
        .write_result = true
    };
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
    uint16_t wire_length;
    uint16_t index;
    uint16_t repeat;

    RobotProtocol_Init(
        &protocol,
        NULL,
        capture_message,
        &capture
    );
    RobotTransport_Init(
        &transport,
        &protocol,
        fake_arm_receive,
        fake_write,
        fake_clock,
        &uart
    );

    assert(RobotTransport_StartRx(&transport));
    wire_length = build_stop_wire(wire);

    /*
     * Two complete frames arrive back-to-back before main polls.
     */
    for (repeat = 0U; repeat < 2U; ++repeat)
    {
        for (index = 0U; index < wire_length; ++index)
        {
            inject_byte(
                &transport,
                &uart,
                wire[index]
            );
        }
    }

    assert(capture.message_count == 0U);
    assert(
        RobotTransport_Poll(&transport) ==
        (uint16_t)(wire_length * 2U)
    );
    assert(capture.message_count == 2U);
    assert(capture.last_opcode == ROBOT_MOTION_STOP);

    /*
     * Sustained 115200-equivalent byte volume, drained periodically.
     */
    for (repeat = 0U; repeat < 800U; ++repeat)
    {
        for (index = 0U; index < wire_length; ++index)
        {
            inject_byte(
                &transport,
                &uart,
                wire[index]
            );
        }

        if ((repeat & 7U) == 7U)
        {
            assert(
                RobotTransport_Poll(
                    &transport
                ) > 0U
            );
        }
    }

    while (UartRing_Count(&transport.rx_ring) > 0U)
    {
        assert(
            RobotTransport_Poll(
                &transport
            ) > 0U
        );
    }

    assert(capture.message_count == 802U);
    assert(transport.rx_ring.overflow_count == 0U);
    assert(
        transport.rx_byte_count ==
        (uint32_t)wire_length * 802U
    );
}


static void test_transport_overflow_and_rearm_errors(void)
{
    RobotProtocolContext protocol;
    RobotTransport transport;
    FakeUart uart = {
        .arm_result = true,
        .write_result = true
    };
    uint16_t index;

    RobotProtocol_Init(
        &protocol,
        NULL,
        NULL,
        NULL
    );
    RobotTransport_Init(
        &transport,
        &protocol,
        fake_arm_receive,
        fake_write,
        fake_clock,
        &uart
    );
    assert(RobotTransport_StartRx(&transport));

    for (
        index = 0U;
        index <= ROBOT_UART_RX_RING_CAPACITY;
        ++index
    )
    {
        inject_byte(&transport, &uart, 0x55U);
    }

    assert(
        transport.rx_ring.overflow_count == 1U
    );
    assert(
        UartRing_Count(&transport.rx_ring) ==
        ROBOT_UART_RX_RING_CAPACITY
    );
    assert(
        RobotTransport_Poll(&transport) ==
        ROBOT_TRANSPORT_MAX_POLL_BYTES
    );
    assert(UartRing_Count(&transport.rx_ring) == 0U);

    uart.arm_result = false;
    RobotTransport_OnErrorFromIsr(&transport);
    assert(transport.uart_error_count == 1U);
    assert(transport.rx_rearm_error_count == 1U);
}


static void test_blocking_tx_is_measured(void)
{
    RobotTransport transport;
    FakeUart uart = {
        .arm_result = true,
        .write_result = true,
        .write_duration_ms = 7U
    };
    const uint8_t data[] = {0x01U, 0x00U};

    RobotTransport_Init(
        &transport,
        NULL,
        fake_arm_receive,
        fake_write,
        fake_clock,
        &uart
    );

    assert(
        RobotTransport_Transmit(
            data,
            (uint16_t)sizeof(data),
            &transport
        )
    );
    assert(transport.tx_success_count == 1U);
    assert(transport.tx_error_count == 0U);
    assert(transport.tx_max_duration_ms == 7U);
    assert(
        uart.last_timeout_ms ==
        ROBOT_UART_TX_TIMEOUT_MS
    );

    uart.write_result = false;
    uart.write_duration_ms = 3U;

    assert(
        !RobotTransport_Transmit(
            data,
            (uint16_t)sizeof(data),
            &transport
        )
    );
    assert(transport.tx_success_count == 1U);
    assert(transport.tx_error_count == 1U);
    assert(transport.tx_max_duration_ms == 7U);
}


static void test_bounded_poll_preserves_motion_deadline(void)
{
    RobotMotionController motion;
    RobotProtocolContext protocol;
    RobotTransport transport;
    FakeUart uart = {
        .arm_result = true,
        .write_result = true
    };
    uint8_t payload[12] = {0};
    RobotProtocolMessage move = {
        .version = ROBOT_PROTOCOL_VERSION,
        .flags = ROBOT_FLAG_REALTIME,
        .src = ROBOT_NODE_LINUX,
        .dst = ROBOT_NODE_STM32,
        .service = ROBOT_SERVICE_MOTION,
        .opcode = ROBOT_MOTION_MOVE,
        .seq = 1U,
        .payload_length = (uint16_t)sizeof(payload),
        .payload = payload
    };
    uint16_t index;

    RobotMotion_Init(
        &motion,
        NULL,
        NULL,
        0U
    );

    write_u16_le(payload, 1U);
    write_u16_le(payload + 2U, 100U);
    write_u16_le(payload + 8U, 300U);

    assert(
        RobotMotion_HandleMessage(
            &motion,
            &move
        ) == ROBOT_STATUS_OK
    );
    assert(motion.motion_active);

    RobotProtocol_Init(
        &protocol,
        NULL,
        NULL,
        NULL
    );
    RobotTransport_Init(
        &transport,
        &protocol,
        fake_arm_receive,
        fake_write,
        fake_clock,
        &uart
    );
    assert(RobotTransport_StartRx(&transport));

    for (
        index = 0U;
        index < ROBOT_UART_RX_RING_CAPACITY;
        ++index
    )
    {
        inject_byte(&transport, &uart, 0x55U);
    }

    assert(
        RobotTransport_Poll(&transport) ==
        ROBOT_TRANSPORT_MAX_POLL_BYTES
    );

    /*
     * Main regains control after a bounded drain and can run the watchdog.
     */
    RobotMotion_Process(&motion, 100U);
    assert(!motion.motion_active);
    assert(motion.motion_timed_out);
}


int main(void)
{
    test_ring_capacity_overflow_and_wrap();
    test_transport_split_sticky_and_pressure();
    test_transport_overflow_and_rearm_errors();
    test_blocking_tx_is_measured();
    test_bounded_poll_preserves_motion_deadline();

    puts("STM32 UART ring and transport host tests passed");
    return 0;
}
