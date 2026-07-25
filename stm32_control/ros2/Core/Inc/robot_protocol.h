#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


#define ROBOT_PROTOCOL_VERSION             1U
#define ROBOT_PROTOCOL_HEADER_SIZE         10U
#define ROBOT_PROTOCOL_MAX_PAYLOAD_SIZE    256U
#define ROBOT_PROTOCOL_MAX_MESSAGE_SIZE    \
    (ROBOT_PROTOCOL_HEADER_SIZE + ROBOT_PROTOCOL_MAX_PAYLOAD_SIZE)
#define ROBOT_PROTOCOL_CRC_SIZE            2U
#define ROBOT_PROTOCOL_MAX_RAW_SIZE        \
    (ROBOT_PROTOCOL_MAX_MESSAGE_SIZE + ROBOT_PROTOCOL_CRC_SIZE)
#define ROBOT_PROTOCOL_MAX_COBS_SIZE       \
    (ROBOT_PROTOCOL_MAX_RAW_SIZE + \
     (ROBOT_PROTOCOL_MAX_RAW_SIZE / 254U) + 1U)
#define ROBOT_PROTOCOL_MAX_WIRE_SIZE       \
    (ROBOT_PROTOCOL_MAX_COBS_SIZE + 1U)


typedef enum
{
    ROBOT_FLAG_ACK_REQUIRED = 1U << 0,
    ROBOT_FLAG_RESPONSE     = 1U << 1,
    ROBOT_FLAG_ERROR        = 1U << 2,
    ROBOT_FLAG_REALTIME     = 1U << 3
} RobotMessageFlag;


#define ROBOT_KNOWN_FLAGS_MASK \
    (ROBOT_FLAG_ACK_REQUIRED | ROBOT_FLAG_RESPONSE | \
     ROBOT_FLAG_ERROR | ROBOT_FLAG_REALTIME)


typedef enum
{
    ROBOT_NODE_LINUX     = 0x01,
    ROBOT_NODE_ESP32     = 0x02,
    ROBOT_NODE_STM32     = 0x03,
    ROBOT_NODE_BROADCAST = 0xFF
} RobotNodeId;


typedef enum
{
    ROBOT_SERVICE_SYSTEM    = 0x01,
    ROBOT_SERVICE_MOTION    = 0x10,
    ROBOT_SERVICE_TELEMETRY = 0x20,
    ROBOT_SERVICE_CONFIG    = 0x30,
    ROBOT_SERVICE_EVENT     = 0x40,
    ROBOT_SERVICE_OTA       = 0x50
} RobotServiceId;


typedef enum
{
    ROBOT_MOTION_MOVE        = 0x01,
    ROBOT_MOTION_STOP        = 0x02,
    ROBOT_MOTION_CENTER      = 0x03,
    ROBOT_MOTION_ESTOP       = 0x04,
    ROBOT_MOTION_CLEAR_ESTOP = 0x05,
    ROBOT_MOTION_STATE       = 0x10
} RobotMotionOpcode;


typedef enum
{
    ROBOT_STATUS_OK              = 0,
    ROBOT_STATUS_BAD_VERSION     = 1,
    ROBOT_STATUS_BAD_LENGTH      = 2,
    ROBOT_STATUS_BAD_CRC         = 3,
    ROBOT_STATUS_UNKNOWN_SERVICE = 4,
    ROBOT_STATUS_UNKNOWN_OPCODE  = 5,
    ROBOT_STATUS_WRONG_SESSION   = 6,
    ROBOT_STATUS_NOT_OWNER       = 7,
    ROBOT_STATUS_OLD_SEQUENCE    = 8,
    ROBOT_STATUS_ESTOP_ACTIVE    = 9,
    ROBOT_STATUS_OUT_OF_RANGE    = 10,
    ROBOT_STATUS_BUSY            = 11,
    ROBOT_STATUS_DEVICE_OFFLINE  = 12,
    ROBOT_STATUS_NOT_IMPLEMENTED = 13
} RobotStatusCode;


typedef struct
{
    uint8_t version;
    uint8_t flags;
    uint8_t src;
    uint8_t dst;
    uint8_t service;
    uint8_t opcode;
    uint16_t seq;
    uint16_t payload_length;
    const uint8_t *payload;
} RobotProtocolMessage;


typedef struct
{
    uint16_t control_epoch;
    uint16_t valid_ms;
    int16_t linear_mm_s;
    int16_t angular_mrad_s;
    int16_t head_yaw_rate_x10;
    int16_t head_pitch_rate_x10;
} RobotMotionMovePayload;


typedef bool (*RobotProtocolTransmitHandler)(
    const uint8_t *data,
    uint16_t length,
    void *user_context
);


typedef void (*RobotProtocolMessageHandler)(
    const RobotProtocolMessage *message,
    void *user_context
);


typedef struct
{
    uint8_t rx_encoded[ROBOT_PROTOCOL_MAX_COBS_SIZE];
    uint8_t rx_raw[ROBOT_PROTOCOL_MAX_RAW_SIZE];
    uint8_t tx_message[ROBOT_PROTOCOL_MAX_MESSAGE_SIZE];
    uint8_t tx_raw[ROBOT_PROTOCOL_MAX_RAW_SIZE];
    uint8_t tx_wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE];

    uint16_t rx_encoded_length;
    uint16_t next_tx_sequence;
    bool dropping_oversized_frame;

    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    uint32_t overflow_error_count;

    RobotProtocolTransmitHandler transmit_handler;
    RobotProtocolMessageHandler message_handler;
    void *user_context;
} RobotProtocolContext;


void RobotProtocol_Init(
    RobotProtocolContext *context,
    RobotProtocolTransmitHandler transmit_handler,
    RobotProtocolMessageHandler message_handler,
    void *user_context
);


void RobotProtocol_InputByte(
    RobotProtocolContext *context,
    uint8_t byte
);


uint16_t RobotProtocol_NextSequence(
    RobotProtocolContext *context
);


bool RobotProtocol_SendMessage(
    RobotProtocolContext *context,
    uint8_t flags,
    uint8_t dst,
    uint8_t service,
    uint8_t opcode,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t payload_length
);


bool RobotProtocol_SendResponse(
    RobotProtocolContext *context,
    const RobotProtocolMessage *request,
    RobotStatusCode status
);


bool RobotProtocol_DecodeMotionMove(
    const RobotProtocolMessage *message,
    RobotMotionMovePayload *move
);


uint16_t RobotProtocol_Crc16CcittFalse(
    const uint8_t *data,
    uint16_t length
);


#ifdef __cplusplus
}
#endif

#endif
