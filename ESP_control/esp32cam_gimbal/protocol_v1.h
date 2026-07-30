#pragma once

#include <stddef.h>
#include <stdint.h>


namespace ProtocolV1
{

static constexpr uint8_t VERSION = 1;
static constexpr size_t HEADER_SIZE = 10;
static constexpr size_t MAX_PAYLOAD_SIZE = 256;
static constexpr size_t MAX_MESSAGE_SIZE =
    HEADER_SIZE + MAX_PAYLOAD_SIZE;

enum MessageFlag : uint8_t
{
    FLAG_ACK_REQUIRED = 1U << 0,
    FLAG_RESPONSE     = 1U << 1,
    FLAG_ERROR        = 1U << 2,
    FLAG_REALTIME     = 1U << 3
};

static constexpr uint8_t KNOWN_FLAGS_MASK =
    FLAG_ACK_REQUIRED |
    FLAG_RESPONSE |
    FLAG_ERROR |
    FLAG_REALTIME;

enum NodeId : uint8_t
{
    NODE_LINUX     = 0x01,
    NODE_ESP32     = 0x02,
    NODE_STM32     = 0x03,
    NODE_BROADCAST = 0xFF
};

enum ServiceId : uint8_t
{
    SERVICE_SYSTEM    = 0x01,
    SERVICE_MOTION    = 0x10,
    SERVICE_TELEMETRY = 0x20,
    SERVICE_CONFIG    = 0x30,
    SERVICE_EVENT     = 0x40,
    SERVICE_OTA       = 0x50,
    SERVICE_SENSOR    = 0x60
};

enum SystemOpcode : uint8_t
{
    SYSTEM_PING = 0x01,
    SYSTEM_PONG = 0x02,
    SYSTEM_RESET = 0x03
};

enum TelemetryOpcode : uint8_t
{
    TELEMETRY_DATA  = 0x01,
    TELEMETRY_BATCH = 0x02,
    TELEMETRY_SAMPLE_BLOCK = 0x03
};

enum ConfigOpcode : uint8_t
{
    CONFIG_GET = 0x01,
    CONFIG_SET = 0x02
};

enum EventOpcode : uint8_t
{
    EVENT_REPORT = 0x01
};

enum OtaOpcode : uint8_t
{
    OTA_START = 0x01,
    OTA_CHUNK = 0x02,
    OTA_FINISH = 0x03,
    OTA_ABORT = 0x04
};

enum SensorOpcode : uint8_t
{
    SENSOR_LIST   = 0x01,
    SENSOR_INFO   = 0x02,
    SENSOR_CONFIG = 0x03,
    SENSOR_START  = 0x04,
    SENSOR_STOP   = 0x05,
    SENSOR_DATA   = 0x06,
    SENSOR_STATUS = 0x07
};

enum StatusCode : uint16_t
{
    STATUS_OK = 0,
    STATUS_BAD_VERSION = 1,
    STATUS_BAD_LENGTH = 2,
    STATUS_BAD_CRC = 3,
    STATUS_UNKNOWN_SERVICE = 4,
    STATUS_UNKNOWN_OPCODE = 5,
    STATUS_WRONG_SESSION = 6,
    STATUS_NOT_OWNER = 7,
    STATUS_OLD_SEQUENCE = 8,
    STATUS_ESTOP_ACTIVE = 9,
    STATUS_OUT_OF_RANGE = 10,
    STATUS_BUSY = 11,
    STATUS_DEVICE_OFFLINE = 12,
    STATUS_NOT_IMPLEMENTED = 13,
    STATUS_REQUEST_ID_CONFLICT = 14
};

enum MotionOpcode : uint8_t
{
    MOTION_MOVE        = 0x01,
    MOTION_STOP        = 0x02,
    MOTION_CENTER      = 0x03,
    MOTION_ESTOP       = 0x04,
    MOTION_CLEAR_ESTOP = 0x05,
    MOTION_STATE       = 0x10
};

struct MessageView
{
    uint8_t version;
    uint8_t flags;
    uint8_t src;
    uint8_t dst;
    uint8_t service;
    uint8_t opcode;
    uint16_t seq;
    uint16_t payloadLength;
    const uint8_t *payload;
};

enum class DecodeStatus : uint8_t
{
    OK = 0,
    NULL_DATA,
    HEADER_TOO_SHORT,
    UNSUPPORTED_VERSION,
    UNKNOWN_FLAGS,
    PAYLOAD_TOO_LARGE,
    LENGTH_MISMATCH
};

uint16_t readUint16Le(
    const uint8_t *data
);

void writeUint16Le(
    uint8_t *data,
    uint16_t value
);

DecodeStatus decodeMessage(
    const uint8_t *data,
    size_t length,
    MessageView &message
);

bool encodeMessage(
    const MessageView &message,
    uint8_t *output,
    size_t outputCapacity,
    size_t &outputLength
);

const char *decodeStatusName(
    DecodeStatus status
);

}
