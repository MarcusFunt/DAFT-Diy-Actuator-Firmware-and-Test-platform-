#pragma once

#include <stddef.h>
#include <stdint.h>

namespace daft {

constexpr uint8_t PROTOCOL_VERSION = 2;
constexpr size_t MAX_PAYLOAD_SIZE = 128;
constexpr size_t MAX_RAW_PACKET_SIZE = 1 + 1 + 1 + 2 + 2 + MAX_PAYLOAD_SIZE + 2;
constexpr size_t MAX_FRAME_SIZE = MAX_RAW_PACKET_SIZE + 4;

enum class MsgId : uint8_t {
  PING = 0x01,
  PONG = 0x02,
  GET_IDENTITY = 0x03,
  IDENTITY = 0x04,
  GET_CAPABILITIES = 0x05,
  CAPABILITIES = 0x06,
  GET_STATUS = 0x07,
  STATUS = 0x08,
  GET_FAULTS = 0x09,
  FAULTS = 0x0A,
  GET_COUNTERS = 0x0B,
  COUNTERS = 0x0C,
  HEARTBEAT = 0x0D,
  SET_CONTROL_MODE = 0x10,
  ESTOP = 0x11,
  RAMP_STOP = 0x12,
  CLEAR_FAULTS = 0x13,
  CONFIG_QUERY = 0x20,
  CONFIG_VALUE = 0x21,
  CONFIG_STAGE_FIELD = 0x22,
  CONFIG_APPLY = 0x23,
  CONFIG_SAVE = 0x24,
  CONFIG_RESET_DEFAULTS = 0x25,
  REBOOT = 0x26,
  GET_DRIVER_STATUS = 0x30,
  DRIVER_STATUS = 0x31,
  MOVE_ABS = 0x40,
  MOVE_REL = 0x41,
  RUN_VEL = 0x42,
  SET_TELEM_RATE = 0x50,
  ACK = 0x80,
  ERROR = 0x81,
  EVENT = 0x82,
};

enum class PacketFlag : uint8_t {
  NONE = 0,
  RESPONSE = 1u << 0,
  ERROR = 1u << 1,
  TELEMETRY = 1u << 2,
};

enum class ErrorCode : uint8_t {
  OK = 0,
  ERR_BUSY = 1,
  ERR_INVALID_FIELD = 2,
  ERR_OUT_OF_RANGE = 3,
  ERR_REQUIRES_REBOOT = 4,
  ERR_UNSUPPORTED = 5,
  ERR_FAULTED = 6,
  ERR_BAD_STATE = 7,
  ERR_BAD_CRC = 8,
  ERR_BAD_PAYLOAD = 9,
  ERR_BAD_VERSION = 10,
  ERR_DUPLICATE_MISMATCH = 11,
  ERR_STORAGE = 12,
};

enum class ActuatorState : uint8_t {
  BOOTING = 0,
  DISABLED = 1,
  IDLE = 2,
  MOVING_POSITION = 3,
  RUNNING_VELOCITY = 4,
  STOPPING = 5,
  FAULTED = 6,
  CONFIG_STAGED = 7,
  COMMISSIONING = 8,
};

enum class ControlMode : uint8_t {
  DISABLED = 0,
  POSITION = 1,
  VELOCITY = 2,
  COMMISSIONING = 3,
};

struct Packet {
  uint8_t msg_id = 0;
  uint8_t flags = 0;
  uint16_t seq = 0;
  uint16_t payload_len = 0;
  uint8_t payload[MAX_PAYLOAD_SIZE] = {};
};

struct Counters {
  uint32_t crc_errors = 0;
  uint32_t decode_errors = 0;
  uint32_t dropped_packets = 0;
  uint32_t duplicate_packets = 0;
  uint32_t command_count = 0;
  uint32_t rejected_commands = 0;
};

uint16_t crc16_ccitt(const uint8_t* data, size_t length);
size_t cobs_encode(const uint8_t* input, size_t length, uint8_t* output, size_t output_capacity);
bool cobs_decode(const uint8_t* input, size_t length, uint8_t* output, size_t output_capacity, size_t* decoded_length);

bool encode_packet(const Packet& packet, uint8_t* frame, size_t frame_capacity, size_t* frame_length);
bool decode_packet(const uint8_t* frame, size_t frame_length, Packet* packet, ErrorCode* error);

void write_u16(uint8_t* p, uint16_t value);
void write_u32(uint8_t* p, uint32_t value);
void write_i32(uint8_t* p, int32_t value);
uint16_t read_u16(const uint8_t* p);
uint32_t read_u32(const uint8_t* p);
int32_t read_i32(const uint8_t* p);

class FramingDecoder {
 public:
  enum class Result : uint8_t {
    NONE = 0,
    PACKET = 1,
    FRAME_ERROR = 2,
  };

  Result consume(uint8_t byte, Packet* packet, ErrorCode* error);
  void reset();

 private:
  uint8_t frame_[MAX_FRAME_SIZE] = {};
  size_t frame_len_ = 0;
};

const char* msg_name(uint8_t msg_id);
const char* error_name(ErrorCode error);
const char* state_name(ActuatorState state);

}  // namespace daft
