#include "daft/protocol_v2.hpp"

#include <string.h>

namespace daft {

namespace {

constexpr size_t HEADER_SIZE = 7;
constexpr size_t CRC_SIZE = 2;

}  // namespace

void write_u16(uint8_t* p, uint16_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFFu);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void write_u32(uint8_t* p, uint32_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFFu);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  p[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  p[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

void write_i32(uint8_t* p, int32_t value) {
  write_u32(p, static_cast<uint32_t>(value));
}

uint16_t read_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
}

uint32_t read_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

int32_t read_i32(const uint8_t* p) {
  return static_cast<int32_t>(read_u32(p));
}

bool encode_packet(const Packet& packet, uint8_t* frame, size_t frame_capacity, size_t* frame_length) {
  if (packet.payload_len > MAX_PAYLOAD_SIZE) {
    return false;
  }

  uint8_t raw[MAX_RAW_PACKET_SIZE] = {};
  raw[0] = PROTOCOL_VERSION;
  raw[1] = packet.msg_id;
  raw[2] = packet.flags;
  write_u16(raw + 3, packet.seq);
  write_u16(raw + 5, packet.payload_len);
  if (packet.payload_len > 0) {
    memcpy(raw + HEADER_SIZE, packet.payload, packet.payload_len);
  }

  const size_t raw_without_crc = HEADER_SIZE + packet.payload_len;
  const uint16_t crc = crc16_ccitt(raw, raw_without_crc);
  write_u16(raw + raw_without_crc, crc);

  const size_t encoded_len = cobs_encode(raw, raw_without_crc + CRC_SIZE, frame, frame_capacity);
  if (encoded_len == 0 || encoded_len + 1 > frame_capacity) {
    return false;
  }

  frame[encoded_len] = 0;
  *frame_length = encoded_len + 1;
  return true;
}

bool decode_packet(const uint8_t* frame, size_t frame_length, Packet* packet, ErrorCode* error) {
  uint8_t raw[MAX_RAW_PACKET_SIZE] = {};
  size_t raw_len = 0;
  if (!cobs_decode(frame, frame_length, raw, sizeof(raw), &raw_len)) {
    *error = ErrorCode::ERR_BAD_PAYLOAD;
    return false;
  }

  if (raw_len < HEADER_SIZE + CRC_SIZE) {
    *error = ErrorCode::ERR_BAD_PAYLOAD;
    return false;
  }

  const uint8_t version = raw[0];
  if (version != PROTOCOL_VERSION) {
    *error = ErrorCode::ERR_BAD_VERSION;
    return false;
  }

  const uint16_t payload_len = read_u16(raw + 5);
  if (payload_len > MAX_PAYLOAD_SIZE || raw_len != HEADER_SIZE + payload_len + CRC_SIZE) {
    *error = ErrorCode::ERR_BAD_PAYLOAD;
    return false;
  }

  const uint16_t expected_crc = read_u16(raw + HEADER_SIZE + payload_len);
  const uint16_t actual_crc = crc16_ccitt(raw, HEADER_SIZE + payload_len);
  if (expected_crc != actual_crc) {
    *error = ErrorCode::ERR_BAD_CRC;
    return false;
  }

  *packet = Packet{};
  packet->msg_id = raw[1];
  packet->flags = raw[2];
  packet->seq = read_u16(raw + 3);
  packet->payload_len = payload_len;
  if (payload_len > 0) {
    memcpy(packet->payload, raw + HEADER_SIZE, payload_len);
  }

  *error = ErrorCode::OK;
  return true;
}

FramingDecoder::Result FramingDecoder::consume(uint8_t byte, Packet* packet, ErrorCode* error) {
  if (byte == 0) {
    if (frame_len_ == 0) {
      return Result::NONE;
    }

    const bool ok = decode_packet(frame_, frame_len_, packet, error);
    frame_len_ = 0;
    return ok ? Result::PACKET : Result::FRAME_ERROR;
  }

  if (frame_len_ >= sizeof(frame_)) {
    frame_len_ = 0;
    *error = ErrorCode::ERR_BAD_PAYLOAD;
    return Result::FRAME_ERROR;
  }

  frame_[frame_len_++] = byte;
  return Result::NONE;
}

void FramingDecoder::reset() {
  frame_len_ = 0;
}

const char* msg_name(uint8_t msg_id) {
  switch (static_cast<MsgId>(msg_id)) {
    case MsgId::PING: return "PING";
    case MsgId::PONG: return "PONG";
    case MsgId::GET_IDENTITY: return "GET_IDENTITY";
    case MsgId::IDENTITY: return "IDENTITY";
    case MsgId::GET_CAPABILITIES: return "GET_CAPABILITIES";
    case MsgId::CAPABILITIES: return "CAPABILITIES";
    case MsgId::GET_STATUS: return "GET_STATUS";
    case MsgId::STATUS: return "STATUS";
    case MsgId::GET_FAULTS: return "GET_FAULTS";
    case MsgId::FAULTS: return "FAULTS";
    case MsgId::GET_COUNTERS: return "GET_COUNTERS";
    case MsgId::COUNTERS: return "COUNTERS";
    case MsgId::HEARTBEAT: return "HEARTBEAT";
    case MsgId::SET_CONTROL_MODE: return "SET_CONTROL_MODE";
    case MsgId::ESTOP: return "ESTOP";
    case MsgId::RAMP_STOP: return "RAMP_STOP";
    case MsgId::CLEAR_FAULTS: return "CLEAR_FAULTS";
    case MsgId::CONFIG_QUERY: return "CONFIG_QUERY";
    case MsgId::CONFIG_VALUE: return "CONFIG_VALUE";
    case MsgId::CONFIG_STAGE_FIELD: return "CONFIG_STAGE_FIELD";
    case MsgId::CONFIG_APPLY: return "CONFIG_APPLY";
    case MsgId::CONFIG_SAVE: return "CONFIG_SAVE";
    case MsgId::CONFIG_RESET_DEFAULTS: return "CONFIG_RESET_DEFAULTS";
    case MsgId::REBOOT: return "REBOOT";
    case MsgId::GET_DRIVER_STATUS: return "GET_DRIVER_STATUS";
    case MsgId::DRIVER_STATUS: return "DRIVER_STATUS";
    case MsgId::MOVE_ABS: return "MOVE_ABS";
    case MsgId::MOVE_REL: return "MOVE_REL";
    case MsgId::RUN_VEL: return "RUN_VEL";
    case MsgId::SET_TELEM_RATE: return "SET_TELEM_RATE";
    case MsgId::ACK: return "ACK";
    case MsgId::ERROR: return "ERROR";
    case MsgId::EVENT: return "EVENT";
    default: return "UNKNOWN";
  }
}

const char* error_name(ErrorCode error) {
  switch (error) {
    case ErrorCode::OK: return "OK";
    case ErrorCode::ERR_BUSY: return "ERR_BUSY";
    case ErrorCode::ERR_INVALID_FIELD: return "ERR_INVALID_FIELD";
    case ErrorCode::ERR_OUT_OF_RANGE: return "ERR_OUT_OF_RANGE";
    case ErrorCode::ERR_REQUIRES_REBOOT: return "ERR_REQUIRES_REBOOT";
    case ErrorCode::ERR_UNSUPPORTED: return "ERR_UNSUPPORTED";
    case ErrorCode::ERR_FAULTED: return "ERR_FAULTED";
    case ErrorCode::ERR_BAD_STATE: return "ERR_BAD_STATE";
    case ErrorCode::ERR_BAD_CRC: return "ERR_BAD_CRC";
    case ErrorCode::ERR_BAD_PAYLOAD: return "ERR_BAD_PAYLOAD";
    case ErrorCode::ERR_BAD_VERSION: return "ERR_BAD_VERSION";
    case ErrorCode::ERR_DUPLICATE_MISMATCH: return "ERR_DUPLICATE_MISMATCH";
    case ErrorCode::ERR_STORAGE: return "ERR_STORAGE";
    default: return "ERR_UNKNOWN";
  }
}

const char* state_name(ActuatorState state) {
  switch (state) {
    case ActuatorState::BOOTING: return "BOOTING";
    case ActuatorState::DISABLED: return "DISABLED";
    case ActuatorState::IDLE: return "IDLE";
    case ActuatorState::MOVING_POSITION: return "MOVING_POSITION";
    case ActuatorState::RUNNING_VELOCITY: return "RUNNING_VELOCITY";
    case ActuatorState::STOPPING: return "STOPPING";
    case ActuatorState::FAULTED: return "FAULTED";
    case ActuatorState::CONFIG_STAGED: return "CONFIG_STAGED";
    case ActuatorState::COMMISSIONING: return "COMMISSIONING";
    default: return "UNKNOWN";
  }
}

const char* event_name(EventType event) {
  switch (event) {
    case EventType::BOOT_REASON: return "BOOT_REASON";
    case EventType::STATE_CHANGED: return "STATE_CHANGED";
    case EventType::FAULT_SET: return "FAULT_SET";
    case EventType::FAULT_CLEARED: return "FAULT_CLEARED";
    case EventType::CONFIG_APPLIED: return "CONFIG_APPLIED";
    case EventType::CONFIG_SAVED: return "CONFIG_SAVED";
    case EventType::DRIVER_UART_FAILURE: return "DRIVER_UART_FAILURE";
    case EventType::HOST_TIMEOUT: return "HOST_TIMEOUT";
    default: return "EVENT_UNKNOWN";
  }
}

}  // namespace daft
