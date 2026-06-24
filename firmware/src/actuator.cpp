#include "daft/actuator.hpp"

#include <string.h>

#include "daft/build_info.hpp"

namespace daft {

namespace {

uint8_t msg_u8(MsgId msg) {
  return static_cast<uint8_t>(msg);
}

uint8_t code_u8(ErrorCode code) {
  return static_cast<uint8_t>(code);
}

uint8_t state_u8(ActuatorState state) {
  return static_cast<uint8_t>(state);
}

uint8_t mode_u8(ControlMode mode) {
  return static_cast<uint8_t>(mode);
}

bool send_packet(const ResponseWriter& writer, const Packet& packet) {
  return writer.send != nullptr && writer.send(writer.context, packet);
}

struct CachedWriterContext {
  ResponseWriter downstream;
  Packet* responses = nullptr;
  uint8_t capacity = 0;
  uint8_t count = 0;
};

bool cached_send(void* context, const Packet& packet) {
  CachedWriterContext* cached = static_cast<CachedWriterContext*>(context);
  if (cached->responses != nullptr && cached->count < cached->capacity) {
    cached->responses[cached->count++] = packet;
  }
  return send_packet(cached->downstream, packet);
}

bool payload_empty(const Packet& packet) {
  return packet.payload_len == 0;
}

bool payload_u16(const Packet& packet, uint16_t* value) {
  if (packet.payload_len != 2) {
    return false;
  }
  *value = read_u16(packet.payload);
  return true;
}

bool soft_limit_violation(const ActuatorConfig& config, int32_t target) {
  return config.motion.soft_limits_enabled &&
         (target < config.motion.soft_min_steps || target > config.motion.soft_max_steps);
}

uint16_t fault_bit(FaultFlag flag) {
  uint32_t value = fault_value(flag);
  uint16_t bit = 0;
  while (value > 1u) {
    value >>= 1;
    ++bit;
  }
  return bit;
}

void copy_ascii(uint8_t* dst, size_t dst_capacity, const char* src, uint8_t* written) {
  size_t count = 0;
  if (src != nullptr) {
    while (src[count] != '\0' && count < dst_capacity) {
      dst[count] = static_cast<uint8_t>(src[count]);
      ++count;
    }
  }
  *written = static_cast<uint8_t>(count);
}

}  // namespace

void Actuator::begin(const ActuatorConfig& defaults, const MotionBackend& backend) {
  defaults_ = defaults;
  active_config_ = defaults;
  staged_config_ = defaults;
  backend_ = backend;
  state_ = ActuatorState::BOOTING;
  control_mode_ = ControlMode::DISABLED;
  faults_ = 0;
  counters_ = {};
  config_generation_ = 0;
  driver_enabled_ = false;
  config_staged_ = false;
  boot_event_pending_ = true;
  boot_reason_ = backend_.boot_reason != nullptr ? backend_.boot_reason(backend_.context) : 0;
  last_host_ms_ = now_ms();
  last_telemetry_ms_ = now_ms();
  last_request_valid_ = false;
  last_request_ = Packet{};
  last_responses_[0] = Packet{};
  last_response_count_ = 0;

  ActuatorConfig loaded{};
  uint32_t generation = 0;
  if (backend_.load_config != nullptr && backend_.load_config(backend_.context, &loaded, &generation)) {
    ConfigValidationResult result = validate_config(loaded);
    if (result.ok) {
      active_config_ = loaded;
      staged_config_ = loaded;
      config_generation_ = generation;
    } else {
      set_fault(FaultFlag::INVALID_CONFIG);
    }
  }

  if (backend_.configure_driver != nullptr && !backend_.configure_driver(backend_.context, active_config_.driver)) {
    set_fault(FaultFlag::DRIVER);
    transition_to(ActuatorState::FAULTED);
    return;
  }

  if (backend_.enable_driver != nullptr) {
    backend_.enable_driver(backend_.context, false);
  }
  transition_to(ActuatorState::DISABLED);
}

void Actuator::update(const ResponseWriter* event_writer) {
  const uint32_t now = now_ms();

  if (boot_event_pending_ && event_writer != nullptr) {
    send_event(*event_writer, 0, EventType::BOOT_REASON, EventSeverity::INFO,
               static_cast<uint16_t>(boot_reason_ & 0xFFFFu), boot_reason_);
    boot_event_pending_ = false;
  }

  if ((state_ == ActuatorState::MOVING_POSITION || state_ == ActuatorState::STOPPING) &&
      backend_.motion_active != nullptr && !backend_.motion_active(backend_.context)) {
    transition_to(driver_enabled_ ? ActuatorState::IDLE : ActuatorState::DISABLED, event_writer);
    control_mode_ = driver_enabled_ ? ControlMode::POSITION : ControlMode::DISABLED;
  }

  if (state_ == ActuatorState::RUNNING_VELOCITY && active_config_.safety.host_timeout_ms > 0 &&
      static_cast<uint32_t>(now - last_host_ms_) > active_config_.safety.host_timeout_ms) {
    set_fault(FaultFlag::HOST_TIMEOUT, event_writer);
    if (event_writer != nullptr) {
      send_event(*event_writer, 0, EventType::HOST_TIMEOUT, EventSeverity::ERROR,
                 static_cast<uint16_t>(active_config_.safety.velocity_timeout_action),
                 active_config_.safety.host_timeout_ms);
    }
    if (active_config_.safety.velocity_timeout_action == TimeoutAction::DISABLE) {
      if (backend_.emergency_stop != nullptr) {
        backend_.emergency_stop(backend_.context);
      }
      if (backend_.enable_driver != nullptr) {
        backend_.enable_driver(backend_.context, false);
      }
      driver_enabled_ = false;
      control_mode_ = ControlMode::DISABLED;
      transition_to(ActuatorState::FAULTED, event_writer);
    } else {
      if (backend_.ramp_stop != nullptr) {
        backend_.ramp_stop(backend_.context, active_config_.motion.default_stop_decel_sps2);
      }
      transition_to(ActuatorState::STOPPING, event_writer);
      control_mode_ = ControlMode::POSITION;
    }
  }
}

bool Actuator::should_send_telemetry() const {
  if (!active_config_.telemetry.binary_stream_enabled || active_config_.telemetry.interval_ms == 0) {
    return false;
  }
  const uint32_t now = now_ms();
  return static_cast<uint32_t>(now - last_telemetry_ms_) >= active_config_.telemetry.interval_ms;
}

void Actuator::mark_telemetry_sent() {
  last_telemetry_ms_ = now_ms();
}

void Actuator::send_telemetry(const ResponseWriter& writer, uint16_t seq) {
  send_status(writer, seq, true);
  mark_telemetry_sent();
}

void Actuator::record_frame_error(ErrorCode error) {
  counters_.decode_errors++;
  if (error == ErrorCode::ERR_BAD_CRC) {
    counters_.crc_errors++;
    set_fault(FaultFlag::BAD_CRC);
  } else if (error == ErrorCode::ERR_BAD_PAYLOAD) {
    set_fault(FaultFlag::BAD_PAYLOAD);
  } else {
    set_fault(FaultFlag::RX_FRAME);
  }
}

void Actuator::handle_packet(const Packet& request, const ResponseWriter& downstream_writer) {
  if (last_request_valid_ && request.seq == last_request_.seq && request.msg_id == last_request_.msg_id) {
    last_host_ms_ = now_ms();
    counters_.duplicate_packets++;
    if (same_request(request)) {
      replay_last_response(downstream_writer);
    } else {
      send_error(downstream_writer, request.seq, static_cast<MsgId>(request.msg_id),
                 ErrorCode::ERR_DUPLICATE_MISMATCH);
    }
    return;
  }

  Packet cached_responses[4]{};
  CachedWriterContext cached_context{};
  cached_context.downstream = downstream_writer;
  cached_context.responses = cached_responses;
  cached_context.capacity = static_cast<uint8_t>(sizeof(cached_responses) / sizeof(cached_responses[0]));

  ResponseWriter writer{};
  writer.context = &cached_context;
  writer.send = cached_send;

  counters_.command_count++;
  last_host_ms_ = now_ms();

  auto dispatch = [&]() {
  const MsgId msg = static_cast<MsgId>(request.msg_id);
  switch (msg) {
    case MsgId::PING:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_ack(writer, request.seq, msg);
      {
        Packet response{};
        response.msg_id = msg_u8(MsgId::PONG);
        response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
        response.seq = request.seq;
        send_response(writer, response);
      }
      return;

    case MsgId::GET_IDENTITY:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_identity(writer, request.seq);
      return;

    case MsgId::GET_CAPABILITIES:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_capabilities(writer, request.seq);
      return;

    case MsgId::GET_STATUS:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_status(writer, request.seq, false);
      return;

    case MsgId::GET_FAULTS:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_faults(writer, request.seq);
      return;

    case MsgId::GET_COUNTERS:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_counters(writer, request.seq);
      return;

    case MsgId::HEARTBEAT:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::SET_CONTROL_MODE:
      if (request.payload_len != 1) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      if (request.payload[0] > mode_u8(ControlMode::COMMISSIONING)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (state_ == ActuatorState::FAULTED && request.payload[0] != mode_u8(ControlMode::DISABLED)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_FAULTED);
        return;
      }
      if (is_motion_state()) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BUSY);
        return;
      }
      control_mode_ = static_cast<ControlMode>(request.payload[0]);
      if (control_mode_ == ControlMode::DISABLED) {
        if (backend_.enable_driver != nullptr) {
          backend_.enable_driver(backend_.context, false);
        }
        driver_enabled_ = false;
        transition_to(ActuatorState::DISABLED, &downstream_writer);
      } else if (control_mode_ == ControlMode::COMMISSIONING) {
        if (backend_.enable_driver != nullptr && !backend_.enable_driver(backend_.context, true)) {
          set_fault(FaultFlag::DRIVER, &downstream_writer);
          send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
          return;
        }
        driver_enabled_ = true;
        transition_to(ActuatorState::COMMISSIONING, &downstream_writer);
      } else {
        if (backend_.enable_driver != nullptr && !backend_.enable_driver(backend_.context, true)) {
          set_fault(FaultFlag::DRIVER, &downstream_writer);
          send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
          return;
        }
        driver_enabled_ = true;
        transition_to(ActuatorState::IDLE, &downstream_writer);
      }
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::ESTOP:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      if (backend_.emergency_stop != nullptr) {
        backend_.emergency_stop(backend_.context);
      }
      if (backend_.enable_driver != nullptr) {
        backend_.enable_driver(backend_.context, false);
      }
      driver_enabled_ = false;
      control_mode_ = ControlMode::DISABLED;
      transition_to(ActuatorState::FAULTED, &downstream_writer);
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::RAMP_STOP:
      if (request.payload_len != 4) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      if (!is_motion_state() &&
          (backend_.motion_active == nullptr || !backend_.motion_active(backend_.context))) {
        send_ack(writer, request.seq, msg);
        return;
      }
      if (backend_.ramp_stop == nullptr || !backend_.ramp_stop(backend_.context, read_u32(request.payload))) {
        set_fault(FaultFlag::DRIVER, &downstream_writer);
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
        return;
      }
      transition_to(ActuatorState::STOPPING, &downstream_writer);
      control_mode_ = ControlMode::POSITION;
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::CLEAR_FAULTS:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      {
        const uint32_t previous_faults = faults_;
        faults_ = 0;
        if (previous_faults != 0) {
          send_event(downstream_writer, request.seq, EventType::FAULT_CLEARED, EventSeverity::INFO, 0,
                     previous_faults);
        }
      }
      if (!driver_enabled_) {
        transition_to(ActuatorState::DISABLED, &downstream_writer);
        control_mode_ = ControlMode::DISABLED;
      } else {
        transition_to(ActuatorState::IDLE, &downstream_writer);
      }
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::CONFIG_QUERY: {
      uint16_t raw_field = 0;
      if (!payload_u16(request, &raw_field)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_config_value(writer, request.seq, static_cast<ConfigField>(raw_field));
      return;
    }

    case MsgId::CONFIG_STAGE_FIELD: {
      ErrorCode code = stage_field(request, &downstream_writer);
      if (code != ErrorCode::OK) {
        send_error(writer, request.seq, msg, code);
        return;
      }
      send_ack(writer, request.seq, msg);
      return;
    }

    case MsgId::CONFIG_APPLY: {
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      ErrorCode code = apply_staged_config(&downstream_writer);
      if (code != ErrorCode::OK) {
        send_error(writer, request.seq, msg, code);
        return;
      }
      send_ack(writer, request.seq, msg);
      return;
    }

    case MsgId::CONFIG_SAVE:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      if (backend_.save_config == nullptr || !backend_.save_config(backend_.context, active_config_)) {
        set_fault(FaultFlag::CONFIG_STORAGE, &downstream_writer);
        send_error(writer, request.seq, msg, ErrorCode::ERR_STORAGE);
        return;
      }
      ++config_generation_;
      send_event(downstream_writer, request.seq, EventType::CONFIG_SAVED, EventSeverity::INFO, CONFIG_VERSION,
                 config_generation_);
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::CONFIG_RESET_DEFAULTS:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      if (is_motion_state()) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BUSY);
        return;
      }
      active_config_ = defaults_;
      staged_config_ = defaults_;
      config_staged_ = false;
      if (backend_.configure_driver != nullptr &&
          !backend_.configure_driver(backend_.context, active_config_.driver)) {
        set_fault(FaultFlag::DRIVER, &downstream_writer);
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
        return;
      }
      if (backend_.reset_config_storage != nullptr && !backend_.reset_config_storage(backend_.context)) {
        set_fault(FaultFlag::CONFIG_STORAGE, &downstream_writer);
        send_error(writer, request.seq, msg, ErrorCode::ERR_STORAGE);
        return;
      }
      send_event(downstream_writer, request.seq, EventType::CONFIG_APPLIED, EventSeverity::INFO, CONFIG_VERSION,
                 config_generation_);
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::REBOOT:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_ack(writer, request.seq, msg);
      if (backend_.reboot != nullptr) {
        backend_.reboot(backend_.context);
      }
      return;

    case MsgId::GET_DRIVER_STATUS:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      send_driver_status(writer, request.seq);
      return;

    case MsgId::MOVE_ABS: {
      if (request.payload_len != 12) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      ErrorCode allowed = validate_motion_allowed(ControlMode::POSITION);
      if (allowed != ErrorCode::OK) {
        send_error(writer, request.seq, msg, allowed);
        return;
      }
      const int32_t target = read_i32(request.payload);
      const uint32_t velocity = read_u32(request.payload + 4);
      const uint32_t accel = read_u32(request.payload + 8);
      if (velocity == 0 || accel == 0 || velocity > active_config_.safety.max_allowed_velocity_sps ||
          accel > active_config_.safety.max_allowed_accel_sps2 ||
          velocity > active_config_.motion.max_velocity_sps ||
          accel > active_config_.motion.max_accel_sps2) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (soft_limit_violation(active_config_, target)) {
        set_fault(FaultFlag::SOFT_LIMIT, &downstream_writer);
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      int32_t backend_target = 0;
      if (!to_backend_value(target, &backend_target)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (backend_.move_absolute == nullptr ||
          !backend_.move_absolute(backend_.context, backend_target, velocity, accel)) {
        set_fault(FaultFlag::DRIVER, &downstream_writer);
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
        return;
      }
      transition_to(ActuatorState::MOVING_POSITION, &downstream_writer);
      control_mode_ = ControlMode::POSITION;
      send_ack(writer, request.seq, msg);
      return;
    }

    case MsgId::MOVE_REL: {
      if (request.payload_len != 12) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      ErrorCode allowed = validate_motion_allowed(ControlMode::POSITION);
      if (allowed != ErrorCode::OK) {
        send_error(writer, request.seq, msg, allowed);
        return;
      }
      const int32_t delta = read_i32(request.payload);
      const uint32_t velocity = read_u32(request.payload + 4);
      const uint32_t accel = read_u32(request.payload + 8);
      const int32_t current = logical_current_position();
      const int64_t target64 = static_cast<int64_t>(current) + static_cast<int64_t>(delta);
      if (target64 < INT32_MIN || target64 > INT32_MAX || velocity == 0 || accel == 0 ||
          velocity > active_config_.safety.max_allowed_velocity_sps ||
          accel > active_config_.safety.max_allowed_accel_sps2 ||
          velocity > active_config_.motion.max_velocity_sps ||
          accel > active_config_.motion.max_accel_sps2) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (soft_limit_violation(active_config_, static_cast<int32_t>(target64))) {
        set_fault(FaultFlag::SOFT_LIMIT, &downstream_writer);
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      int32_t backend_delta = 0;
      if (!to_backend_value(delta, &backend_delta)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (backend_.move_relative == nullptr ||
          !backend_.move_relative(backend_.context, backend_delta, velocity, accel)) {
        set_fault(FaultFlag::DRIVER, &downstream_writer);
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
        return;
      }
      transition_to(ActuatorState::MOVING_POSITION, &downstream_writer);
      control_mode_ = ControlMode::POSITION;
      send_ack(writer, request.seq, msg);
      return;
    }

    case MsgId::RUN_VEL: {
      if (request.payload_len != 8) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      ErrorCode allowed = validate_motion_allowed(ControlMode::VELOCITY);
      if (allowed != ErrorCode::OK) {
        send_error(writer, request.seq, msg, allowed);
        return;
      }
      const int32_t velocity = read_i32(request.payload);
      const uint32_t accel = read_u32(request.payload + 4);
      const uint32_t abs_velocity = velocity < 0 ? static_cast<uint32_t>(-static_cast<int64_t>(velocity))
                                                 : static_cast<uint32_t>(velocity);
      if (velocity == 0 || accel == 0 || abs_velocity > active_config_.safety.max_allowed_velocity_sps ||
          accel > active_config_.safety.max_allowed_accel_sps2 ||
          abs_velocity > active_config_.motion.max_velocity_sps ||
          accel > active_config_.motion.max_accel_sps2) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      int32_t backend_velocity = 0;
      if (!to_backend_value(velocity, &backend_velocity)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (backend_.run_velocity == nullptr || !backend_.run_velocity(backend_.context, backend_velocity, accel)) {
        set_fault(FaultFlag::DRIVER, &downstream_writer);
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
        return;
      }
      transition_to(ActuatorState::RUNNING_VELOCITY, &downstream_writer);
      control_mode_ = ControlMode::VELOCITY;
      send_ack(writer, request.seq, msg);
      return;
    }

    case MsgId::SET_TELEM_RATE:
      if (request.payload_len != 2) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      active_config_.telemetry.interval_ms = read_u16(request.payload);
      if (config_staged_) {
        staged_config_.telemetry.interval_ms = active_config_.telemetry.interval_ms;
      } else {
        staged_config_ = active_config_;
      }
      send_ack(writer, request.seq, msg);
      return;

    default:
      set_fault(FaultFlag::UNSUPPORTED_COMMAND, &downstream_writer);
      send_error(writer, request.seq, msg, ErrorCode::ERR_UNSUPPORTED);
      return;
  }
  };

  dispatch();
  remember_request(request, cached_responses, cached_context.count);
}

void Actuator::send_ack(const ResponseWriter& writer, uint16_t seq, MsgId acked_msg, ErrorCode code) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::ACK);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 2;
  response.payload[0] = msg_u8(acked_msg);
  response.payload[1] = code_u8(code);
  send_response(writer, response);
}

void Actuator::send_error(const ResponseWriter& writer, uint16_t seq, MsgId rejected_msg, ErrorCode code) {
  counters_.rejected_commands++;
  Packet response{};
  response.msg_id = msg_u8(MsgId::ERROR);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE) | static_cast<uint8_t>(PacketFlag::ERROR);
  response.seq = seq;
  response.payload_len = 2;
  response.payload[0] = msg_u8(rejected_msg);
  response.payload[1] = code_u8(code);
  send_response(writer, response);
}

void Actuator::send_status(const ResponseWriter& writer, uint16_t seq, bool telemetry) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::STATUS);
  response.flags = telemetry ? static_cast<uint8_t>(PacketFlag::TELEMETRY) : static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 32;
  response.payload[0] = state_u8(state_);
  response.payload[1] = mode_u8(control_mode_);
  response.payload[2] = driver_enabled_ ? 1 : 0;
  response.payload[3] = config_staged_ ? 1 : 0;
  write_u32(response.payload + 4, faults_);
  write_i32(response.payload + 8, logical_current_position());
  write_i32(response.payload + 12, logical_target_position());
  write_i32(response.payload + 16, logical_current_speed());
  write_u32(response.payload + 20, now_ms());
  write_u32(response.payload + 24, config_generation_);
  write_u32(response.payload + 28, counters_.command_count);
  send_response(writer, response);
}

void Actuator::send_identity(const ResponseWriter& writer, uint16_t seq) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::IDENTITY);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 64;
  response.payload[0] = PROTOCOL_VERSION;
  response.payload[1] = FIRMWARE_VERSION_MAJOR;
  response.payload[2] = FIRMWARE_VERSION_MINOR;
  response.payload[3] = FIRMWARE_VERSION_PATCH;
  response.payload[4] = 1;  // board family: ESP32-C6 V1
  response.payload[5] = 1;  // axis count
  write_u16(response.payload + 6, CONFIG_VERSION);
  write_u32(response.payload + 8, 0x54464144u);  // DAFT
  write_u32(response.payload + 12, config_generation_);
  response.payload[16] = BUILD_GIT_DIRTY ? 1 : 0;
  copy_ascii(response.payload + 20, 20, BUILD_GIT_SHA, &response.payload[17]);
  copy_ascii(response.payload + 40, 24, BUILD_DATE, &response.payload[18]);
  response.payload[19] = 0;
  send_response(writer, response);
}

void Actuator::send_capabilities(const ResponseWriter& writer, uint16_t seq) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::CAPABILITIES);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 16;
  response.payload[0] = 1;  // driver: TMC2209
  response.payload[1] = 0;  // homing unsupported in V1
  response.payload[2] = 0;  // encoder unsupported in V1
  response.payload[3] = 1;  // persistent config supported
  write_u16(response.payload + 4, static_cast<uint16_t>(MAX_PAYLOAD_SIZE));
  write_u16(response.payload + 6, 100);  // practical max telemetry Hz
  write_u32(response.payload + 8, 0x00000001u);  // feature flags: serial binary v2
  write_u32(response.payload + 12, active_config_.safety.host_timeout_ms);
  send_response(writer, response);
}

void Actuator::send_faults(const ResponseWriter& writer, uint16_t seq) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::FAULTS);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 4;
  write_u32(response.payload, faults_);
  send_response(writer, response);
}

void Actuator::send_counters(const ResponseWriter& writer, uint16_t seq) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::COUNTERS);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 28;
  write_u32(response.payload, counters_.crc_errors);
  write_u32(response.payload + 4, counters_.decode_errors);
  write_u32(response.payload + 8, counters_.dropped_packets);
  write_u32(response.payload + 12, counters_.duplicate_packets);
  write_u32(response.payload + 16, counters_.command_count);
  write_u32(response.payload + 20, counters_.rejected_commands);
  write_u32(response.payload + 24, now_ms());
  send_response(writer, response);
}

void Actuator::send_driver_status(const ResponseWriter& writer, uint16_t seq) {
  DriverStatus status{};
  if (backend_.driver_status != nullptr) {
    status = backend_.driver_status(backend_.context);
  }
  if (status.configured != 0 && status.uart_ok == 0) {
    set_fault(FaultFlag::DRIVER_UART, &writer);
    send_event(writer, seq, EventType::DRIVER_UART_FAILURE, EventSeverity::ERROR, 0, status.raw_status);
  }

  Packet response{};
  response.msg_id = msg_u8(MsgId::DRIVER_STATUS);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 16;
  response.payload[0] = status.configured;
  response.payload[1] = status.uart_ok;
  response.payload[2] = status.overtemperature_pre_warning;
  response.payload[3] = status.overtemperature;
  write_u16(response.payload + 4, status.run_current_ma);
  write_u16(response.payload + 6, status.microsteps);
  write_u32(response.payload + 8, status.raw_status);
  write_u32(response.payload + 12, 0);
  send_response(writer, response);
}

void Actuator::send_config_value(const ResponseWriter& writer, uint16_t seq, ConfigField field) {
  int32_t value = 0;
  if (!config_field_value(active_config_, field, &value)) {
    send_error(writer, seq, MsgId::CONFIG_QUERY, ErrorCode::ERR_INVALID_FIELD);
    return;
  }

  Packet response{};
  response.msg_id = msg_u8(MsgId::CONFIG_VALUE);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 8;
  write_u16(response.payload, static_cast<uint16_t>(field));
  response.payload[2] = static_cast<uint8_t>(config_field_mutability(field));
  response.payload[3] = config_staged_ ? 1 : 0;
  write_i32(response.payload + 4, value);
  send_response(writer, response);
}

void Actuator::send_event(const ResponseWriter& writer, uint16_t seq, EventType event, EventSeverity severity,
                          uint16_t detail, uint32_t value) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::EVENT);
  response.flags = 0;
  response.seq = seq;
  response.payload_len = 12;
  response.payload[0] = static_cast<uint8_t>(event);
  response.payload[1] = static_cast<uint8_t>(severity);
  write_u16(response.payload + 2, detail);
  write_u32(response.payload + 4, value);
  write_u32(response.payload + 8, now_ms());
  send_response(writer, response);
}

ErrorCode Actuator::stage_field(const Packet& request, const ResponseWriter* event_writer) {
  if (request.payload_len != 6) {
    return ErrorCode::ERR_BAD_PAYLOAD;
  }

  const ConfigField field = static_cast<ConfigField>(read_u16(request.payload));
  const int32_t value = read_i32(request.payload + 2);
  if (!config_field_known(field)) {
    return ErrorCode::ERR_INVALID_FIELD;
  }

  const ConfigMutability mutability = config_field_mutability(field);
  if (mutability == ConfigMutability::COMPILE_TIME_ONLY || mutability == ConfigMutability::RESERVED) {
    return ErrorCode::ERR_UNSUPPORTED;
  }
  if (mutability == ConfigMutability::REQUIRES_REBOOT) {
    return ErrorCode::ERR_REQUIRES_REBOOT;
  }

  ActuatorConfig candidate = config_staged_ ? staged_config_ : active_config_;
  const ConfigValidationResult validation = stage_config_field(candidate, field, value);
  if (!validation.ok) {
    return ErrorCode::ERR_OUT_OF_RANGE;
  }

  staged_config_ = candidate;
  config_staged_ = true;
  if (!is_motion_state()) {
    transition_to(ActuatorState::CONFIG_STAGED, event_writer);
  }
  return ErrorCode::OK;
}

ErrorCode Actuator::apply_staged_config(const ResponseWriter* event_writer) {
  if (!config_staged_) {
    return ErrorCode::OK;
  }
  if (is_motion_state()) {
    return ErrorCode::ERR_BUSY;
  }

  const ConfigValidationResult validation = validate_config(staged_config_);
  if (!validation.ok) {
    set_fault(FaultFlag::INVALID_CONFIG, event_writer);
    return ErrorCode::ERR_OUT_OF_RANGE;
  }

  const bool driver_changed =
      staged_config_.driver.run_current_ma != active_config_.driver.run_current_ma ||
      staged_config_.driver.hold_current_ma != active_config_.driver.hold_current_ma ||
      staged_config_.driver.microsteps != active_config_.driver.microsteps ||
      staged_config_.driver.mode != active_config_.driver.mode;

  if (driver_changed && backend_.configure_driver != nullptr &&
      !backend_.configure_driver(backend_.context, staged_config_.driver)) {
    set_fault(FaultFlag::DRIVER, event_writer);
    return ErrorCode::ERR_BAD_STATE;
  }

  active_config_ = staged_config_;
  config_staged_ = false;
  transition_to(driver_enabled_ ? ActuatorState::IDLE : ActuatorState::DISABLED, event_writer);
  if (event_writer != nullptr) {
    send_event(*event_writer, 0, EventType::CONFIG_APPLIED, EventSeverity::INFO, CONFIG_VERSION,
               config_generation_);
  }
  return ErrorCode::OK;
}

ErrorCode Actuator::validate_motion_allowed(ControlMode requested_mode) const {
  if (state_ == ActuatorState::FAULTED) {
    return ErrorCode::ERR_FAULTED;
  }
  if (!driver_enabled_) {
    return ErrorCode::ERR_BAD_STATE;
  }
  if (config_staged_) {
    return ErrorCode::ERR_BAD_STATE;
  }
  if (requested_mode == ControlMode::POSITION && state_ == ActuatorState::RUNNING_VELOCITY) {
    return ErrorCode::ERR_BUSY;
  }
  if (requested_mode == ControlMode::VELOCITY && state_ == ActuatorState::MOVING_POSITION) {
    return ErrorCode::ERR_BUSY;
  }
  if (state_ == ActuatorState::STOPPING) {
    return ErrorCode::ERR_BUSY;
  }
  return ErrorCode::OK;
}

bool Actuator::is_motion_state() const {
  return state_ == ActuatorState::MOVING_POSITION || state_ == ActuatorState::RUNNING_VELOCITY ||
         state_ == ActuatorState::STOPPING;
}

uint32_t Actuator::now_ms() const {
  return backend_.millis != nullptr ? backend_.millis(backend_.context) : 0;
}

bool Actuator::send_response(const ResponseWriter& writer, const Packet& packet) {
  if (!send_packet(writer, packet)) {
    counters_.dropped_packets++;
    return false;
  }
  return true;
}

bool Actuator::same_request(const Packet& request) const {
  if (!last_request_valid_ || request.msg_id != last_request_.msg_id || request.flags != last_request_.flags ||
      request.seq != last_request_.seq || request.payload_len != last_request_.payload_len) {
    return false;
  }
  return request.payload_len == 0 || memcmp(request.payload, last_request_.payload, request.payload_len) == 0;
}

void Actuator::remember_request(const Packet& request, const Packet* responses, uint8_t response_count) {
  last_request_ = request;
  last_response_count_ = response_count > 4 ? 4 : response_count;
  for (uint8_t i = 0; i < last_response_count_; ++i) {
    last_responses_[i] = responses[i];
  }
  last_request_valid_ = true;
}

void Actuator::replay_last_response(const ResponseWriter& writer) {
  for (uint8_t i = 0; i < last_response_count_; ++i) {
    send_response(writer, last_responses_[i]);
  }
}

bool Actuator::to_backend_value(int32_t logical, int32_t* backend_value) const {
  if (!active_config_.calibration.direction_inverted) {
    *backend_value = logical;
    return true;
  }
  if (logical == INT32_MIN) {
    return false;
  }
  *backend_value = -logical;
  return true;
}

int32_t Actuator::from_backend_value(int32_t backend_value) const {
  if (!active_config_.calibration.direction_inverted) {
    return backend_value;
  }
  return backend_value == INT32_MIN ? INT32_MAX : -backend_value;
}

int32_t Actuator::logical_current_position() const {
  return from_backend_value(backend_.current_position != nullptr ? backend_.current_position(backend_.context) : 0);
}

int32_t Actuator::logical_target_position() const {
  return from_backend_value(backend_.target_position != nullptr ? backend_.target_position(backend_.context) : 0);
}

int32_t Actuator::logical_current_speed() const {
  return from_backend_value(backend_.current_speed != nullptr ? backend_.current_speed(backend_.context) : 0);
}

void Actuator::set_fault(FaultFlag flag, const ResponseWriter* event_writer) {
  const uint32_t mask = fault_value(flag);
  const bool newly_set = (faults_ & mask) == 0;
  faults_ |= mask;
  if (newly_set && event_writer != nullptr) {
    send_event(*event_writer, 0, EventType::FAULT_SET, EventSeverity::ERROR, fault_bit(flag), faults_);
  }
  if (flag != FaultFlag::HOST_TIMEOUT && flag != FaultFlag::SOFT_LIMIT) {
    transition_to(ActuatorState::FAULTED, event_writer);
  }
}

void Actuator::transition_to(ActuatorState state, const ResponseWriter* event_writer) {
  if (state_ == state) {
    return;
  }
  const ActuatorState previous = state_;
  state_ = state;
  if (event_writer != nullptr) {
    send_event(*event_writer, 0, EventType::STATE_CHANGED, EventSeverity::INFO,
               static_cast<uint16_t>(previous), static_cast<uint32_t>(state));
  }
}

}  // namespace daft
