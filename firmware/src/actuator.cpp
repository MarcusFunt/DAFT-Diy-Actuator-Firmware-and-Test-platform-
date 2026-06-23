#include "daft/actuator.hpp"

#include <string.h>

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
  last_host_ms_ = now_ms();
  last_telemetry_ms_ = now_ms();

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
    state_ = ActuatorState::FAULTED;
    return;
  }

  if (backend_.enable_driver != nullptr) {
    backend_.enable_driver(backend_.context, false);
  }
  state_ = ActuatorState::DISABLED;
}

void Actuator::update() {
  const uint32_t now = now_ms();

  if ((state_ == ActuatorState::MOVING_POSITION || state_ == ActuatorState::STOPPING) &&
      backend_.motion_active != nullptr && !backend_.motion_active(backend_.context)) {
    state_ = driver_enabled_ ? ActuatorState::IDLE : ActuatorState::DISABLED;
    control_mode_ = driver_enabled_ ? ControlMode::POSITION : ControlMode::DISABLED;
  }

  if (state_ == ActuatorState::RUNNING_VELOCITY && active_config_.safety.host_timeout_ms > 0 &&
      static_cast<uint32_t>(now - last_host_ms_) > active_config_.safety.host_timeout_ms) {
    set_fault(FaultFlag::HOST_TIMEOUT);
    if (active_config_.safety.velocity_timeout_action == TimeoutAction::DISABLE) {
      if (backend_.emergency_stop != nullptr) {
        backend_.emergency_stop(backend_.context);
      }
      if (backend_.enable_driver != nullptr) {
        backend_.enable_driver(backend_.context, false);
      }
      driver_enabled_ = false;
      control_mode_ = ControlMode::DISABLED;
      state_ = ActuatorState::FAULTED;
    } else {
      if (backend_.ramp_stop != nullptr) {
        backend_.ramp_stop(backend_.context, active_config_.motion.default_stop_decel_sps2);
      }
      state_ = ActuatorState::STOPPING;
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
  } else {
    set_fault(FaultFlag::RX_FRAME);
  }
}

void Actuator::handle_packet(const Packet& request, const ResponseWriter& writer) {
  counters_.command_count++;
  last_host_ms_ = now_ms();

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
        send_packet(writer, response);
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
      send_faults(writer, request.seq);
      return;

    case MsgId::GET_COUNTERS:
      send_counters(writer, request.seq);
      return;

    case MsgId::HEARTBEAT:
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::SET_CONTROL_MODE:
      if (request.payload_len != 1) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
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
        state_ = ActuatorState::DISABLED;
      } else if (control_mode_ == ControlMode::COMMISSIONING) {
        if (backend_.enable_driver != nullptr && !backend_.enable_driver(backend_.context, true)) {
          set_fault(FaultFlag::DRIVER);
          send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
          return;
        }
        driver_enabled_ = true;
        state_ = ActuatorState::COMMISSIONING;
      } else {
        if (backend_.enable_driver != nullptr && !backend_.enable_driver(backend_.context, true)) {
          set_fault(FaultFlag::DRIVER);
          send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
          return;
        }
        driver_enabled_ = true;
        state_ = ActuatorState::IDLE;
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
      state_ = ActuatorState::FAULTED;
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::RAMP_STOP:
      if (request.payload_len != 4) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      if (backend_.ramp_stop == nullptr || !backend_.ramp_stop(backend_.context, read_u32(request.payload))) {
        set_fault(FaultFlag::DRIVER);
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
        return;
      }
      state_ = ActuatorState::STOPPING;
      control_mode_ = ControlMode::POSITION;
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::CLEAR_FAULTS:
      if (!payload_empty(request)) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      faults_ = 0;
      if (!driver_enabled_) {
        state_ = ActuatorState::DISABLED;
        control_mode_ = ControlMode::DISABLED;
      } else {
        state_ = ActuatorState::IDLE;
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
      ErrorCode code = stage_field(request);
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
      ErrorCode code = apply_staged_config();
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
        set_fault(FaultFlag::CONFIG_STORAGE);
        send_error(writer, request.seq, msg, ErrorCode::ERR_STORAGE);
        return;
      }
      ++config_generation_;
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
      if (backend_.configure_driver != nullptr) {
        backend_.configure_driver(backend_.context, active_config_.driver);
      }
      if (backend_.reset_config_storage != nullptr) {
        backend_.reset_config_storage(backend_.context);
      }
      send_ack(writer, request.seq, msg);
      return;

    case MsgId::REBOOT:
      send_ack(writer, request.seq, msg);
      if (backend_.reboot != nullptr) {
        backend_.reboot(backend_.context);
      }
      return;

    case MsgId::GET_DRIVER_STATUS:
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
          accel > active_config_.safety.max_allowed_accel_sps2) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (soft_limit_violation(active_config_, target)) {
        set_fault(FaultFlag::SOFT_LIMIT);
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (backend_.move_absolute == nullptr || !backend_.move_absolute(backend_.context, target, velocity, accel)) {
        set_fault(FaultFlag::DRIVER);
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
        return;
      }
      state_ = ActuatorState::MOVING_POSITION;
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
      const int32_t current = backend_.current_position != nullptr ? backend_.current_position(backend_.context) : 0;
      const int64_t target64 = static_cast<int64_t>(current) + static_cast<int64_t>(delta);
      if (target64 < INT32_MIN || target64 > INT32_MAX || velocity == 0 || accel == 0 ||
          velocity > active_config_.safety.max_allowed_velocity_sps ||
          accel > active_config_.safety.max_allowed_accel_sps2) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (soft_limit_violation(active_config_, static_cast<int32_t>(target64))) {
        set_fault(FaultFlag::SOFT_LIMIT);
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (backend_.move_relative == nullptr || !backend_.move_relative(backend_.context, delta, velocity, accel)) {
        set_fault(FaultFlag::DRIVER);
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
        return;
      }
      state_ = ActuatorState::MOVING_POSITION;
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
          accel > active_config_.safety.max_allowed_accel_sps2) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_OUT_OF_RANGE);
        return;
      }
      if (backend_.run_velocity == nullptr || !backend_.run_velocity(backend_.context, velocity, accel)) {
        set_fault(FaultFlag::DRIVER);
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_STATE);
        return;
      }
      state_ = ActuatorState::RUNNING_VELOCITY;
      control_mode_ = ControlMode::VELOCITY;
      send_ack(writer, request.seq, msg);
      return;
    }

    case MsgId::SET_TELEM_RATE:
      if (request.payload_len != 2) {
        send_error(writer, request.seq, msg, ErrorCode::ERR_BAD_PAYLOAD);
        return;
      }
      staged_config_ = active_config_;
      staged_config_.telemetry.interval_ms = read_u16(request.payload);
      active_config_.telemetry.interval_ms = staged_config_.telemetry.interval_ms;
      send_ack(writer, request.seq, msg);
      return;

    default:
      counters_.rejected_commands++;
      set_fault(FaultFlag::UNSUPPORTED_COMMAND);
      send_error(writer, request.seq, msg, ErrorCode::ERR_UNSUPPORTED);
      return;
  }
}

void Actuator::send_ack(const ResponseWriter& writer, uint16_t seq, MsgId acked_msg, ErrorCode code) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::ACK);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 2;
  response.payload[0] = msg_u8(acked_msg);
  response.payload[1] = code_u8(code);
  send_packet(writer, response);
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
  send_packet(writer, response);
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
  write_i32(response.payload + 8, backend_.current_position != nullptr ? backend_.current_position(backend_.context) : 0);
  write_i32(response.payload + 12, backend_.target_position != nullptr ? backend_.target_position(backend_.context) : 0);
  write_i32(response.payload + 16, backend_.current_speed != nullptr ? backend_.current_speed(backend_.context) : 0);
  write_u32(response.payload + 20, now_ms());
  write_u32(response.payload + 24, config_generation_);
  write_u32(response.payload + 28, counters_.command_count);
  send_packet(writer, response);
}

void Actuator::send_identity(const ResponseWriter& writer, uint16_t seq) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::IDENTITY);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 16;
  response.payload[0] = PROTOCOL_VERSION;
  response.payload[1] = 1;  // firmware major
  response.payload[2] = 0;  // firmware minor
  response.payload[3] = 0;  // firmware patch
  response.payload[4] = 1;  // board family: ESP32-C6 V1
  response.payload[5] = 1;  // axis count
  write_u16(response.payload + 6, CONFIG_VERSION);
  write_u32(response.payload + 8, 0x54464144u);  // DAFT
  write_u32(response.payload + 12, config_generation_);
  send_packet(writer, response);
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
  send_packet(writer, response);
}

void Actuator::send_faults(const ResponseWriter& writer, uint16_t seq) {
  Packet response{};
  response.msg_id = msg_u8(MsgId::FAULTS);
  response.flags = static_cast<uint8_t>(PacketFlag::RESPONSE);
  response.seq = seq;
  response.payload_len = 4;
  write_u32(response.payload, faults_);
  send_packet(writer, response);
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
  send_packet(writer, response);
}

void Actuator::send_driver_status(const ResponseWriter& writer, uint16_t seq) {
  DriverStatus status{};
  if (backend_.driver_status != nullptr) {
    status = backend_.driver_status(backend_.context);
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
  send_packet(writer, response);
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
  send_packet(writer, response);
}

ErrorCode Actuator::stage_field(const Packet& request) {
  if (request.payload_len != 6) {
    return ErrorCode::ERR_BAD_PAYLOAD;
  }

  const ConfigField field = static_cast<ConfigField>(read_u16(request.payload));
  const int32_t value = read_i32(request.payload + 2);
  if (!config_field_known(field)) {
    return ErrorCode::ERR_INVALID_FIELD;
  }

  const ConfigMutability mutability = config_field_mutability(field);
  if (mutability == ConfigMutability::COMPILE_TIME_ONLY) {
    return ErrorCode::ERR_UNSUPPORTED;
  }
  if (mutability == ConfigMutability::REQUIRES_REBOOT) {
    return ErrorCode::ERR_REQUIRES_REBOOT;
  }

  staged_config_ = config_staged_ ? staged_config_ : active_config_;
  const ConfigValidationResult validation = stage_config_field(staged_config_, field, value);
  if (!validation.ok) {
    staged_config_ = active_config_;
    return ErrorCode::ERR_OUT_OF_RANGE;
  }

  config_staged_ = true;
  if (!is_motion_state()) {
    state_ = ActuatorState::CONFIG_STAGED;
  }
  return ErrorCode::OK;
}

ErrorCode Actuator::apply_staged_config() {
  if (!config_staged_) {
    return ErrorCode::OK;
  }
  if (is_motion_state()) {
    return ErrorCode::ERR_BUSY;
  }

  const ConfigValidationResult validation = validate_config(staged_config_);
  if (!validation.ok) {
    set_fault(FaultFlag::INVALID_CONFIG);
    return ErrorCode::ERR_OUT_OF_RANGE;
  }

  const bool driver_changed =
      staged_config_.driver.run_current_ma != active_config_.driver.run_current_ma ||
      staged_config_.driver.hold_current_ma != active_config_.driver.hold_current_ma ||
      staged_config_.driver.microsteps != active_config_.driver.microsteps ||
      staged_config_.driver.mode != active_config_.driver.mode;

  if (driver_changed && backend_.configure_driver != nullptr &&
      !backend_.configure_driver(backend_.context, staged_config_.driver)) {
    set_fault(FaultFlag::DRIVER);
    return ErrorCode::ERR_BAD_STATE;
  }

  active_config_ = staged_config_;
  config_staged_ = false;
  state_ = driver_enabled_ ? ActuatorState::IDLE : ActuatorState::DISABLED;
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

void Actuator::set_fault(FaultFlag flag) {
  faults_ |= fault_value(flag);
  if (flag != FaultFlag::HOST_TIMEOUT && flag != FaultFlag::SOFT_LIMIT) {
    state_ = ActuatorState::FAULTED;
  }
}

}  // namespace daft
