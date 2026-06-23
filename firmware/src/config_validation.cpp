#include "daft/config.hpp"

#include <string.h>

#include "daft/protocol_v2.hpp"

namespace daft {

namespace {

bool supported_microsteps(uint16_t microsteps) {
  switch (microsteps) {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
    case 256:
      return true;
    default:
      return false;
  }
}

ConfigValidationResult fail(ConfigField field, const char* message) {
  ConfigValidationResult result;
  result.ok = false;
  result.field = field;
  result.message = message;
  return result;
}

ConfigValidationResult ok() {
  ConfigValidationResult result;
  result.ok = true;
  result.message = "ok";
  return result;
}

void write_bool(uint8_t* p, bool value) {
  *p = value ? 1 : 0;
}

bool read_bool(const uint8_t* p) {
  return *p != 0;
}

}  // namespace

ActuatorConfig default_config() {
  return ActuatorConfig{};
}

bool config_field_known(ConfigField field) {
  switch (field) {
    case ConfigField::TELEMETRY_INTERVAL_MS:
    case ConfigField::SOFT_LIMITS_ENABLED:
    case ConfigField::SOFT_LIMIT_MIN_STEPS:
    case ConfigField::SOFT_LIMIT_MAX_STEPS:
    case ConfigField::DEFAULT_MAX_VELOCITY_SPS:
    case ConfigField::DEFAULT_MAX_ACCEL_SPS2:
    case ConfigField::DEFAULT_STOP_DECEL_SPS2:
    case ConfigField::RUN_CURRENT_MA:
    case ConfigField::HOLD_CURRENT_MA:
    case ConfigField::MICROSTEPS:
    case ConfigField::DRIVER_MODE:
    case ConfigField::POSITION_OFFSET_STEPS:
    case ConfigField::HOME_POSITION_STEPS:
    case ConfigField::DIRECTION_INVERTED:
    case ConfigField::HOST_TIMEOUT_MS:
    case ConfigField::VELOCITY_TIMEOUT_ACTION:
    case ConfigField::ENABLE_TIMEOUT_MS:
    case ConfigField::MAX_ALLOWED_CURRENT_MA:
    case ConfigField::MAX_ALLOWED_VELOCITY_SPS:
    case ConfigField::MAX_ALLOWED_ACCEL_SPS2:
      return true;
    default:
      return false;
  }
}

ConfigMutability config_field_mutability(ConfigField field) {
  switch (field) {
    case ConfigField::TELEMETRY_INTERVAL_MS:
    case ConfigField::SOFT_LIMITS_ENABLED:
    case ConfigField::SOFT_LIMIT_MIN_STEPS:
    case ConfigField::SOFT_LIMIT_MAX_STEPS:
    case ConfigField::DEFAULT_MAX_VELOCITY_SPS:
    case ConfigField::DEFAULT_MAX_ACCEL_SPS2:
    case ConfigField::DEFAULT_STOP_DECEL_SPS2:
    case ConfigField::POSITION_OFFSET_STEPS:
    case ConfigField::HOME_POSITION_STEPS:
    case ConfigField::DIRECTION_INVERTED:
    case ConfigField::HOST_TIMEOUT_MS:
    case ConfigField::VELOCITY_TIMEOUT_ACTION:
    case ConfigField::ENABLE_TIMEOUT_MS:
    case ConfigField::MAX_ALLOWED_CURRENT_MA:
    case ConfigField::MAX_ALLOWED_VELOCITY_SPS:
    case ConfigField::MAX_ALLOWED_ACCEL_SPS2:
      return ConfigMutability::RUNTIME_SAFE;
    case ConfigField::RUN_CURRENT_MA:
    case ConfigField::HOLD_CURRENT_MA:
    case ConfigField::MICROSTEPS:
    case ConfigField::DRIVER_MODE:
      return ConfigMutability::REQUIRES_IDLE;
    default:
      return ConfigMutability::COMPILE_TIME_ONLY;
  }
}

ConfigValidationResult validate_config(const ActuatorConfig& config) {
  if (config.version != CONFIG_VERSION) {
    return fail(ConfigField::TELEMETRY_INTERVAL_MS, "unsupported config version");
  }
  if (config.motor.full_steps_per_rev == 0 || config.motor.gear_ratio_den == 0 || config.motor.units_per_rev == 0) {
    return fail(ConfigField::DEFAULT_MAX_VELOCITY_SPS, "invalid motor spec");
  }
  if (config.driver.run_current_ma == 0 || config.driver.run_current_ma > config.safety.max_allowed_current_ma ||
      config.driver.run_current_ma > config.motor.max_current_ma) {
    return fail(ConfigField::RUN_CURRENT_MA, "run current out of range");
  }
  if (config.driver.hold_current_ma > config.driver.run_current_ma) {
    return fail(ConfigField::HOLD_CURRENT_MA, "hold current exceeds run current");
  }
  if (!supported_microsteps(config.driver.microsteps)) {
    return fail(ConfigField::MICROSTEPS, "unsupported microsteps");
  }
  if (config.motion.max_velocity_sps == 0 || config.motion.max_velocity_sps > config.safety.max_allowed_velocity_sps) {
    return fail(ConfigField::DEFAULT_MAX_VELOCITY_SPS, "velocity out of range");
  }
  if (config.motion.max_accel_sps2 == 0 || config.motion.max_accel_sps2 > config.safety.max_allowed_accel_sps2) {
    return fail(ConfigField::DEFAULT_MAX_ACCEL_SPS2, "acceleration out of range");
  }
  if (config.motion.default_stop_decel_sps2 == 0 ||
      config.motion.default_stop_decel_sps2 > config.safety.max_allowed_accel_sps2) {
    return fail(ConfigField::DEFAULT_STOP_DECEL_SPS2, "stop decel out of range");
  }
  if (config.motion.soft_limits_enabled && config.motion.soft_min_steps > config.motion.soft_max_steps) {
    return fail(ConfigField::SOFT_LIMIT_MIN_STEPS, "soft min exceeds soft max");
  }
  if (config.safety.host_timeout_ms < 100 || config.safety.host_timeout_ms > 60000) {
    return fail(ConfigField::HOST_TIMEOUT_MS, "host timeout out of range");
  }
  return ok();
}

ConfigValidationResult stage_config_field(ActuatorConfig& staged, ConfigField field, int32_t value) {
  if (!config_field_known(field)) {
    return fail(field, "unknown config field");
  }

  switch (field) {
    case ConfigField::TELEMETRY_INTERVAL_MS:
      if (value < 0 || value > 65535) return fail(field, "telemetry interval out of range");
      staged.telemetry.interval_ms = static_cast<uint16_t>(value);
      break;
    case ConfigField::SOFT_LIMITS_ENABLED:
      staged.motion.soft_limits_enabled = value != 0;
      break;
    case ConfigField::SOFT_LIMIT_MIN_STEPS:
      staged.motion.soft_min_steps = value;
      break;
    case ConfigField::SOFT_LIMIT_MAX_STEPS:
      staged.motion.soft_max_steps = value;
      break;
    case ConfigField::DEFAULT_MAX_VELOCITY_SPS:
      if (value <= 0) return fail(field, "velocity must be positive");
      staged.motion.max_velocity_sps = static_cast<uint32_t>(value);
      break;
    case ConfigField::DEFAULT_MAX_ACCEL_SPS2:
      if (value <= 0) return fail(field, "acceleration must be positive");
      staged.motion.max_accel_sps2 = static_cast<uint32_t>(value);
      break;
    case ConfigField::DEFAULT_STOP_DECEL_SPS2:
      if (value <= 0) return fail(field, "stop decel must be positive");
      staged.motion.default_stop_decel_sps2 = static_cast<uint32_t>(value);
      break;
    case ConfigField::RUN_CURRENT_MA:
      if (value <= 0 || value > 65535) return fail(field, "run current out of range");
      staged.driver.run_current_ma = static_cast<uint16_t>(value);
      break;
    case ConfigField::HOLD_CURRENT_MA:
      if (value < 0 || value > 65535) return fail(field, "hold current out of range");
      staged.driver.hold_current_ma = static_cast<uint16_t>(value);
      break;
    case ConfigField::MICROSTEPS:
      if (value <= 0 || value > 256) return fail(field, "microsteps out of range");
      staged.driver.microsteps = static_cast<uint16_t>(value);
      break;
    case ConfigField::DRIVER_MODE:
      if (value < 0 || value > 1) return fail(field, "driver mode out of range");
      staged.driver.mode = static_cast<DriverMode>(value);
      break;
    case ConfigField::POSITION_OFFSET_STEPS:
      staged.calibration.position_offset_steps = value;
      break;
    case ConfigField::HOME_POSITION_STEPS:
      staged.calibration.home_position_steps = value;
      break;
    case ConfigField::DIRECTION_INVERTED:
      staged.calibration.direction_inverted = value != 0;
      break;
    case ConfigField::HOST_TIMEOUT_MS:
      if (value <= 0) return fail(field, "host timeout must be positive");
      staged.safety.host_timeout_ms = static_cast<uint32_t>(value);
      break;
    case ConfigField::VELOCITY_TIMEOUT_ACTION:
      if (value < 0 || value > 1) return fail(field, "timeout action out of range");
      staged.safety.velocity_timeout_action = static_cast<TimeoutAction>(value);
      break;
    case ConfigField::ENABLE_TIMEOUT_MS:
      if (value < 0) return fail(field, "enable timeout out of range");
      staged.safety.enable_timeout_ms = static_cast<uint32_t>(value);
      break;
    case ConfigField::MAX_ALLOWED_CURRENT_MA:
      if (value <= 0 || value > 65535) return fail(field, "max current out of range");
      staged.safety.max_allowed_current_ma = static_cast<uint16_t>(value);
      break;
    case ConfigField::MAX_ALLOWED_VELOCITY_SPS:
      if (value <= 0) return fail(field, "max velocity must be positive");
      staged.safety.max_allowed_velocity_sps = static_cast<uint32_t>(value);
      break;
    case ConfigField::MAX_ALLOWED_ACCEL_SPS2:
      if (value <= 0) return fail(field, "max accel must be positive");
      staged.safety.max_allowed_accel_sps2 = static_cast<uint32_t>(value);
      break;
    default:
      return fail(field, "field cannot be staged");
  }

  return validate_config(staged);
}

bool config_field_value(const ActuatorConfig& config, ConfigField field, int32_t* value) {
  switch (field) {
    case ConfigField::TELEMETRY_INTERVAL_MS:
      *value = config.telemetry.interval_ms;
      return true;
    case ConfigField::SOFT_LIMITS_ENABLED:
      *value = config.motion.soft_limits_enabled ? 1 : 0;
      return true;
    case ConfigField::SOFT_LIMIT_MIN_STEPS:
      *value = config.motion.soft_min_steps;
      return true;
    case ConfigField::SOFT_LIMIT_MAX_STEPS:
      *value = config.motion.soft_max_steps;
      return true;
    case ConfigField::DEFAULT_MAX_VELOCITY_SPS:
      *value = static_cast<int32_t>(config.motion.max_velocity_sps);
      return true;
    case ConfigField::DEFAULT_MAX_ACCEL_SPS2:
      *value = static_cast<int32_t>(config.motion.max_accel_sps2);
      return true;
    case ConfigField::DEFAULT_STOP_DECEL_SPS2:
      *value = static_cast<int32_t>(config.motion.default_stop_decel_sps2);
      return true;
    case ConfigField::RUN_CURRENT_MA:
      *value = config.driver.run_current_ma;
      return true;
    case ConfigField::HOLD_CURRENT_MA:
      *value = config.driver.hold_current_ma;
      return true;
    case ConfigField::MICROSTEPS:
      *value = config.driver.microsteps;
      return true;
    case ConfigField::DRIVER_MODE:
      *value = static_cast<int32_t>(config.driver.mode);
      return true;
    case ConfigField::POSITION_OFFSET_STEPS:
      *value = config.calibration.position_offset_steps;
      return true;
    case ConfigField::HOME_POSITION_STEPS:
      *value = config.calibration.home_position_steps;
      return true;
    case ConfigField::DIRECTION_INVERTED:
      *value = config.calibration.direction_inverted ? 1 : 0;
      return true;
    case ConfigField::HOST_TIMEOUT_MS:
      *value = static_cast<int32_t>(config.safety.host_timeout_ms);
      return true;
    case ConfigField::VELOCITY_TIMEOUT_ACTION:
      *value = static_cast<int32_t>(config.safety.velocity_timeout_action);
      return true;
    case ConfigField::ENABLE_TIMEOUT_MS:
      *value = static_cast<int32_t>(config.safety.enable_timeout_ms);
      return true;
    case ConfigField::MAX_ALLOWED_CURRENT_MA:
      *value = config.safety.max_allowed_current_ma;
      return true;
    case ConfigField::MAX_ALLOWED_VELOCITY_SPS:
      *value = static_cast<int32_t>(config.safety.max_allowed_velocity_sps);
      return true;
    case ConfigField::MAX_ALLOWED_ACCEL_SPS2:
      *value = static_cast<int32_t>(config.safety.max_allowed_accel_sps2);
      return true;
    default:
      return false;
  }
}

bool serialize_config(const ActuatorConfig& config, uint8_t* out, size_t capacity, size_t* length) {
  if (capacity < CONFIG_BLOB_MAX_SIZE || !validate_config(config).ok) {
    return false;
  }

  memset(out, 0, capacity);
  size_t offset = 0;
  auto put_u16 = [&](uint16_t value) {
    write_u16(out + offset, value);
    offset += 2;
  };
  auto put_u32 = [&](uint32_t value) {
    write_u32(out + offset, value);
    offset += 4;
  };
  auto put_i32 = [&](int32_t value) {
    write_i32(out + offset, value);
    offset += 4;
  };
  auto put_u8 = [&](uint8_t value) {
    out[offset++] = value;
  };

  put_u16(config.version);
  put_u16(static_cast<uint16_t>(config.pins.step_pin));
  put_u16(static_cast<uint16_t>(config.pins.dir_pin));
  put_u16(static_cast<uint16_t>(config.pins.enable_pin));
  put_u8(config.pins.enable_active_low ? 1 : 0);
  put_u16(static_cast<uint16_t>(config.pins.tmc_uart_rx_pin));
  put_u16(static_cast<uint16_t>(config.pins.tmc_uart_tx_pin));
  put_u32(config.pins.tmc_uart_baud);
  put_u8(config.pins.tmc_driver_address);
  put_u16(config.pins.sense_resistor_milliohm);
  put_u16(config.motor.full_steps_per_rev);
  put_u16(config.motor.gear_ratio_num);
  put_u16(config.motor.gear_ratio_den);
  put_u32(config.motor.units_per_rev);
  put_u16(config.motor.max_current_ma);
  put_u16(config.driver.run_current_ma);
  put_u16(config.driver.hold_current_ma);
  put_u16(config.driver.microsteps);
  put_u8(static_cast<uint8_t>(config.driver.mode));
  put_u8(config.driver.blank_time);
  put_u8(config.driver.toff);
  write_bool(out + offset++, config.driver.interpolation);
  put_u32(config.motion.max_velocity_sps);
  put_u32(config.motion.max_accel_sps2);
  put_i32(config.motion.max_travel_steps);
  write_bool(out + offset++, config.motion.soft_limits_enabled);
  put_i32(config.motion.soft_min_steps);
  put_i32(config.motion.soft_max_steps);
  put_u32(config.motion.default_stop_decel_sps2);
  put_u16(config.telemetry.interval_ms);
  put_u16(config.telemetry.enabled_fields);
  write_bool(out + offset++, config.telemetry.binary_stream_enabled);
  put_i32(config.calibration.position_offset_steps);
  put_i32(config.calibration.home_position_steps);
  write_bool(out + offset++, config.calibration.direction_inverted);
  put_i32(config.calibration.soft_limit_origin_steps);
  put_u32(config.safety.host_timeout_ms);
  put_u8(static_cast<uint8_t>(config.safety.velocity_timeout_action));
  put_u32(config.safety.enable_timeout_ms);
  put_u16(config.safety.max_allowed_current_ma);
  put_u32(config.safety.max_allowed_velocity_sps);
  put_u32(config.safety.max_allowed_accel_sps2);

  *length = offset;
  return true;
}

bool deserialize_config(const uint8_t* data, size_t length, ActuatorConfig* config) {
  if (length > CONFIG_BLOB_MAX_SIZE || length < 96) {
    return false;
  }

  ActuatorConfig decoded{};
  size_t offset = 0;
  auto get_u16 = [&]() {
    uint16_t value = read_u16(data + offset);
    offset += 2;
    return value;
  };
  auto get_u32 = [&]() {
    uint32_t value = read_u32(data + offset);
    offset += 4;
    return value;
  };
  auto get_i32 = [&]() {
    int32_t value = read_i32(data + offset);
    offset += 4;
    return value;
  };
  auto get_u8 = [&]() {
    return data[offset++];
  };

  decoded.version = get_u16();
  decoded.pins.step_pin = static_cast<int16_t>(get_u16());
  decoded.pins.dir_pin = static_cast<int16_t>(get_u16());
  decoded.pins.enable_pin = static_cast<int16_t>(get_u16());
  decoded.pins.enable_active_low = read_bool(data + offset++);
  decoded.pins.tmc_uart_rx_pin = static_cast<int16_t>(get_u16());
  decoded.pins.tmc_uart_tx_pin = static_cast<int16_t>(get_u16());
  decoded.pins.tmc_uart_baud = get_u32();
  decoded.pins.tmc_driver_address = get_u8();
  decoded.pins.sense_resistor_milliohm = get_u16();
  decoded.motor.full_steps_per_rev = get_u16();
  decoded.motor.gear_ratio_num = get_u16();
  decoded.motor.gear_ratio_den = get_u16();
  decoded.motor.units_per_rev = get_u32();
  decoded.motor.max_current_ma = get_u16();
  decoded.driver.run_current_ma = get_u16();
  decoded.driver.hold_current_ma = get_u16();
  decoded.driver.microsteps = get_u16();
  decoded.driver.mode = static_cast<DriverMode>(get_u8());
  decoded.driver.blank_time = get_u8();
  decoded.driver.toff = get_u8();
  decoded.driver.interpolation = read_bool(data + offset++);
  decoded.motion.max_velocity_sps = get_u32();
  decoded.motion.max_accel_sps2 = get_u32();
  decoded.motion.max_travel_steps = get_i32();
  decoded.motion.soft_limits_enabled = read_bool(data + offset++);
  decoded.motion.soft_min_steps = get_i32();
  decoded.motion.soft_max_steps = get_i32();
  decoded.motion.default_stop_decel_sps2 = get_u32();
  decoded.telemetry.interval_ms = get_u16();
  decoded.telemetry.enabled_fields = get_u16();
  decoded.telemetry.binary_stream_enabled = read_bool(data + offset++);
  decoded.calibration.position_offset_steps = get_i32();
  decoded.calibration.home_position_steps = get_i32();
  decoded.calibration.direction_inverted = read_bool(data + offset++);
  decoded.calibration.soft_limit_origin_steps = get_i32();
  decoded.safety.host_timeout_ms = get_u32();
  decoded.safety.velocity_timeout_action = static_cast<TimeoutAction>(get_u8());
  decoded.safety.enable_timeout_ms = get_u32();
  decoded.safety.max_allowed_current_ma = get_u16();
  decoded.safety.max_allowed_velocity_sps = get_u32();
  decoded.safety.max_allowed_accel_sps2 = get_u32();

  if (offset > length || !validate_config(decoded).ok) {
    return false;
  }

  *config = decoded;
  return true;
}

}  // namespace daft
