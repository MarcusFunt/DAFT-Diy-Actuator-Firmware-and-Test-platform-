#pragma once

#include <stddef.h>
#include <stdint.h>

namespace daft {

constexpr uint16_t CONFIG_VERSION = 1;
constexpr size_t CONFIG_BLOB_MAX_SIZE = 128;

enum class DriverMode : uint8_t {
  STEALTHCHOP = 0,
  SPREADCYCLE = 1,
};

enum class TimeoutAction : uint8_t {
  RAMP_STOP = 0,
  DISABLE = 1,
};

enum class ConfigField : uint16_t {
  TELEMETRY_INTERVAL_MS = 1,
  SOFT_LIMITS_ENABLED = 2,
  SOFT_LIMIT_MIN_STEPS = 3,
  SOFT_LIMIT_MAX_STEPS = 4,
  DEFAULT_MAX_VELOCITY_SPS = 5,
  DEFAULT_MAX_ACCEL_SPS2 = 6,
  DEFAULT_STOP_DECEL_SPS2 = 7,
  RUN_CURRENT_MA = 8,
  HOLD_CURRENT_MA = 9,
  MICROSTEPS = 10,
  DRIVER_MODE = 11,
  POSITION_OFFSET_STEPS = 12,
  HOME_POSITION_STEPS = 13,
  DIRECTION_INVERTED = 14,
  HOST_TIMEOUT_MS = 15,
  VELOCITY_TIMEOUT_ACTION = 16,
  ENABLE_TIMEOUT_MS = 17,
  MAX_ALLOWED_CURRENT_MA = 18,
  MAX_ALLOWED_VELOCITY_SPS = 19,
  MAX_ALLOWED_ACCEL_SPS2 = 20,
};

enum class ConfigMutability : uint8_t {
  RUNTIME_SAFE = 0,
  REQUIRES_IDLE = 1,
  REQUIRES_DRIVER_REINIT = 2,
  REQUIRES_REBOOT = 3,
  COMPILE_TIME_ONLY = 4,
  RESERVED = 5,
};

struct BoardPins {
  int16_t step_pin = 20;
  int16_t dir_pin = 19;
  int16_t enable_pin = 18;
  bool enable_active_low = true;
  int16_t tmc_uart_rx_pin = 17;
  int16_t tmc_uart_tx_pin = 16;
  uint32_t tmc_uart_baud = 115200;
  uint8_t tmc_driver_address = 0;
  uint16_t sense_resistor_milliohm = 110;
};

struct MotorSpec {
  uint16_t full_steps_per_rev = 200;
  uint16_t gear_ratio_num = 1;
  uint16_t gear_ratio_den = 1;
  uint32_t units_per_rev = 3200;
  uint16_t max_current_ma = 1400;
  char axis_name[16] = {'a', 'x', 'i', 's', '0', 0};
};

struct Tmc2209Config {
  uint16_t run_current_ma = 800;
  uint16_t hold_current_ma = 0;
  uint16_t microsteps = 16;
  DriverMode mode = DriverMode::STEALTHCHOP;
  uint8_t blank_time = 24;
  uint8_t toff = 5;
  bool interpolation = true;
};

struct MotionLimits {
  uint32_t max_velocity_sps = 2000;
  uint32_t max_accel_sps2 = 2000;
  int32_t max_travel_steps = 100000;
  bool soft_limits_enabled = false;
  int32_t soft_min_steps = -100000;
  int32_t soft_max_steps = 100000;
  uint32_t default_stop_decel_sps2 = 1500;
};

struct TelemetryConfig {
  uint16_t interval_ms = 0;
  // Reserved in V1. STATUS telemetry currently always uses the fixed status payload.
  uint16_t enabled_fields = 0xFFFF;
  bool binary_stream_enabled = true;
};

struct CalibrationConfig {
  // Offset/origin fields are persisted for future commissioning semantics but reserved in V1 commands.
  int32_t position_offset_steps = 0;
  int32_t home_position_steps = 0;
  bool direction_inverted = false;
  int32_t soft_limit_origin_steps = 0;
};

struct SafetyConfig {
  uint32_t host_timeout_ms = 1000;
  TimeoutAction velocity_timeout_action = TimeoutAction::RAMP_STOP;
  // Reserved in V1. Drivers are enabled/disabled explicitly by control mode commands.
  uint32_t enable_timeout_ms = 0;
  uint16_t max_allowed_current_ma = 1400;
  uint32_t max_allowed_velocity_sps = 50000;
  uint32_t max_allowed_accel_sps2 = 100000;
};

struct ActuatorConfig {
  uint16_t version = CONFIG_VERSION;
  BoardPins pins;
  MotorSpec motor;
  Tmc2209Config driver;
  MotionLimits motion;
  TelemetryConfig telemetry;
  CalibrationConfig calibration;
  SafetyConfig safety;
};

struct ConfigValidationResult {
  bool ok = true;
  ConfigField field = ConfigField::TELEMETRY_INTERVAL_MS;
  const char* message = "ok";
};

ActuatorConfig default_config();
ConfigMutability config_field_mutability(ConfigField field);
bool config_field_known(ConfigField field);
ConfigValidationResult validate_config(const ActuatorConfig& config);
ConfigValidationResult stage_config_field(ActuatorConfig& staged, ConfigField field, int32_t value);
bool config_field_value(const ActuatorConfig& config, ConfigField field, int32_t* value);
bool serialize_config(const ActuatorConfig& config, uint8_t* out, size_t capacity, size_t* length);
bool deserialize_config(const uint8_t* data, size_t length, ActuatorConfig* config);

}  // namespace daft
