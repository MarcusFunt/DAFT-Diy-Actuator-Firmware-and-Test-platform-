#pragma once

#include <stdint.h>

#include "daft/config.hpp"

namespace daft {

struct DriverStatus {
  uint8_t configured = 0;
  uint8_t uart_ok = 0;
  uint8_t overtemperature_pre_warning = 0;
  uint8_t overtemperature = 0;
  uint16_t run_current_ma = 0;
  uint16_t microsteps = 0;
  uint32_t raw_status = 0;
};

struct MotionBackend {
  void* context = nullptr;
  uint32_t (*millis)(void* context) = nullptr;
  bool (*enable_driver)(void* context, bool enabled) = nullptr;
  bool (*configure_driver)(void* context, const Tmc2209Config& config) = nullptr;
  bool (*move_absolute)(void* context, int32_t target_steps, uint32_t max_velocity_sps, uint32_t accel_sps2) = nullptr;
  bool (*move_relative)(void* context, int32_t delta_steps, uint32_t max_velocity_sps, uint32_t accel_sps2) = nullptr;
  bool (*run_velocity)(void* context, int32_t velocity_sps, uint32_t accel_sps2) = nullptr;
  bool (*ramp_stop)(void* context, uint32_t decel_sps2) = nullptr;
  void (*emergency_stop)(void* context) = nullptr;
  int32_t (*current_position)(void* context) = nullptr;
  int32_t (*target_position)(void* context) = nullptr;
  int32_t (*current_speed)(void* context) = nullptr;
  bool (*motion_active)(void* context) = nullptr;
  DriverStatus (*driver_status)(void* context) = nullptr;
  bool (*save_config)(void* context, const ActuatorConfig& config) = nullptr;
  bool (*load_config)(void* context, ActuatorConfig* config, uint32_t* generation) = nullptr;
  bool (*reset_config_storage)(void* context) = nullptr;
  void (*reboot)(void* context) = nullptr;
  uint32_t (*boot_reason)(void* context) = nullptr;
};

}  // namespace daft
