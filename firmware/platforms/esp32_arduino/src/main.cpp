#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Preferences.h>
#include <TMCStepper.h>
#include <esp_system.h>
#include <stdint.h>
#include <string.h>

#ifdef DISABLED
#undef DISABLED
#endif

#include "daft/actuator.hpp"
#include "daft/config.hpp"
#include "daft/config_store.hpp"
#include "daft/protocol_v2.hpp"

using namespace daft;

namespace {

constexpr uint32_t USB_BAUD = 115200;
constexpr char NVS_NAMESPACE[] = "daft_cfg";

HardwareSerial TmcSerial(1);
TMC2209Stepper driver(&TmcSerial, 0.11f, 0);
FastAccelStepperEngine stepper_engine;
FastAccelStepper* stepper = nullptr;

Actuator actuator;
FramingDecoder decoder;
uint16_t device_seq = 1;
bool driver_configured = false;
ActuatorConfig boot_config = default_config();

struct Esp32Context {
  ActuatorConfig defaults;
};

Esp32Context context;

uint16_t next_device_seq() {
  uint16_t seq = device_seq++;
  if (device_seq == 0) {
    device_seq = 1;
  }
  return seq;
}

void slot_keys(uint8_t slot, const char** valid_key, const char** meta_key, const char** payload_key) {
  static const char* valid_keys[] = {"valid0", "valid1"};
  static const char* meta_keys[] = {"meta0", "meta1"};
  static const char* payload_keys[] = {"payload0", "payload1"};
  *valid_key = valid_keys[slot];
  *meta_key = meta_keys[slot];
  *payload_key = payload_keys[slot];
}

bool esp32_read_slot(void*, uint8_t slot, ConfigStoreSlotRead* out) {
  const char* valid_key = nullptr;
  const char* meta_key = nullptr;
  const char* payload_key = nullptr;
  slot_keys(slot, &valid_key, &meta_key, &payload_key);

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true)) {
    return false;
  }

  out->valid_marker = prefs.getUInt(valid_key, 0) == CONFIG_STORE_VALID_MARKER;
  out->meta_len = prefs.getBytes(meta_key, out->meta, sizeof(out->meta));
  out->payload_len = prefs.getBytes(payload_key, out->payload, sizeof(out->payload));
  prefs.end();
  return true;
}

bool esp32_write_slot(void*, uint8_t slot, const uint8_t* meta, size_t meta_len,
                      const uint8_t* payload, size_t payload_len) {
  const char* valid_key = nullptr;
  const char* meta_key = nullptr;
  const char* payload_key = nullptr;
  slot_keys(slot, &valid_key, &meta_key, &payload_key);

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    return false;
  }
  prefs.putUInt(valid_key, 0);
  const bool ok = prefs.putBytes(payload_key, payload, payload_len) == payload_len &&
                  prefs.putBytes(meta_key, meta, meta_len) == meta_len &&
                  prefs.putUInt(valid_key, CONFIG_STORE_VALID_MARKER) == sizeof(uint32_t);
  prefs.end();
  return ok;
}

bool esp32_clear_config_store(void*) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    return false;
  }
  const bool ok = prefs.clear();
  prefs.end();
  return ok;
}

ConfigStoreIo config_store_io() {
  ConfigStoreIo io{};
  io.read_slot = esp32_read_slot;
  io.write_slot = esp32_write_slot;
  io.clear = esp32_clear_config_store;
  return io;
}

uint32_t cb_millis(void*) {
  return millis();
}

bool cb_enable_driver(void*, bool enabled) {
  if (stepper != nullptr) {
    if (enabled) {
      stepper->enableOutputs();
    } else {
      stepper->disableOutputs();
    }
    return true;
  }
  const BoardPins& pins = boot_config.pins;
  digitalWrite(pins.enable_pin, enabled == pins.enable_active_low ? LOW : HIGH);
  return true;
}

bool cb_configure_driver(void*, const Tmc2209Config& config) {
  driver.begin();
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.toff(config.toff);
  driver.blank_time(config.blank_time);
  driver.rms_current(config.run_current_ma);
  driver.microsteps(config.microsteps);
  driver.en_spreadCycle(config.mode == DriverMode::SPREADCYCLE);
  driver.pwm_autoscale(true);
  driver.intpol(config.interpolation);
  driver.TCOOLTHRS(0xFFFFF);
  driver_configured = driver.test_connection() == 0;
  return driver_configured;
}

bool cb_move_absolute(void*, int32_t target_steps, uint32_t max_velocity_sps, uint32_t accel_sps2) {
  if (stepper == nullptr) {
    return false;
  }
  return stepper->setSpeedInHz(max_velocity_sps) == 0 &&
         stepper->setAcceleration(static_cast<int32_t>(accel_sps2)) == 0 &&
         stepper->moveTo(target_steps) == MOVE_OK;
}

bool cb_move_relative(void*, int32_t delta_steps, uint32_t max_velocity_sps, uint32_t accel_sps2) {
  if (stepper == nullptr) {
    return false;
  }
  return stepper->setSpeedInHz(max_velocity_sps) == 0 &&
         stepper->setAcceleration(static_cast<int32_t>(accel_sps2)) == 0 &&
         stepper->move(delta_steps) == MOVE_OK;
}

bool cb_run_velocity(void*, int32_t velocity_sps, uint32_t accel_sps2) {
  if (stepper == nullptr || velocity_sps == 0) {
    return false;
  }
  const uint32_t abs_velocity = velocity_sps < 0 ? static_cast<uint32_t>(-static_cast<int64_t>(velocity_sps))
                                                : static_cast<uint32_t>(velocity_sps);
  if (stepper->setSpeedInHz(abs_velocity) != 0 || stepper->setAcceleration(static_cast<int32_t>(accel_sps2)) != 0) {
    return false;
  }
  return velocity_sps > 0 ? stepper->runForward() == MOVE_OK : stepper->runBackward() == MOVE_OK;
}

bool cb_ramp_stop(void*, uint32_t decel_sps2) {
  if (stepper == nullptr) {
    return false;
  }
  stepper->setAcceleration(static_cast<int32_t>(decel_sps2));
  stepper->stopMove();
  return true;
}

void cb_emergency_stop(void*) {
  if (stepper != nullptr) {
    const int32_t position = stepper->getCurrentPosition();
    stepper->forceStopAndNewPosition(position);
    stepper->disableOutputs();
  }
}

int32_t cb_current_position(void*) {
  return stepper != nullptr ? stepper->getCurrentPosition() : 0;
}

int32_t cb_target_position(void*) {
  return stepper != nullptr ? stepper->targetPos() : 0;
}

int32_t cb_current_speed(void*) {
  return stepper != nullptr ? stepper->getCurrentSpeedInMilliHz(false) / 1000 : 0;
}

bool cb_motion_active(void*) {
  return stepper != nullptr && stepper->isRunning();
}

DriverStatus cb_driver_status(void*) {
  DriverStatus status{};
  status.configured = driver_configured ? 1 : 0;
  status.run_current_ma = actuator.active_config().driver.run_current_ma;
  status.microsteps = actuator.active_config().driver.microsteps;
  const uint8_t test = driver.test_connection();
  status.uart_ok = test == 0 ? 1 : 0;
  const uint32_t drv_status = driver.DRV_STATUS();
  status.raw_status = drv_status;
  status.overtemperature_pre_warning = driver.otpw() ? 1 : 0;
  status.overtemperature = driver.ot() ? 1 : 0;
  return status;
}

bool cb_save_config(void*, const ActuatorConfig& config) {
  uint32_t generation = 0;
  return config_store_save(config_store_io(), config, &generation);
}

bool cb_load_config(void*, ActuatorConfig* config, uint32_t* generation) {
  if (!config_store_load(config_store_io(), config, generation)) {
    return false;
  }
  boot_config = *config;
  return true;
}

bool cb_reset_config_storage(void*) {
  return config_store_clear(config_store_io());
}

void cb_reboot(void*) {
  delay(50);
  ESP.restart();
}

uint32_t cb_boot_reason(void*) {
  return static_cast<uint32_t>(esp_reset_reason());
}

MotionBackend make_backend() {
  MotionBackend backend{};
  backend.context = &context;
  backend.millis = cb_millis;
  backend.enable_driver = cb_enable_driver;
  backend.configure_driver = cb_configure_driver;
  backend.move_absolute = cb_move_absolute;
  backend.move_relative = cb_move_relative;
  backend.run_velocity = cb_run_velocity;
  backend.ramp_stop = cb_ramp_stop;
  backend.emergency_stop = cb_emergency_stop;
  backend.current_position = cb_current_position;
  backend.target_position = cb_target_position;
  backend.current_speed = cb_current_speed;
  backend.motion_active = cb_motion_active;
  backend.driver_status = cb_driver_status;
  backend.save_config = cb_save_config;
  backend.load_config = cb_load_config;
  backend.reset_config_storage = cb_reset_config_storage;
  backend.reboot = cb_reboot;
  backend.boot_reason = cb_boot_reason;
  return backend;
}

bool serial_send(void*, const Packet& packet) {
  uint8_t frame[MAX_FRAME_SIZE]{};
  size_t frame_len = 0;
  if (!encode_packet(packet, frame, sizeof(frame), &frame_len)) {
    return false;
  }
  return Serial.write(frame, frame_len) == frame_len;
}

ResponseWriter serial_writer() {
  ResponseWriter writer{};
  writer.send = serial_send;
  return writer;
}

void service_serial() {
  Packet packet{};
  ErrorCode error = ErrorCode::OK;
  while (Serial.available() > 0) {
    const uint8_t byte_in = static_cast<uint8_t>(Serial.read());
    const FramingDecoder::Result result = decoder.consume(byte_in, &packet, &error);
    if (result == FramingDecoder::Result::PACKET) {
      actuator.handle_packet(packet, serial_writer());
    } else if (result == FramingDecoder::Result::FRAME_ERROR) {
      actuator.record_frame_error(error);
    }
  }
}

void service_telemetry() {
  if (actuator.should_send_telemetry()) {
    actuator.send_telemetry(serial_writer(), next_device_seq());
  }
}

}  // namespace

void setup() {
  context.defaults = default_config();
  boot_config = context.defaults;

  const BoardPins& pins = boot_config.pins;
  pinMode(pins.enable_pin, OUTPUT);
  digitalWrite(pins.enable_pin, pins.enable_active_low ? HIGH : LOW);

  Serial.begin(USB_BAUD);
  Serial.setTimeout(1);
  TmcSerial.begin(pins.tmc_uart_baud, SERIAL_8N1, pins.tmc_uart_rx_pin, pins.tmc_uart_tx_pin);

  stepper_engine.init();
  stepper = stepper_engine.stepperConnectToPin(pins.step_pin);
  if (stepper != nullptr) {
    stepper->setDirectionPin(pins.dir_pin);
    stepper->setEnablePin(pins.enable_pin, pins.enable_active_low);
    stepper->setAutoEnable(false);
    stepper->setSpeedInHz(context.defaults.motion.max_velocity_sps);
    stepper->setAcceleration(static_cast<int32_t>(context.defaults.motion.max_accel_sps2));
    stepper->disableOutputs();
  }

  actuator.begin(context.defaults, make_backend());
}

void loop() {
  service_serial();
  ResponseWriter writer = serial_writer();
  actuator.update(&writer);
  service_telemetry();
}
