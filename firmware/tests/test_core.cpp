#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "daft/actuator.hpp"
#include "daft/config.hpp"
#include "daft/protocol_v2.hpp"

using namespace daft;

namespace {

struct FakeBackend {
  uint32_t now = 0;
  bool enabled = false;
  bool moving = false;
  int32_t position = 0;
  int32_t target = 0;
  int32_t speed = 0;
  uint32_t save_count = 0;
  ActuatorConfig saved{};
};

struct Capture {
  Packet packets[8]{};
  size_t count = 0;
};

uint32_t fake_millis(void* context) {
  return static_cast<FakeBackend*>(context)->now;
}

bool fake_enable(void* context, bool enabled) {
  static_cast<FakeBackend*>(context)->enabled = enabled;
  return true;
}

bool fake_configure(void*, const Tmc2209Config&) {
  return true;
}

bool fake_move_abs(void* context, int32_t target, uint32_t, uint32_t) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->target = target;
  backend->moving = true;
  return true;
}

bool fake_move_rel(void* context, int32_t delta, uint32_t, uint32_t) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->target = backend->position + delta;
  backend->moving = true;
  return true;
}

bool fake_run_vel(void* context, int32_t velocity, uint32_t) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->speed = velocity;
  backend->moving = true;
  return true;
}

bool fake_stop(void* context, uint32_t) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->speed = 0;
  backend->moving = false;
  return true;
}

void fake_estop(void* context) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->speed = 0;
  backend->moving = false;
  backend->enabled = false;
}

int32_t fake_position(void* context) {
  return static_cast<FakeBackend*>(context)->position;
}

int32_t fake_target(void* context) {
  return static_cast<FakeBackend*>(context)->target;
}

int32_t fake_speed(void* context) {
  return static_cast<FakeBackend*>(context)->speed;
}

bool fake_motion_active(void* context) {
  return static_cast<FakeBackend*>(context)->moving;
}

DriverStatus fake_driver_status(void*) {
  DriverStatus status{};
  status.configured = 1;
  status.uart_ok = 1;
  status.run_current_ma = 800;
  status.microsteps = 16;
  return status;
}

bool fake_save_config(void* context, const ActuatorConfig& config) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->saved = config;
  backend->save_count++;
  return true;
}

bool fake_load_config(void*, ActuatorConfig*, uint32_t*) {
  return false;
}

bool fake_reset_config(void*) {
  return true;
}

bool capture_send(void* context, const Packet& packet) {
  Capture* capture = static_cast<Capture*>(context);
  assert(capture->count < 8);
  capture->packets[capture->count++] = packet;
  return true;
}

MotionBackend make_backend(FakeBackend* fake) {
  MotionBackend backend{};
  backend.context = fake;
  backend.millis = fake_millis;
  backend.enable_driver = fake_enable;
  backend.configure_driver = fake_configure;
  backend.move_absolute = fake_move_abs;
  backend.move_relative = fake_move_rel;
  backend.run_velocity = fake_run_vel;
  backend.ramp_stop = fake_stop;
  backend.emergency_stop = fake_estop;
  backend.current_position = fake_position;
  backend.target_position = fake_target;
  backend.current_speed = fake_speed;
  backend.motion_active = fake_motion_active;
  backend.driver_status = fake_driver_status;
  backend.save_config = fake_save_config;
  backend.load_config = fake_load_config;
  backend.reset_config_storage = fake_reset_config;
  return backend;
}

ResponseWriter make_writer(Capture* capture) {
  ResponseWriter writer{};
  writer.context = capture;
  writer.send = capture_send;
  return writer;
}

Packet request(MsgId msg, uint16_t seq = 1) {
  Packet packet{};
  packet.msg_id = static_cast<uint8_t>(msg);
  packet.seq = seq;
  return packet;
}

void test_crc_cobs_packet_roundtrip() {
  Packet packet = request(MsgId::MOVE_ABS, 42);
  packet.payload_len = 12;
  write_i32(packet.payload, 12345);
  write_u32(packet.payload + 4, 2000);
  write_u32(packet.payload + 8, 3000);

  uint8_t frame[MAX_FRAME_SIZE]{};
  size_t frame_len = 0;
  assert(encode_packet(packet, frame, sizeof(frame), &frame_len));
  assert(frame[frame_len - 1] == 0);

  Packet decoded{};
  ErrorCode error = ErrorCode::OK;
  assert(decode_packet(frame, frame_len - 1, &decoded, &error));
  assert(decoded.msg_id == packet.msg_id);
  assert(decoded.seq == 42);
  assert(decoded.payload_len == 12);
  assert(read_i32(decoded.payload) == 12345);

  frame[3] ^= 0x55;
  assert(!decode_packet(frame, frame_len - 1, &decoded, &error));
}

void test_framing_partial_and_back_to_back() {
  Packet first = request(MsgId::PING, 1);
  Packet second = request(MsgId::GET_STATUS, 2);
  uint8_t f1[MAX_FRAME_SIZE]{};
  uint8_t f2[MAX_FRAME_SIZE]{};
  size_t f1_len = 0;
  size_t f2_len = 0;
  assert(encode_packet(first, f1, sizeof(f1), &f1_len));
  assert(encode_packet(second, f2, sizeof(f2), &f2_len));

  FramingDecoder decoder;
  Packet out{};
  ErrorCode error = ErrorCode::OK;
  size_t packets = 0;
  for (size_t i = 0; i < f1_len; ++i) {
    if (decoder.consume(f1[i], &out, &error) == FramingDecoder::Result::PACKET) {
      assert(out.seq == 1);
      packets++;
    }
  }
  for (size_t i = 0; i < f2_len; ++i) {
    if (decoder.consume(f2[i], &out, &error) == FramingDecoder::Result::PACKET) {
      assert(out.seq == 2);
      packets++;
    }
  }
  assert(packets == 2);
}

void test_config_validation_and_serialization() {
  ActuatorConfig config = default_config();
  assert(validate_config(config).ok);
  ConfigValidationResult result = stage_config_field(config, ConfigField::MICROSTEPS, 32);
  assert(result.ok);
  assert(config.driver.microsteps == 32);

  uint8_t blob[CONFIG_BLOB_MAX_SIZE]{};
  size_t length = 0;
  assert(serialize_config(config, blob, sizeof(blob), &length));
  ActuatorConfig decoded{};
  assert(deserialize_config(blob, length, &decoded));
  assert(decoded.driver.microsteps == 32);

  result = stage_config_field(config, ConfigField::MICROSTEPS, 3);
  assert(!result.ok);
}

void test_actuator_motion_and_busy_rejection() {
  FakeBackend fake{};
  Actuator actuator;
  actuator.begin(default_config(), make_backend(&fake));
  Capture capture{};
  ResponseWriter writer = make_writer(&capture);

  Packet mode = request(MsgId::SET_CONTROL_MODE, 10);
  mode.payload_len = 1;
  mode.payload[0] = static_cast<uint8_t>(ControlMode::POSITION);
  actuator.handle_packet(mode, writer);
  assert(fake.enabled);
  assert(actuator.state() == ActuatorState::IDLE);

  Packet move = request(MsgId::MOVE_ABS, 11);
  move.payload_len = 12;
  write_i32(move.payload, 1000);
  write_u32(move.payload + 4, 2000);
  write_u32(move.payload + 8, 2000);
  actuator.handle_packet(move, writer);
  assert(actuator.state() == ActuatorState::MOVING_POSITION);

  Packet run = request(MsgId::RUN_VEL, 12);
  run.payload_len = 8;
  write_i32(run.payload, 1000);
  write_u32(run.payload + 4, 2000);
  actuator.handle_packet(run, writer);
  assert(capture.packets[capture.count - 1].msg_id == static_cast<uint8_t>(MsgId::ERROR));
  assert(capture.packets[capture.count - 1].payload[1] == static_cast<uint8_t>(ErrorCode::ERR_BUSY));
}

void test_velocity_timeout() {
  FakeBackend fake{};
  ActuatorConfig config = default_config();
  config.safety.host_timeout_ms = 250;
  Actuator actuator;
  actuator.begin(config, make_backend(&fake));
  Capture capture{};
  ResponseWriter writer = make_writer(&capture);

  Packet mode = request(MsgId::SET_CONTROL_MODE, 20);
  mode.payload_len = 1;
  mode.payload[0] = static_cast<uint8_t>(ControlMode::VELOCITY);
  actuator.handle_packet(mode, writer);

  Packet run = request(MsgId::RUN_VEL, 21);
  run.payload_len = 8;
  write_i32(run.payload, 1000);
  write_u32(run.payload + 4, 2000);
  actuator.handle_packet(run, writer);
  assert(actuator.state() == ActuatorState::RUNNING_VELOCITY);

  fake.now = 300;
  actuator.update();
  assert(has_fault(actuator.faults(), FaultFlag::HOST_TIMEOUT));
  assert(actuator.state() == ActuatorState::STOPPING || actuator.state() == ActuatorState::IDLE);
}

}  // namespace

int main() {
  test_crc_cobs_packet_roundtrip();
  test_framing_partial_and_back_to_back();
  test_config_validation_and_serialization();
  test_actuator_motion_and_busy_rejection();
  test_velocity_timeout();
  printf("daft_core_tests passed\n");
  return 0;
}
