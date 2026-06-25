#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "daft/actuator.hpp"
#include "daft/config_store.hpp"
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
  uint32_t move_abs_count = 0;
  uint32_t move_rel_count = 0;
  uint32_t run_vel_count = 0;
  uint32_t stop_count = 0;
  uint32_t configure_count = 0;
  uint32_t save_count = 0;
  bool configure_ok = true;
  ActuatorConfig saved{};
};

struct Capture {
  Packet packets[64]{};
  size_t count = 0;
};

struct SimStoreSlot {
  bool valid_marker = false;
  uint8_t meta[CONFIG_STORE_META_SIZE]{};
  size_t meta_len = 0;
  uint8_t payload[CONFIG_BLOB_MAX_SIZE]{};
  size_t payload_len = 0;
};

struct SimStore {
  SimStoreSlot slots[CONFIG_STORE_SLOT_COUNT]{};
  bool fail_next_write = false;
  bool partial_next_write = false;
};

uint32_t fake_millis(void* context) {
  return static_cast<FakeBackend*>(context)->now;
}

bool fake_enable(void* context, bool enabled) {
  static_cast<FakeBackend*>(context)->enabled = enabled;
  return true;
}

bool fake_configure(void* context, const Tmc2209Config&) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->configure_count++;
  return backend->configure_ok;
}

bool fake_move_abs(void* context, int32_t target, uint32_t, uint32_t) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->target = target;
  backend->moving = true;
  backend->move_abs_count++;
  return true;
}

bool fake_move_rel(void* context, int32_t delta, uint32_t, uint32_t) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->target = backend->position + delta;
  backend->moving = true;
  backend->move_rel_count++;
  return true;
}

bool fake_run_vel(void* context, int32_t velocity, uint32_t) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->speed = velocity;
  backend->moving = true;
  backend->run_vel_count++;
  return true;
}

bool fake_stop(void* context, uint32_t) {
  FakeBackend* backend = static_cast<FakeBackend*>(context);
  backend->speed = 0;
  backend->moving = false;
  backend->stop_count++;
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
  assert(capture->count < sizeof(capture->packets) / sizeof(capture->packets[0]));
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

bool sim_read_slot(void* context, uint8_t slot, ConfigStoreSlotRead* out) {
  SimStore* store = static_cast<SimStore*>(context);
  if (slot >= CONFIG_STORE_SLOT_COUNT) {
    return false;
  }
  const SimStoreSlot& source = store->slots[slot];
  out->valid_marker = source.valid_marker;
  out->meta_len = source.meta_len;
  out->payload_len = source.payload_len;
  memcpy(out->meta, source.meta, sizeof(out->meta));
  memcpy(out->payload, source.payload, sizeof(out->payload));
  return true;
}

bool sim_write_slot(void* context, uint8_t slot, const uint8_t* meta, size_t meta_len,
                    const uint8_t* payload, size_t payload_len) {
  SimStore* store = static_cast<SimStore*>(context);
  if (slot >= CONFIG_STORE_SLOT_COUNT || store->fail_next_write) {
    store->fail_next_write = false;
    return false;
  }

  SimStoreSlot& target = store->slots[slot];
  target.valid_marker = false;
  target.payload_len = store->partial_next_write && payload_len > 0 ? payload_len - 1 : payload_len;
  memcpy(target.payload, payload, target.payload_len);
  target.meta_len = meta_len;
  memcpy(target.meta, meta, meta_len);
  target.valid_marker = true;
  store->partial_next_write = false;
  return true;
}

bool sim_clear_store(void* context) {
  *static_cast<SimStore*>(context) = SimStore{};
  return true;
}

ConfigStoreIo make_store_io(SimStore* store) {
  ConfigStoreIo io{};
  io.context = store;
  io.read_slot = sim_read_slot;
  io.write_slot = sim_write_slot;
  io.clear = sim_clear_store;
  return io;
}

Packet request(MsgId msg, uint16_t seq = 1) {
  Packet packet{};
  packet.msg_id = static_cast<uint8_t>(msg);
  packet.seq = seq;
  return packet;
}

void sim_put_slot(SimStore* store, uint8_t slot, const ActuatorConfig& config, uint32_t generation,
                  bool valid_marker = true) {
  SimStoreSlot& target = store->slots[slot];
  memset(&target, 0, sizeof(target));
  size_t payload_len = 0;
  assert(serialize_config(config, target.payload, sizeof(target.payload), &payload_len));
  target.payload_len = payload_len;
  target.meta_len = CONFIG_STORE_META_SIZE;
  write_u32(target.meta, CONFIG_STORE_MAGIC);
  write_u16(target.meta + 4, CONFIG_VERSION);
  write_u16(target.meta + 6, static_cast<uint16_t>(payload_len));
  write_u32(target.meta + 8, generation);
  write_u16(target.meta + 12, crc16_ccitt(target.payload, payload_len));
  write_u16(target.meta + 14, 0);
  target.valid_marker = valid_marker;
}

size_t split_fixture_line(char* line, char** parts, size_t capacity) {
  size_t count = 0;
  parts[count++] = line;
  for (char* p = line; *p != '\0'; ++p) {
    if (*p == '|') {
      *p = '\0';
      assert(count < capacity);
      parts[count++] = p + 1;
    }
  }
  return count;
}

uint8_t parse_hex_nibble(char value) {
  if (value >= '0' && value <= '9') {
    return static_cast<uint8_t>(value - '0');
  }
  if (value >= 'A' && value <= 'F') {
    return static_cast<uint8_t>(value - 'A' + 10);
  }
  if (value >= 'a' && value <= 'f') {
    return static_cast<uint8_t>(value - 'a' + 10);
  }
  assert(false);
  return 0;
}

size_t parse_hex_bytes(const char* hex, uint8_t* out, size_t capacity) {
  const size_t hex_len = strlen(hex);
  assert(hex_len % 2 == 0);
  const size_t byte_len = hex_len / 2;
  assert(byte_len <= capacity);
  for (size_t i = 0; i < byte_len; ++i) {
    out[i] = static_cast<uint8_t>((parse_hex_nibble(hex[i * 2]) << 4) |
                                  parse_hex_nibble(hex[i * 2 + 1]));
  }
  return byte_len;
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

void test_golden_frame_fixtures() {
  FILE* input = fopen(DAFT_GOLDEN_FIXTURE_PATH, "r");
  assert(input != nullptr);

  char line[512]{};
  size_t cases = 0;
  while (fgets(line, sizeof(line), input) != nullptr) {
    const size_t line_len = strlen(line);
    if (line_len > 0 && line[line_len - 1] == '\n') {
      line[line_len - 1] = '\0';
    }
    const size_t trimmed_len = strlen(line);
    if (trimmed_len > 0 && line[trimmed_len - 1] == '\r') {
      line[trimmed_len - 1] = '\0';
    }
    if (line[0] == '\0' || line[0] == '#') {
      continue;
    }

    char* parts[6]{};
    assert(split_fixture_line(line, parts, sizeof(parts) / sizeof(parts[0])) == 6);
    uint8_t payload[MAX_PAYLOAD_SIZE]{};
    const size_t payload_len = parse_hex_bytes(parts[4], payload, sizeof(payload));
    uint8_t expected_frame[MAX_FRAME_SIZE]{};
    const size_t expected_frame_len = parse_hex_bytes(parts[5], expected_frame, sizeof(expected_frame));

    Packet packet{};
    packet.msg_id = static_cast<uint8_t>(strtoul(parts[1], nullptr, 16));
    packet.flags = static_cast<uint8_t>(strtoul(parts[2], nullptr, 16));
    packet.seq = static_cast<uint16_t>(strtoul(parts[3], nullptr, 10));
    packet.payload_len = static_cast<uint16_t>(payload_len);
    assert(packet.payload_len <= MAX_PAYLOAD_SIZE);
    if (payload_len > 0) {
      memcpy(packet.payload, payload, payload_len);
    }

    uint8_t frame[MAX_FRAME_SIZE]{};
    size_t frame_len = 0;
    assert(encode_packet(packet, frame, sizeof(frame), &frame_len));
    assert(frame_len == expected_frame_len);
    assert(memcmp(frame, expected_frame, expected_frame_len) == 0);

    Packet decoded{};
    ErrorCode error = ErrorCode::OK;
    assert(decode_packet(expected_frame, expected_frame_len - 1, &decoded, &error));
    assert(decoded.msg_id == packet.msg_id);
    assert(decoded.flags == packet.flags);
    assert(decoded.seq == packet.seq);
    assert(decoded.payload_len == packet.payload_len);
    assert(decoded.payload_len == 0 || memcmp(decoded.payload, packet.payload, decoded.payload_len) == 0);
    ++cases;
  }
  fclose(input);

  assert(cases > 0);
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

  assert(config_field_mutability(ConfigField::ENABLE_TIMEOUT_MS) == ConfigMutability::RESERVED);
  result = stage_config_field(config, ConfigField::ENABLE_TIMEOUT_MS, 500);
  assert(!result.ok);
}

void test_config_store_load_rejects_invalid_marker_and_corruption() {
  SimStore store{};
  ConfigStoreIo io = make_store_io(&store);
  ActuatorConfig config = default_config();
  config.driver.run_current_ma = 700;

  sim_put_slot(&store, 0, config, 1, false);
  ActuatorConfig loaded{};
  uint32_t generation = 0;
  assert(!config_store_load(io, &loaded, &generation));

  sim_put_slot(&store, 0, config, 1, true);
  store.slots[0].payload[4] ^= 0x55;
  assert(!config_store_load(io, &loaded, &generation));
}

void test_config_store_uses_newest_valid_generation_and_ignores_partial_write() {
  SimStore store{};
  ConfigStoreIo io = make_store_io(&store);

  ActuatorConfig old_config = default_config();
  old_config.driver.run_current_ma = 700;
  ActuatorConfig new_config = default_config();
  new_config.driver.run_current_ma = 900;

  sim_put_slot(&store, 0, old_config, 10, true);
  sim_put_slot(&store, 1, new_config, 11, true);
  store.slots[1].payload_len -= 1;

  ActuatorConfig loaded{};
  uint32_t generation = 0;
  assert(config_store_load(io, &loaded, &generation));
  assert(generation == 10);
  assert(loaded.driver.run_current_ma == 700);
}

void test_config_store_generation_wrap_and_save_targeting() {
  SimStore store{};
  ConfigStoreIo io = make_store_io(&store);

  ActuatorConfig before_wrap = default_config();
  before_wrap.driver.run_current_ma = 700;
  ActuatorConfig after_wrap = default_config();
  after_wrap.driver.run_current_ma = 900;

  sim_put_slot(&store, 0, before_wrap, UINT32_MAX, true);
  sim_put_slot(&store, 1, after_wrap, 0, true);

  ActuatorConfig loaded{};
  uint32_t generation = 123;
  assert(config_store_load(io, &loaded, &generation));
  assert(generation == 0);
  assert(loaded.driver.run_current_ma == 900);

  ActuatorConfig saved = default_config();
  saved.driver.run_current_ma = 800;
  assert(config_store_save(io, saved, &generation));
  assert(generation == 1);
  assert(config_store_load(io, &loaded, &generation));
  assert(generation == 1);
  assert(loaded.driver.run_current_ma == 800);
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

void test_driver_reconfigure_required_before_enable_after_fault() {
  FakeBackend fake{};
  fake.configure_ok = false;
  Actuator actuator;
  actuator.begin(default_config(), make_backend(&fake));
  assert(actuator.state() == ActuatorState::FAULTED);
  assert(has_fault(actuator.faults(), FaultFlag::DRIVER));
  assert(fake.configure_count == 1);

  Capture capture{};
  ResponseWriter writer = make_writer(&capture);

  Packet clear = request(MsgId::CLEAR_FAULTS, 70);
  actuator.handle_packet(clear, writer);
  assert(actuator.state() == ActuatorState::DISABLED);
  assert(actuator.faults() == 0);

  Packet mode = request(MsgId::SET_CONTROL_MODE, 71);
  mode.payload_len = 1;
  mode.payload[0] = static_cast<uint8_t>(ControlMode::POSITION);
  actuator.handle_packet(mode, writer);
  assert(!fake.enabled);
  assert(fake.configure_count == 2);
  assert(actuator.state() == ActuatorState::FAULTED);
  assert(has_fault(actuator.faults(), FaultFlag::DRIVER));
  assert(capture.packets[capture.count - 1].msg_id == static_cast<uint8_t>(MsgId::ERROR));
  assert(capture.packets[capture.count - 1].payload[1] == static_cast<uint8_t>(ErrorCode::ERR_BAD_STATE));

  fake.configure_ok = true;
  Packet clear_again = request(MsgId::CLEAR_FAULTS, 72);
  actuator.handle_packet(clear_again, writer);
  Packet mode_again = request(MsgId::SET_CONTROL_MODE, 73);
  mode_again.payload_len = 1;
  mode_again.payload[0] = static_cast<uint8_t>(ControlMode::POSITION);
  actuator.handle_packet(mode_again, writer);
  assert(fake.enabled);
  assert(fake.configure_count == 3);
  assert(actuator.state() == ActuatorState::IDLE);
  assert(actuator.faults() == 0);
  assert(capture.packets[capture.count - 1].msg_id == static_cast<uint8_t>(MsgId::ACK));
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

void test_duplicate_replay_and_mismatch() {
  FakeBackend fake{};
  Actuator actuator;
  actuator.begin(default_config(), make_backend(&fake));
  Capture capture{};
  ResponseWriter writer = make_writer(&capture);

  Packet mode = request(MsgId::SET_CONTROL_MODE, 30);
  mode.payload_len = 1;
  mode.payload[0] = static_cast<uint8_t>(ControlMode::POSITION);
  actuator.handle_packet(mode, writer);

  Packet move = request(MsgId::MOVE_REL, 31);
  move.payload_len = 12;
  write_i32(move.payload, 100);
  write_u32(move.payload + 4, 2000);
  write_u32(move.payload + 8, 2000);
  actuator.handle_packet(move, writer);
  const size_t first_move_response = capture.count - 1;
  assert(fake.move_rel_count == 1);

  actuator.handle_packet(move, writer);
  assert(fake.move_rel_count == 1);
  assert(actuator.counters().duplicate_packets == 1);
  assert(capture.packets[capture.count - 1].msg_id == capture.packets[first_move_response].msg_id);
  assert(capture.packets[capture.count - 1].payload[0] == capture.packets[first_move_response].payload[0]);

  Packet mismatch = move;
  write_i32(mismatch.payload, 200);
  actuator.handle_packet(mismatch, writer);
  assert(fake.move_rel_count == 1);
  assert(actuator.counters().duplicate_packets == 2);
  assert(capture.packets[capture.count - 1].msg_id == static_cast<uint8_t>(MsgId::ERROR));
  assert(capture.packets[capture.count - 1].payload[1] ==
         static_cast<uint8_t>(ErrorCode::ERR_DUPLICATE_MISMATCH));
}

void test_invalid_payloads_and_control_mode() {
  FakeBackend fake{};
  Actuator actuator;
  actuator.begin(default_config(), make_backend(&fake));
  Capture capture{};
  ResponseWriter writer = make_writer(&capture);

  Packet mode = request(MsgId::SET_CONTROL_MODE, 40);
  mode.payload_len = 1;
  mode.payload[0] = 99;
  actuator.handle_packet(mode, writer);
  assert(actuator.state() == ActuatorState::DISABLED);
  assert(capture.packets[capture.count - 1].msg_id == static_cast<uint8_t>(MsgId::ERROR));
  assert(capture.packets[capture.count - 1].payload[1] == static_cast<uint8_t>(ErrorCode::ERR_OUT_OF_RANGE));

  Packet counters = request(MsgId::GET_COUNTERS, 41);
  counters.payload_len = 1;
  counters.payload[0] = 0;
  actuator.handle_packet(counters, writer);
  assert(capture.packets[capture.count - 1].msg_id == static_cast<uint8_t>(MsgId::ERROR));
  assert(capture.packets[capture.count - 1].payload[1] == static_cast<uint8_t>(ErrorCode::ERR_BAD_PAYLOAD));

  Packet stop = request(MsgId::RAMP_STOP, 42);
  stop.payload_len = 4;
  write_u32(stop.payload, 1500);
  actuator.handle_packet(stop, writer);
  assert(fake.stop_count == 0);
  assert(actuator.state() == ActuatorState::DISABLED);
  assert(capture.packets[capture.count - 1].msg_id == static_cast<uint8_t>(MsgId::ACK));

  actuator.record_frame_error(ErrorCode::ERR_BAD_PAYLOAD);
  assert(has_fault(actuator.faults(), FaultFlag::BAD_PAYLOAD));
}

void test_failed_config_stage_preserves_existing_staged_values() {
  FakeBackend fake{};
  Actuator actuator;
  actuator.begin(default_config(), make_backend(&fake));
  Capture capture{};
  ResponseWriter writer = make_writer(&capture);

  Packet stage_telem = request(MsgId::CONFIG_STAGE_FIELD, 50);
  stage_telem.payload_len = 6;
  write_u16(stage_telem.payload, static_cast<uint16_t>(ConfigField::TELEMETRY_INTERVAL_MS));
  write_i32(stage_telem.payload + 2, 50);
  actuator.handle_packet(stage_telem, writer);
  assert(actuator.staged_config().telemetry.interval_ms == 50);

  Packet bad_stage = request(MsgId::CONFIG_STAGE_FIELD, 51);
  bad_stage.payload_len = 6;
  write_u16(bad_stage.payload, static_cast<uint16_t>(ConfigField::MICROSTEPS));
  write_i32(bad_stage.payload + 2, 3);
  actuator.handle_packet(bad_stage, writer);
  assert(capture.packets[capture.count - 1].msg_id == static_cast<uint8_t>(MsgId::ERROR));
  assert(actuator.staged_config().telemetry.interval_ms == 50);
  assert(actuator.active_config().telemetry.interval_ms == 0);
}

void test_motion_limits_and_direction_inversion() {
  FakeBackend fake{};
  ActuatorConfig config = default_config();
  config.motion.max_velocity_sps = 100;
  config.motion.max_accel_sps2 = 100;
  config.calibration.direction_inverted = true;
  Actuator actuator;
  actuator.begin(config, make_backend(&fake));
  Capture capture{};
  ResponseWriter writer = make_writer(&capture);

  Packet mode = request(MsgId::SET_CONTROL_MODE, 60);
  mode.payload_len = 1;
  mode.payload[0] = static_cast<uint8_t>(ControlMode::POSITION);
  actuator.handle_packet(mode, writer);

  Packet too_fast = request(MsgId::MOVE_REL, 61);
  too_fast.payload_len = 12;
  write_i32(too_fast.payload, 50);
  write_u32(too_fast.payload + 4, 101);
  write_u32(too_fast.payload + 8, 100);
  actuator.handle_packet(too_fast, writer);
  assert(fake.move_rel_count == 0);
  assert(capture.packets[capture.count - 1].payload[1] == static_cast<uint8_t>(ErrorCode::ERR_OUT_OF_RANGE));

  Packet move = too_fast;
  move.seq = 62;
  write_u32(move.payload + 4, 100);
  actuator.handle_packet(move, writer);
  assert(fake.move_rel_count == 1);
  assert(fake.target == -50);

  fake.position = -50;
  fake.target = -50;
  fake.moving = false;
  actuator.update();

  Packet velocity_mode = request(MsgId::SET_CONTROL_MODE, 63);
  velocity_mode.payload_len = 1;
  velocity_mode.payload[0] = static_cast<uint8_t>(ControlMode::VELOCITY);
  actuator.handle_packet(velocity_mode, writer);

  Packet run = request(MsgId::RUN_VEL, 64);
  run.payload_len = 8;
  write_i32(run.payload, 40);
  write_u32(run.payload + 4, 100);
  actuator.handle_packet(run, writer);
  assert(fake.run_vel_count == 1);
  assert(fake.speed == -40);

  Packet status = request(MsgId::GET_STATUS, 65);
  actuator.handle_packet(status, writer);
  const Packet& response = capture.packets[capture.count - 1];
  assert(response.msg_id == static_cast<uint8_t>(MsgId::STATUS));
  assert(read_i32(response.payload + 8) == 50);
  assert(read_i32(response.payload + 12) == 50);
  assert(read_i32(response.payload + 16) == 40);
}

}  // namespace

int main() {
  test_crc_cobs_packet_roundtrip();
  test_golden_frame_fixtures();
  test_framing_partial_and_back_to_back();
  test_config_validation_and_serialization();
  test_config_store_load_rejects_invalid_marker_and_corruption();
  test_config_store_uses_newest_valid_generation_and_ignores_partial_write();
  test_config_store_generation_wrap_and_save_targeting();
  test_actuator_motion_and_busy_rejection();
  test_driver_reconfigure_required_before_enable_after_fault();
  test_velocity_timeout();
  test_duplicate_replay_and_mismatch();
  test_invalid_payloads_and_control_mode();
  test_failed_config_stage_preserves_existing_staged_values();
  test_motion_limits_and_direction_inversion();
  printf("daft_core_tests passed\n");
  return 0;
}
