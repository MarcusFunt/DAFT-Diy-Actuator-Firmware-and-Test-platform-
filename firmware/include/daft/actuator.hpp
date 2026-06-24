#pragma once

#include <stdint.h>

#include "daft/config.hpp"
#include "daft/faults.hpp"
#include "daft/platform.hpp"
#include "daft/protocol_v2.hpp"

namespace daft {

struct ResponseWriter {
  void* context = nullptr;
  bool (*send)(void* context, const Packet& packet) = nullptr;
};

class Actuator {
 public:
  void begin(const ActuatorConfig& defaults, const MotionBackend& backend);
  void update(const ResponseWriter* event_writer = nullptr);
  bool should_send_telemetry() const;
  void mark_telemetry_sent();
  void send_telemetry(const ResponseWriter& writer, uint16_t seq);
  void record_frame_error(ErrorCode error);
  void handle_packet(const Packet& request, const ResponseWriter& writer);

  const ActuatorConfig& active_config() const { return active_config_; }
  const ActuatorConfig& staged_config() const { return staged_config_; }
  ActuatorState state() const { return state_; }
  uint32_t faults() const { return faults_; }
  const Counters& counters() const { return counters_; }

 private:
  void send_ack(const ResponseWriter& writer, uint16_t seq, MsgId acked_msg, ErrorCode code = ErrorCode::OK);
  void send_error(const ResponseWriter& writer, uint16_t seq, MsgId rejected_msg, ErrorCode code);
  void send_status(const ResponseWriter& writer, uint16_t seq, bool telemetry);
  void send_identity(const ResponseWriter& writer, uint16_t seq);
  void send_capabilities(const ResponseWriter& writer, uint16_t seq);
  void send_faults(const ResponseWriter& writer, uint16_t seq);
  void send_counters(const ResponseWriter& writer, uint16_t seq);
  void send_driver_status(const ResponseWriter& writer, uint16_t seq);
  void send_config_value(const ResponseWriter& writer, uint16_t seq, ConfigField field);
  void send_event(const ResponseWriter& writer, uint16_t seq, EventType event, EventSeverity severity,
                  uint16_t detail, uint32_t value);
  ErrorCode stage_field(const Packet& request, const ResponseWriter* event_writer);
  ErrorCode apply_staged_config(const ResponseWriter* event_writer);
  ErrorCode validate_motion_allowed(ControlMode requested_mode) const;
  bool is_motion_state() const;
  uint32_t now_ms() const;
  void set_fault(FaultFlag flag, const ResponseWriter* event_writer = nullptr);
  void transition_to(ActuatorState state, const ResponseWriter* event_writer = nullptr);
  bool send_response(const ResponseWriter& writer, const Packet& packet);
  bool same_request(const Packet& request) const;
  void remember_request(const Packet& request, const Packet* responses, uint8_t response_count);
  void replay_last_response(const ResponseWriter& writer);
  bool to_backend_value(int32_t logical, int32_t* backend_value) const;
  int32_t from_backend_value(int32_t backend_value) const;
  int32_t logical_current_position() const;
  int32_t logical_target_position() const;
  int32_t logical_current_speed() const;

  MotionBackend backend_{};
  ActuatorConfig defaults_{};
  ActuatorConfig active_config_{};
  ActuatorConfig staged_config_{};
  ActuatorState state_ = ActuatorState::BOOTING;
  ControlMode control_mode_ = ControlMode::DISABLED;
  uint32_t faults_ = 0;
  Counters counters_{};
  uint32_t config_generation_ = 0;
  bool driver_enabled_ = false;
  bool config_staged_ = false;
  bool boot_event_pending_ = false;
  uint32_t boot_reason_ = 0;
  uint32_t last_host_ms_ = 0;
  uint32_t last_telemetry_ms_ = 0;
  bool last_request_valid_ = false;
  Packet last_request_{};
  Packet last_responses_[4]{};
  uint8_t last_response_count_ = 0;
};

}  // namespace daft
