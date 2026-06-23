#pragma once

#include <stdint.h>

namespace daft {

enum class FaultFlag : uint32_t {
  NONE = 0,
  RX_FRAME = 1u << 0,
  BAD_CRC = 1u << 1,
  BAD_PAYLOAD = 1u << 2,
  UNSUPPORTED_COMMAND = 1u << 3,
  INVALID_CONFIG = 1u << 4,
  CONFIG_STORAGE = 1u << 5,
  SOFT_LIMIT = 1u << 6,
  DRIVER = 1u << 7,
  DRIVER_UART = 1u << 8,
  HOST_TIMEOUT = 1u << 9,
  COMMAND_REJECTED = 1u << 10,
};

inline uint32_t fault_value(FaultFlag flag) {
  return static_cast<uint32_t>(flag);
}

inline bool has_fault(uint32_t faults, FaultFlag flag) {
  return (faults & fault_value(flag)) != 0;
}

}  // namespace daft
