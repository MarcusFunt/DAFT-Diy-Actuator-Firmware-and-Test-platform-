#pragma once

#include <stddef.h>
#include <stdint.h>

#include "daft/config.hpp"

namespace daft {

constexpr uint32_t CONFIG_STORE_MAGIC = 0x44414654u;  // DAFT
constexpr uint32_t CONFIG_STORE_VALID_MARKER = 0xA5D0C0DEu;
constexpr size_t CONFIG_STORE_SLOT_COUNT = 2;
constexpr size_t CONFIG_STORE_META_SIZE = 16;

struct ConfigStoreSlotRead {
  bool valid_marker = false;
  uint8_t meta[CONFIG_STORE_META_SIZE] = {};
  size_t meta_len = 0;
  uint8_t payload[CONFIG_BLOB_MAX_SIZE] = {};
  size_t payload_len = 0;
};

struct ConfigStoreIo {
  void* context = nullptr;
  bool (*read_slot)(void* context, uint8_t slot, ConfigStoreSlotRead* out) = nullptr;
  bool (*write_slot)(void* context, uint8_t slot, const uint8_t* meta, size_t meta_len,
                     const uint8_t* payload, size_t payload_len) = nullptr;
  bool (*clear)(void* context) = nullptr;
};

bool config_store_generation_newer(uint32_t candidate, uint32_t current);
bool config_store_load(const ConfigStoreIo& io, ActuatorConfig* config, uint32_t* generation);
bool config_store_save(const ConfigStoreIo& io, const ActuatorConfig& config, uint32_t* generation);
bool config_store_clear(const ConfigStoreIo& io);

}  // namespace daft
