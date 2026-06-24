#include "daft/config_store.hpp"

#include "daft/protocol_v2.hpp"

namespace daft {

namespace {

struct SlotMeta {
  uint16_t version = CONFIG_VERSION;
  uint16_t length = 0;
  uint32_t generation = 0;
  uint16_t crc = 0;
};

void write_meta(uint8_t* out, const SlotMeta& meta) {
  write_u32(out, CONFIG_STORE_MAGIC);
  write_u16(out + 4, meta.version);
  write_u16(out + 6, meta.length);
  write_u32(out + 8, meta.generation);
  write_u16(out + 12, meta.crc);
  write_u16(out + 14, 0);
}

bool read_meta(const uint8_t* in, size_t length, SlotMeta* meta) {
  if (length != CONFIG_STORE_META_SIZE || read_u32(in) != CONFIG_STORE_MAGIC) {
    return false;
  }
  meta->version = read_u16(in + 4);
  meta->length = read_u16(in + 6);
  meta->generation = read_u32(in + 8);
  meta->crc = read_u16(in + 12);
  return meta->version == CONFIG_VERSION && meta->length <= CONFIG_BLOB_MAX_SIZE;
}

bool decode_slot(const ConfigStoreSlotRead& slot, ActuatorConfig* config, uint32_t* generation) {
  if (!slot.valid_marker) {
    return false;
  }

  SlotMeta meta{};
  if (!read_meta(slot.meta, slot.meta_len, &meta)) {
    return false;
  }
  if (slot.payload_len != meta.length || crc16_ccitt(slot.payload, meta.length) != meta.crc) {
    return false;
  }
  if (!deserialize_config(slot.payload, slot.payload_len, config)) {
    return false;
  }

  *generation = meta.generation;
  return true;
}

}  // namespace

bool config_store_generation_newer(uint32_t candidate, uint32_t current) {
  return candidate != current && static_cast<int32_t>(candidate - current) > 0;
}

bool config_store_load(const ConfigStoreIo& io, ActuatorConfig* config, uint32_t* generation) {
  if (io.read_slot == nullptr || config == nullptr || generation == nullptr) {
    return false;
  }

  ActuatorConfig best_config{};
  uint32_t best_generation = 0;
  bool found = false;

  for (uint8_t slot_index = 0; slot_index < CONFIG_STORE_SLOT_COUNT; ++slot_index) {
    ConfigStoreSlotRead slot{};
    if (!io.read_slot(io.context, slot_index, &slot)) {
      continue;
    }

    ActuatorConfig candidate{};
    uint32_t candidate_generation = 0;
    if (!decode_slot(slot, &candidate, &candidate_generation)) {
      continue;
    }

    if (!found || config_store_generation_newer(candidate_generation, best_generation)) {
      best_config = candidate;
      best_generation = candidate_generation;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  *config = best_config;
  *generation = best_generation;
  return true;
}

bool config_store_save(const ConfigStoreIo& io, const ActuatorConfig& config, uint32_t* generation) {
  if (io.read_slot == nullptr || io.write_slot == nullptr) {
    return false;
  }

  uint8_t payload[CONFIG_BLOB_MAX_SIZE]{};
  size_t payload_len = 0;
  if (!serialize_config(config, payload, sizeof(payload), &payload_len)) {
    return false;
  }

  bool valid[CONFIG_STORE_SLOT_COUNT]{};
  uint32_t generations[CONFIG_STORE_SLOT_COUNT]{};
  ActuatorConfig ignored{};
  bool found = false;
  uint32_t newest_generation = 0;
  uint8_t newest_slot = 0;

  for (uint8_t slot_index = 0; slot_index < CONFIG_STORE_SLOT_COUNT; ++slot_index) {
    ConfigStoreSlotRead slot{};
    if (!io.read_slot(io.context, slot_index, &slot)) {
      continue;
    }
    valid[slot_index] = decode_slot(slot, &ignored, &generations[slot_index]);
    if (valid[slot_index] &&
        (!found || config_store_generation_newer(generations[slot_index], newest_generation))) {
      newest_generation = generations[slot_index];
      newest_slot = slot_index;
      found = true;
    }
  }

  uint8_t target_slot = 0;
  if (!valid[0]) {
    target_slot = 0;
  } else if (!valid[1]) {
    target_slot = 1;
  } else {
    target_slot = newest_slot == 0 ? 1 : 0;
  }

  const uint32_t next_generation = found ? newest_generation + 1u : 1u;
  SlotMeta meta{};
  meta.length = static_cast<uint16_t>(payload_len);
  meta.generation = next_generation;
  meta.crc = crc16_ccitt(payload, payload_len);

  uint8_t meta_raw[CONFIG_STORE_META_SIZE]{};
  write_meta(meta_raw, meta);

  if (!io.write_slot(io.context, target_slot, meta_raw, sizeof(meta_raw), payload, payload_len)) {
    return false;
  }

  if (generation != nullptr) {
    *generation = next_generation;
  }
  return true;
}

bool config_store_clear(const ConfigStoreIo& io) {
  return io.clear != nullptr && io.clear(io.context);
}

}  // namespace daft
