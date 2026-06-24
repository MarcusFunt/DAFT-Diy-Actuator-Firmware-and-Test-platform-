#pragma once

#include <stdint.h>

namespace daft {

#ifndef DAFT_FIRMWARE_VERSION_MAJOR
#define DAFT_FIRMWARE_VERSION_MAJOR 1
#endif

#ifndef DAFT_FIRMWARE_VERSION_MINOR
#define DAFT_FIRMWARE_VERSION_MINOR 0
#endif

#ifndef DAFT_FIRMWARE_VERSION_PATCH
#define DAFT_FIRMWARE_VERSION_PATCH 0
#endif

#ifndef DAFT_GIT_SHA
#define DAFT_GIT_SHA "unknown"
#endif

#ifndef DAFT_BUILD_DATE
#define DAFT_BUILD_DATE "unknown"
#endif

#ifndef DAFT_GIT_DIRTY
#define DAFT_GIT_DIRTY 1
#endif

constexpr uint8_t FIRMWARE_VERSION_MAJOR = DAFT_FIRMWARE_VERSION_MAJOR;
constexpr uint8_t FIRMWARE_VERSION_MINOR = DAFT_FIRMWARE_VERSION_MINOR;
constexpr uint8_t FIRMWARE_VERSION_PATCH = DAFT_FIRMWARE_VERSION_PATCH;
constexpr const char* BUILD_GIT_SHA = DAFT_GIT_SHA;
constexpr const char* BUILD_DATE = DAFT_BUILD_DATE;
constexpr bool BUILD_GIT_DIRTY = DAFT_GIT_DIRTY != 0;

}  // namespace daft
