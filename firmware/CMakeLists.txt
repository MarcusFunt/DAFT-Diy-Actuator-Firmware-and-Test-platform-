cmake_minimum_required(VERSION 3.20)

project(daft_firmware_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(daft_core
  src/actuator.cpp
  src/cobs.cpp
  src/config_store.cpp
  src/config_validation.cpp
  src/crc.cpp
  src/protocol_v2.cpp
)

target_include_directories(daft_core PUBLIC include)

if(MSVC)
  target_compile_options(daft_core PRIVATE /W4)
else()
  target_compile_options(daft_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(daft_core_tests
  tests/test_core.cpp
)

target_link_libraries(daft_core_tests PRIVATE daft_core)
target_compile_definitions(daft_core_tests PRIVATE
  DAFT_GOLDEN_FIXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../protocol/golden_frames.txt"
)

enable_testing()
add_test(NAME daft_core_tests COMMAND daft_core_tests)
