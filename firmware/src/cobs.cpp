#include "daft/protocol_v2.hpp"

#include <string.h>

namespace daft {

size_t cobs_encode(const uint8_t* input, size_t length, uint8_t* output, size_t output_capacity) {
  if (output_capacity == 0) {
    return 0;
  }

  size_t read_index = 0;
  size_t write_index = 1;
  size_t code_index = 0;
  uint8_t code = 1;

  while (read_index < length) {
    if (write_index >= output_capacity) {
      return 0;
    }

    if (input[read_index] == 0) {
      output[code_index] = code;
      code_index = write_index++;
      code = 1;
      ++read_index;
    } else {
      output[write_index++] = input[read_index++];
      ++code;
      if (code == 0xFF) {
        if (write_index >= output_capacity) {
          return 0;
        }
        output[code_index] = code;
        code_index = write_index++;
        code = 1;
      }
    }
  }

  output[code_index] = code;
  return write_index;
}

bool cobs_decode(const uint8_t* input, size_t length, uint8_t* output, size_t output_capacity, size_t* decoded_length) {
  size_t read_index = 0;
  size_t write_index = 0;

  while (read_index < length) {
    uint8_t code = input[read_index++];
    if (code == 0) {
      return false;
    }

    size_t copy_length = static_cast<size_t>(code) - 1;
    if (read_index + copy_length > length || write_index + copy_length > output_capacity) {
      return false;
    }

    if (copy_length > 0) {
      memcpy(output + write_index, input + read_index, copy_length);
      write_index += copy_length;
      read_index += copy_length;
    }

    if (code < 0xFF && read_index < length) {
      if (write_index >= output_capacity) {
        return false;
      }
      output[write_index++] = 0;
    }
  }

  *decoded_length = write_index;
  return true;
}

}  // namespace daft
