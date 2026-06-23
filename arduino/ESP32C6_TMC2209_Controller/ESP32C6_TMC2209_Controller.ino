/*
  ESP32-C6 + TMC2209 + NEMA17 binary motion controller

  Arduino IDE libraries:
    - FastAccelStepper by gin66
    - TMCStepper by teemuatlut

  The USB serial command port uses DAFT Motion Binary v1:
    - fixed 32-byte little-endian raw packets
    - COBS framing with 0x00 delimiter
    - CRC-16/CCITT-FALSE over bytes 0..29
    - binary ACK/NACK, event, and status packets

  Do not print text logs on this port during motion.
*/

#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

// -- Stepper / TMC2209 --------------------------------------------------------
#define PIN_STEP            20
#define PIN_DIR             19
#define PIN_ENABLE          18      // active LOW

// TMC2209 UART: GPIO16 TX -> 1 kOhm -> TMC PDN_UART, GPIO17 RX -> TMC PDN_UART
#define PIN_TMC_UART_RX     17
#define PIN_TMC_UART_TX     16
#define TMC_UART_BAUD       115200
#define TMC_R_SENSE         0.11f   // BTT TMC2209 v1.3 sense resistor (ohms)
#define TMC_DRIVER_ADDR     0       // MS1/MS2 address pins both LOW

// -- Protocol -----------------------------------------------------------------
static const uint8_t PROTO_VERSION = 1;
static const size_t RAW_PACKET_SIZE = 32;
static const size_t PAYLOAD_SIZE = 23;
static const size_t CRC_OFFSET = 30;
static const size_t RX_FRAME_MAX = 40;
static const uint8_t COMMAND_QUEUE_DEPTH = 4;

static const uint8_t MSG_PING = 0x01;
static const uint8_t MSG_GET_STATUS = 0x02;
static const uint8_t MSG_SET_TELEM_RATE = 0x03;

static const uint8_t MSG_ENABLE = 0x10;
static const uint8_t MSG_DISABLE = 0x11;
static const uint8_t MSG_ESTOP = 0x12;
static const uint8_t MSG_STOP_RAMP = 0x13;
static const uint8_t MSG_ZERO_POSITION = 0x14;

static const uint8_t MSG_MOVE_ABS = 0x20;
static const uint8_t MSG_MOVE_REL = 0x21;
static const uint8_t MSG_RUN_VEL = 0x22;
static const uint8_t MSG_SET_SOFT_LIMITS = 0x23;

static const uint8_t MSG_DRIVER_CONFIG = 0x30;
static const uint8_t MSG_CLEAR_FAULTS = 0x40;

static const uint8_t MSG_ACK = 0x80;
static const uint8_t MSG_NACK = 0x81;
static const uint8_t MSG_STATUS = 0x82;
static const uint8_t MSG_EVENT = 0x83;

static const uint8_t NACK_BAD_VERSION = 2;
static const uint8_t NACK_UNKNOWN_COMMAND = 3;
static const uint8_t NACK_BAD_PAYLOAD_LENGTH = 4;
static const uint8_t NACK_VALUE_OUT_OF_RANGE = 5;
static const uint8_t NACK_SOFT_LIMIT = 6;
static const uint8_t NACK_DRIVER_DISABLED = 8;
static const uint8_t NACK_QUEUE_FULL = 10;
static const uint8_t NACK_DUPLICATE_MISMATCH = 11;

static const uint8_t EVENT_MOVE_DONE = 1;
static const uint8_t EVENT_LIMIT_HIT = 2;
static const uint8_t EVENT_ESTOP = 3;
static const uint8_t EVENT_FAULT = 4;

static const uint16_t FAULT_RX_FRAME = 0x0001;
static const uint16_t FAULT_CRC = 0x0002;
static const uint16_t FAULT_QUEUE_FULL = 0x0004;
static const uint16_t FAULT_SOFT_LIMIT = 0x0008;
static const uint16_t FAULT_STEPPER = 0x0010;

// -- Defaults -----------------------------------------------------------------
static const long USB_BAUD = 115200;
static const uint16_t DEFAULT_CURRENT_MA = 800;
static const uint16_t DEFAULT_MICROSTEPS = 16;
static const uint32_t DEFAULT_MAX_SPEED = 2000;     // microsteps/s
static const uint32_t DEFAULT_ACCEL = 2000;         // microsteps/s^2
static const uint16_t DEFAULT_STATUS_INTERVAL_MS = 0;
static const uint32_t COMMAND_APPLY_TICK_US = 1000;

HardwareSerial TmcSerial(1);
TMC2209Stepper driver(&TmcSerial, TMC_R_SENSE, TMC_DRIVER_ADDR);
FastAccelStepperEngine stepperEngine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

enum MotionState : uint8_t {
  STATE_IDLE = 0,
  STATE_MOVING = 1,
  STATE_VELOCITY = 2,
  STATE_STOPPING = 3,
  STATE_FAULT = 4
};

struct PendingCommand {
  uint8_t msgId;
  uint8_t flags;
  uint16_t seq;
  uint8_t payloadLen;
  uint8_t payload[PAYLOAD_SIZE];
};

MotionState motionState = STATE_IDLE;
bool motorEnabled = false;
bool velocityControlActive = false;
bool motionWasRunning = false;

int32_t velocityTarget = 0;

int32_t activeTarget = 0;
bool softLimitsEnabled = false;
int32_t softLimitMin = INT32_MIN;
int32_t softLimitMax = INT32_MAX;

uint16_t faultFlags = 0;
uint16_t telemetryIntervalMs = DEFAULT_STATUS_INTERVAL_MS;
uint32_t lastStatusMs = 0;
uint32_t lastCommandApplyUs = 0;
uint16_t deviceSeq = 1;

uint8_t rxFrame[RX_FRAME_MAX];
size_t rxFrameLen = 0;

PendingCommand commandQueue[COMMAND_QUEUE_DEPTH];
uint8_t queueHead = 0;
uint8_t queueTail = 0;
uint8_t queueCount = 0;

bool hasLastHostPacket = false;
uint16_t lastHostSeq = 0;
uint8_t lastHostPacket[RAW_PACKET_SIZE];
uint8_t lastReplyMsgId = 0;
uint8_t lastReplyPayload[PAYLOAD_SIZE];
uint8_t lastReplyLen = 0;

uint16_t readU16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint32_t readU32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int32_t readI32(const uint8_t *p) {
  return (int32_t)readU32(p);
}

void writeU16(uint8_t *p, uint16_t value) {
  p[0] = (uint8_t)(value & 0xFF);
  p[1] = (uint8_t)((value >> 8) & 0xFF);
}

void writeU32(uint8_t *p, uint32_t value) {
  p[0] = (uint8_t)(value & 0xFF);
  p[1] = (uint8_t)((value >> 8) & 0xFF);
  p[2] = (uint8_t)((value >> 16) & 0xFF);
  p[3] = (uint8_t)((value >> 24) & 0xFF);
}

void writeI32(uint8_t *p, int32_t value) {
  writeU32(p, (uint32_t)value);
}

uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000) != 0) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output) {
  size_t readIndex = 0;
  size_t writeIndex = 1;
  size_t codeIndex = 0;
  uint8_t code = 1;

  while (readIndex < length) {
    if (input[readIndex] == 0) {
      output[codeIndex] = code;
      codeIndex = writeIndex++;
      code = 1;
      ++readIndex;
    } else {
      output[writeIndex++] = input[readIndex++];
      ++code;
      if (code == 0xFF) {
        output[codeIndex] = code;
        codeIndex = writeIndex++;
        code = 1;
      }
    }
  }

  output[codeIndex] = code;
  return writeIndex;
}

bool cobsDecode(const uint8_t *input, size_t length, uint8_t *output, size_t outputCapacity, size_t *decodedLength) {
  size_t readIndex = 0;
  size_t writeIndex = 0;

  while (readIndex < length) {
    uint8_t code = input[readIndex++];
    if (code == 0) {
      return false;
    }

    size_t copyLength = (size_t)code - 1;
    if (readIndex + copyLength > length || writeIndex + copyLength > outputCapacity) {
      return false;
    }

    if (copyLength > 0) {
      memcpy(output + writeIndex, input + readIndex, copyLength);
      writeIndex += copyLength;
      readIndex += copyLength;
    }

    if (code < 0xFF && readIndex < length) {
      if (writeIndex >= outputCapacity) {
        return false;
      }
      output[writeIndex++] = 0;
    }
  }

  *decodedLength = writeIndex;
  return true;
}

uint16_t nextDeviceSeq() {
  uint16_t seq = deviceSeq++;
  if (deviceSeq == 0) {
    deviceSeq = 1;
  }
  return seq;
}

bool serialTxHasRoom(size_t byteCount) {
  int available = Serial.availableForWrite();
  return available < 0 || (size_t)available >= byteCount;
}

bool sendPacket(uint8_t msgId, uint8_t flags, uint16_t seq, const uint8_t *payload, uint8_t payloadLen, bool dropIfBusy = false) {
  uint8_t raw[RAW_PACKET_SIZE];
  memset(raw, 0, sizeof(raw));

  raw[0] = PROTO_VERSION;
  raw[1] = msgId;
  raw[2] = flags;
  raw[3] = payloadLen;
  writeU16(raw + 4, seq);
  if (payload != nullptr && payloadLen > 0) {
    memcpy(raw + 6, payload, payloadLen);
  }

  uint16_t crc = crc16Ccitt(raw, CRC_OFFSET);
  writeU16(raw + CRC_OFFSET, crc);

  uint8_t encoded[RAW_PACKET_SIZE + 2];
  size_t encodedLen = cobsEncode(raw, RAW_PACKET_SIZE, encoded);
  if (dropIfBusy && !serialTxHasRoom(encodedLen + 1)) {
    return false;
  }

  Serial.write(encoded, encodedLen);
  Serial.write((uint8_t)0);
  return true;
}

void rememberReply(uint8_t msgId, const uint8_t *payload, uint8_t payloadLen) {
  lastReplyMsgId = msgId;
  lastReplyLen = payloadLen;
  memset(lastReplyPayload, 0, sizeof(lastReplyPayload));
  if (payload != nullptr && payloadLen > 0) {
    memcpy(lastReplyPayload, payload, payloadLen);
  }
}

void sendAck(uint16_t seq, uint8_t ackedMsgId) {
  uint8_t payload[2] = {ackedMsgId, 0};
  sendPacket(MSG_ACK, 0, seq, payload, sizeof(payload));
  rememberReply(MSG_ACK, payload, sizeof(payload));
}

void sendNack(uint16_t seq, uint8_t rejectedMsgId, uint8_t reason) {
  uint8_t payload[2] = {rejectedMsgId, reason};
  sendPacket(MSG_NACK, 0, seq, payload, sizeof(payload));
  rememberReply(MSG_NACK, payload, sizeof(payload));
}

void resendLastReply() {
  sendPacket(lastReplyMsgId, 0, lastHostSeq, lastReplyPayload, lastReplyLen);
}

void sendEvent(uint8_t eventId, uint8_t detail) {
  uint8_t payload[2] = {eventId, detail};
  sendPacket(MSG_EVENT, 0, nextDeviceSeq(), payload, sizeof(payload), true);
}

void setMotorEnabled(bool enabled) {
  motorEnabled = enabled;
  if (stepper != nullptr) {
    if (enabled) {
      stepper->enableOutputs();
    } else {
      stepper->disableOutputs();
    }
  } else {
    digitalWrite(PIN_ENABLE, enabled ? LOW : HIGH);
  }
}

void haltMotionState() {
  int32_t position = 0;
  if (stepper != nullptr) {
    position = stepper->getCurrentPosition();
    stepper->forceStopAndNewPosition(position);
  }
  velocityTarget = 0;
  velocityControlActive = false;
  motionWasRunning = false;
  activeTarget = position;
  motionState = STATE_IDLE;
}

bool supportedMicrosteps(uint8_t microsteps) {
  switch (microsteps) {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
    case 256:
      return true;
    default:
      return false;
  }
}

void configureDriver(uint16_t currentMa, uint8_t microsteps, uint8_t driverMode) {
  driver.begin();
  driver.pdn_disable(true);       // Use UART, not PDN-only mode.
  driver.I_scale_analog(false);   // Use UART current scaling.
  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(currentMa);
  driver.microsteps(microsteps);
  driver.en_spreadCycle(driverMode == 1);
  driver.pwm_autoscale(true);
  driver.TCOOLTHRS(0xFFFFF);
}

uint8_t queueFreeSlots() {
  return COMMAND_QUEUE_DEPTH - queueCount;
}

void sendStatus(bool dropIfBusy = true) {
  uint8_t payload[PAYLOAD_SIZE];
  memset(payload, 0, sizeof(payload));

  int32_t currentPosition = stepper != nullptr ? stepper->getCurrentPosition() : 0;
  int32_t currentTarget = velocityControlActive || stepper == nullptr ? activeTarget : stepper->targetPos();
  int32_t currentSpeed = stepper != nullptr ? stepper->getCurrentSpeedInMilliHz(false) / 1000 : 0;

  payload[0] = 0;  // axis
  payload[1] = (uint8_t)motionState;
  writeU16(payload + 2, faultFlags);
  writeI32(payload + 4, currentPosition);
  writeI32(payload + 8, currentTarget);
  writeI32(payload + 12, currentSpeed);
  writeU32(payload + 16, millis());
  payload[20] = queueFreeSlots();
  payload[21] = motorEnabled ? 1 : 0;
  payload[22] = 0;

  sendPacket(MSG_STATUS, 0, nextDeviceSeq(), payload, sizeof(payload), dropIfBusy);
}

bool enqueueCommand(uint8_t msgId, uint8_t flags, uint16_t seq, const uint8_t *payload, uint8_t payloadLen) {
  if (queueCount >= COMMAND_QUEUE_DEPTH) {
    return false;
  }

  PendingCommand &slot = commandQueue[queueTail];
  slot.msgId = msgId;
  slot.flags = flags;
  slot.seq = seq;
  slot.payloadLen = payloadLen;
  memset(slot.payload, 0, sizeof(slot.payload));
  if (payloadLen > 0) {
    memcpy(slot.payload, payload, payloadLen);
  }

  queueTail = (queueTail + 1) % COMMAND_QUEUE_DEPTH;
  ++queueCount;
  return true;
}

void popCommand() {
  if (queueCount == 0) {
    return;
  }
  queueHead = (queueHead + 1) % COMMAND_QUEUE_DEPTH;
  --queueCount;
}

bool targetViolatesSoftLimits(int32_t target) {
  return softLimitsEnabled && (target < softLimitMin || target > softLimitMax);
}

uint8_t validateCommand(uint8_t msgId, const uint8_t *payload, uint8_t payloadLen) {
  switch (msgId) {
    case MSG_PING:
    case MSG_GET_STATUS:
    case MSG_ENABLE:
    case MSG_DISABLE:
    case MSG_ESTOP:
    case MSG_ZERO_POSITION:
    case MSG_CLEAR_FAULTS:
      return payloadLen == 0 ? 0 : NACK_BAD_PAYLOAD_LENGTH;

    case MSG_SET_TELEM_RATE:
      return payloadLen == 2 ? 0 : NACK_BAD_PAYLOAD_LENGTH;

    case MSG_STOP_RAMP:
      if (payloadLen != 12 || payload[0] != 0) {
        return NACK_BAD_PAYLOAD_LENGTH;
      }
      return readU32(payload + 4) > 0 ? 0 : NACK_VALUE_OUT_OF_RANGE;

    case MSG_MOVE_ABS:
    case MSG_MOVE_REL: {
      if (payloadLen != PAYLOAD_SIZE || payload[0] != 0) {
        return NACK_BAD_PAYLOAD_LENGTH;
      }
      if (stepper == nullptr) {
        return NACK_VALUE_OUT_OF_RANGE;
      }
      if (!motorEnabled) {
        return NACK_DRIVER_DISABLED;
      }

      uint32_t maxSpeed = readU32(payload + 8);
      uint32_t accel = readU32(payload + 12);
      if (maxSpeed == 0 || accel == 0 || maxSpeed > stepper->getMaxSpeedInHz()) {
        return NACK_VALUE_OUT_OF_RANGE;
      }

      int32_t target = readI32(payload + 4);
      if (msgId == MSG_MOVE_REL) {
        int64_t relativeTarget = (int64_t)stepper->getCurrentPosition() + (int64_t)target;
        if (relativeTarget < INT32_MIN || relativeTarget > INT32_MAX) {
          return NACK_VALUE_OUT_OF_RANGE;
        }
        target = (int32_t)relativeTarget;
      }

      if ((payload[1] & 0x01) != 0 && targetViolatesSoftLimits(target)) {
        faultFlags |= FAULT_SOFT_LIMIT;
        return NACK_SOFT_LIMIT;
      }
      return 0;
    }

    case MSG_RUN_VEL:
      if (payloadLen != PAYLOAD_SIZE || payload[0] != 0) {
        return NACK_BAD_PAYLOAD_LENGTH;
      }
      if (stepper == nullptr) {
        return NACK_VALUE_OUT_OF_RANGE;
      }
      if (!motorEnabled) {
        return NACK_DRIVER_DISABLED;
      }
      if (readU32(payload + 8) == 0) {
        return NACK_VALUE_OUT_OF_RANGE;
      }
      {
        int32_t speed = readI32(payload + 4);
        uint32_t absSpeed = speed < 0 ? (uint32_t)(-(int64_t)speed) : (uint32_t)speed;
        if (absSpeed > stepper->getMaxSpeedInHz()) {
          return NACK_VALUE_OUT_OF_RANGE;
        }
      }
      return 0;

    case MSG_SET_SOFT_LIMITS:
      if (payloadLen != 12 || payload[0] != 0) {
        return NACK_BAD_PAYLOAD_LENGTH;
      }
      if ((payload[1] & 0x01) != 0 && readI32(payload + 4) > readI32(payload + 8)) {
        return NACK_VALUE_OUT_OF_RANGE;
      }
      return 0;

    case MSG_DRIVER_CONFIG:
      if (payloadLen != PAYLOAD_SIZE || payload[0] != 0) {
        return NACK_BAD_PAYLOAD_LENGTH;
      }
      if (!supportedMicrosteps(payload[1]) || payload[2] > 1 || readU16(payload + 4) == 0) {
        return NACK_VALUE_OUT_OF_RANGE;
      }
      return 0;

    default:
      return NACK_UNKNOWN_COMMAND;
  }
}

uint32_t commandApplyTick(const PendingCommand &command) {
  switch (command.msgId) {
    case MSG_MOVE_ABS:
    case MSG_MOVE_REL:
      return readU32(command.payload + 16);
    case MSG_RUN_VEL:
      return readU32(command.payload + 12);
    case MSG_STOP_RAMP:
      return readU32(command.payload + 8);
    default:
      return 0;
  }
}

void startPositionMove(int32_t target, uint32_t maxSpeed, uint32_t accel) {
  if (stepper == nullptr) {
    faultFlags |= FAULT_STEPPER;
    motionState = STATE_FAULT;
    sendEvent(EVENT_FAULT, MSG_MOVE_ABS);
    return;
  }

  velocityTarget = 0;
  velocityControlActive = false;
  activeTarget = target;

  if (stepper->setSpeedInHz(maxSpeed) != 0 || stepper->setAcceleration((int32_t)accel) != 0 ||
      stepper->moveTo(target) != MOVE_OK) {
    faultFlags |= FAULT_STEPPER;
    motionState = STATE_FAULT;
    sendEvent(EVENT_FAULT, MSG_MOVE_ABS);
    return;
  }

  motionWasRunning = true;
  motionState = stepper->isRunning() ? STATE_MOVING : STATE_IDLE;
}

void startVelocityMove(int32_t speed, uint32_t accel) {
  if (stepper == nullptr) {
    faultFlags |= FAULT_STEPPER;
    motionState = STATE_FAULT;
    sendEvent(EVENT_FAULT, MSG_RUN_VEL);
    return;
  }

  if (speed == 0) {
    startRampStop(accel);
    return;
  }

  uint32_t absSpeed = speed < 0 ? (uint32_t)(-(int64_t)speed) : (uint32_t)speed;
  velocityTarget = speed;
  velocityControlActive = true;
  activeTarget = stepper->getCurrentPosition();

  MoveResultCode result = MOVE_OK;
  if (stepper->setSpeedInHz(absSpeed) != 0 || stepper->setAcceleration((int32_t)accel) != 0) {
    result = MOVE_ERR_SPEED_IS_UNDEFINED;
  } else {
    result = speed > 0 ? stepper->runForward() : stepper->runBackward();
  }

  if (result != MOVE_OK) {
    faultFlags |= FAULT_STEPPER;
    motionState = STATE_FAULT;
    sendEvent(EVENT_FAULT, MSG_RUN_VEL);
    return;
  }

  motionWasRunning = true;
  motionState = STATE_VELOCITY;
}

void startRampStop(uint32_t decel) {
  if (stepper == nullptr) {
    faultFlags |= FAULT_STEPPER;
    motionState = STATE_FAULT;
    sendEvent(EVENT_FAULT, MSG_STOP_RAMP);
    return;
  }

  if (stepper->isRunning()) {
    stepper->setAcceleration((int32_t)decel);
    stepper->stopMove();
    velocityTarget = 0;
    velocityControlActive = false;
    motionWasRunning = true;
    motionState = STATE_STOPPING;
  } else {
    velocityTarget = 0;
    velocityControlActive = false;
    motionState = STATE_IDLE;
  }
}

void emergencyStop() {
  haltMotionState();
  setMotorEnabled(false);
  sendEvent(EVENT_ESTOP, 0);
}

void applyCommand(const PendingCommand &command) {
  const uint8_t *payload = command.payload;

  switch (command.msgId) {
    case MSG_ENABLE:
      setMotorEnabled(true);
      break;

    case MSG_DISABLE:
      haltMotionState();
      setMotorEnabled(false);
      break;

    case MSG_ZERO_POSITION:
      if (stepper != nullptr) {
        stepper->setCurrentPosition(0);
      }
      activeTarget = 0;
      break;

    case MSG_CLEAR_FAULTS:
      faultFlags = 0;
      break;

    case MSG_STOP_RAMP:
      startRampStop(readU32(payload + 4));
      break;

    case MSG_MOVE_ABS:
      startPositionMove(readI32(payload + 4), readU32(payload + 8), readU32(payload + 12));
      break;

    case MSG_MOVE_REL: {
      int32_t delta = readI32(payload + 4);
      int32_t target = (int32_t)((int64_t)stepper->getCurrentPosition() + (int64_t)delta);
      startPositionMove(target, readU32(payload + 8), readU32(payload + 12));
      break;
    }

    case MSG_RUN_VEL:
      startVelocityMove(readI32(payload + 4), readU32(payload + 8));
      break;

    case MSG_SET_SOFT_LIMITS:
      softLimitsEnabled = (payload[1] & 0x01) != 0;
      softLimitMin = readI32(payload + 4);
      softLimitMax = readI32(payload + 8);
      break;

    case MSG_DRIVER_CONFIG:
      configureDriver(readU16(payload + 4), payload[1], payload[2]);
      break;
  }
}

void serviceCommandQueue() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastCommandApplyUs) < COMMAND_APPLY_TICK_US) {
    return;
  }
  lastCommandApplyUs = nowUs;

  if (queueCount == 0) {
    return;
  }

  PendingCommand &command = commandQueue[queueHead];
  uint32_t applyTick = commandApplyTick(command);
  if (applyTick != 0 && (int32_t)(millis() - applyTick) < 0) {
    return;
  }

  applyCommand(command);
  popCommand();
}

void acceptOrRejectCommand(uint8_t msgId, uint8_t flags, uint16_t seq, const uint8_t *payload, uint8_t payloadLen) {
  uint8_t validation = validateCommand(msgId, payload, payloadLen);
  if (validation != 0) {
    sendNack(seq, msgId, validation);
    return;
  }

  if (msgId == MSG_PING) {
    sendAck(seq, msgId);
    return;
  }

  if (msgId == MSG_GET_STATUS) {
    sendAck(seq, msgId);
    sendStatus(false);
    return;
  }

  if (msgId == MSG_SET_TELEM_RATE) {
    telemetryIntervalMs = readU16(payload);
    sendAck(seq, msgId);
    return;
  }

  if (msgId == MSG_ESTOP) {
    emergencyStop();
    sendAck(seq, msgId);
    return;
  }

  if (!enqueueCommand(msgId, flags, seq, payload, payloadLen)) {
    faultFlags |= FAULT_QUEUE_FULL;
    sendNack(seq, msgId, NACK_QUEUE_FULL);
    return;
  }

  sendAck(seq, msgId);
}

void processFrame(const uint8_t *frame, size_t frameLen) {
  uint8_t raw[RAW_PACKET_SIZE];
  size_t rawLen = 0;
  if (!cobsDecode(frame, frameLen, raw, sizeof(raw), &rawLen) || rawLen != RAW_PACKET_SIZE) {
    faultFlags |= FAULT_RX_FRAME;
    return;
  }

  uint16_t expectedCrc = readU16(raw + CRC_OFFSET);
  uint16_t actualCrc = crc16Ccitt(raw, CRC_OFFSET);
  if (expectedCrc != actualCrc) {
    faultFlags |= FAULT_CRC;
    return;
  }

  uint8_t version = raw[0];
  uint8_t msgId = raw[1];
  uint8_t flags = raw[2];
  uint8_t payloadLen = raw[3];
  uint16_t seq = readU16(raw + 4);

  if (hasLastHostPacket && seq == lastHostSeq) {
    if (memcmp(raw, lastHostPacket, RAW_PACKET_SIZE) == 0) {
      resendLastReply();
    } else {
      sendNack(seq, msgId, NACK_DUPLICATE_MISMATCH);
    }
    return;
  }

  hasLastHostPacket = true;
  lastHostSeq = seq;
  memcpy(lastHostPacket, raw, RAW_PACKET_SIZE);

  if (version != PROTO_VERSION) {
    sendNack(seq, msgId, NACK_BAD_VERSION);
    return;
  }

  if (payloadLen > PAYLOAD_SIZE) {
    sendNack(seq, msgId, NACK_BAD_PAYLOAD_LENGTH);
    return;
  }

  acceptOrRejectCommand(msgId, flags, seq, raw + 6, payloadLen);
}

void readUsbSerial() {
  while (Serial.available() > 0) {
    uint8_t byteIn = (uint8_t)Serial.read();
    if (byteIn == 0) {
      if (rxFrameLen > 0) {
        processFrame(rxFrame, rxFrameLen);
        rxFrameLen = 0;
      }
      continue;
    }

    if (rxFrameLen < sizeof(rxFrame)) {
      rxFrame[rxFrameLen++] = byteIn;
    } else {
      rxFrameLen = 0;
      faultFlags |= FAULT_RX_FRAME;
    }
  }
}

void serviceMotion() {
  if (stepper == nullptr) {
    return;
  }

  bool running = stepper->isRunning();
  if (running) {
    motionWasRunning = true;
    return;
  }

  if (motionWasRunning) {
    motionWasRunning = false;
    velocityTarget = 0;
    velocityControlActive = false;
    motionState = STATE_IDLE;
    activeTarget = stepper->getCurrentPosition();
    sendEvent(EVENT_MOVE_DONE, 0);
  }
}

void serviceTelemetry() {
  if (telemetryIntervalMs == 0) {
    return;
  }

  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastStatusMs) >= telemetryIntervalMs) {
    lastStatusMs = nowMs;
    sendStatus();
  }
}

void setup() {
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH);

  Serial.begin(USB_BAUD);
  Serial.setTimeout(1);

  TmcSerial.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_UART_RX, PIN_TMC_UART_TX);
  configureDriver(DEFAULT_CURRENT_MA, DEFAULT_MICROSTEPS, 0);

  stepperEngine.init();
  stepper = stepperEngine.stepperConnectToPin(PIN_STEP);
  if (stepper != nullptr) {
    stepper->setDirectionPin(PIN_DIR);
    stepper->setEnablePin(PIN_ENABLE, true);
    stepper->setAutoEnable(false);
    stepper->setSpeedInHz(DEFAULT_MAX_SPEED);
    stepper->setAcceleration((int32_t)DEFAULT_ACCEL);
    stepper->disableOutputs();
  } else {
    faultFlags |= FAULT_STEPPER;
    motionState = STATE_FAULT;
  }

  activeTarget = stepper != nullptr ? stepper->getCurrentPosition() : 0;
  lastCommandApplyUs = micros();
}

void loop() {
  readUsbSerial();
  serviceCommandQueue();
  serviceMotion();
  serviceTelemetry();
}
