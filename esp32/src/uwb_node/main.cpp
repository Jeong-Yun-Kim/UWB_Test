#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>
#include <esp_system.h>

#ifndef NODE_ID
#error "NODE_ID must be supplied by platformio.ini"
#endif

#ifndef PEER_ID
#error "PEER_ID must be supplied by platformio.ini"
#endif

// Keep this wiring identical to the existing AI Rescue Box DWM1000 wiring.
constexpr uint8_t PIN_RST = 15;
constexpr uint8_t PIN_IRQ = 27;
constexpr uint8_t PIN_SS = 5;
constexpr uint8_t PIN_SCK = 18;
constexpr uint8_t PIN_MISO = 19;
constexpr uint8_t PIN_MOSI = 23;
constexpr uint8_t PIN_SPI_SS = 2;

constexpr uint32_t SERIAL_BAUD_RATE = 460800;
constexpr uint16_t UWB_NETWORK_ID = 10;

constexpr uint8_t LOCAL_NODE_ID = NODE_ID;
constexpr uint8_t REMOTE_NODE_ID = PEER_ID;

constexpr size_t MAX_UWB_FRAME_SIZE = 120;
constexpr size_t FRAME_HEADER_SIZE = 9;
constexpr size_t MAX_APP_PAYLOAD_SIZE =
    MAX_UWB_FRAME_SIZE - FRAME_HEADER_SIZE;

constexpr uint8_t FRAME_MAGIC_0 = 0xA1;
constexpr uint8_t FRAME_MAGIC_1 = 0x52;
constexpr uint8_t PROTOCOL_VERSION = 1;

constexpr uint8_t MAX_RETRIES = 4;
constexpr uint8_t MAX_SEND_ATTEMPTS = 1 + MAX_RETRIES;
constexpr uint32_t ACK_TIMEOUT_MS = 40;
constexpr uint32_t TX_WATCHDOG_MS = 25;
constexpr uint32_t RETRY_BACKOFF_MIN_MS = 5;
constexpr uint32_t RETRY_BACKOFF_MAX_MS = 30;
constexpr size_t DUPLICATE_HISTORY_SIZE = 16;

static_assert(LOCAL_NODE_ID > 0 && REMOTE_NODE_ID > 0,
              "Node IDs must be non-zero");
static_assert(LOCAL_NODE_ID != REMOTE_NODE_ID,
              "Local and peer IDs must be different");
static_assert(MAX_APP_PAYLOAD_SIZE == 111,
              "The binary protocol header must leave 111 payload bytes");

enum class FrameType : uint8_t {
  Data = 1,
  Ack = 2,
};

enum class TransmitPurpose : uint8_t {
  None,
  Data,
  Ack,
};

struct ParsedFrame {
  FrameType type;
  uint8_t source;
  uint8_t destination;
  uint16_t sequence;
  uint8_t payloadLength;
  const byte *payload;
};

struct OutboundMessage {
  bool active = false;
  bool waitingForAck = false;
  bool sendScheduled = false;
  uint16_t sequence = 0;
  uint8_t payloadLength = 0;
  uint8_t attempts = 0;
  uint32_t deadline = 0;
  byte payload[MAX_APP_PAYLOAD_SIZE] = {};
};

// DW1000 v0.9's byte-array setData() includes the two hardware FCS bytes in
// its copy length. Keep two accessible safety bytes after every UWB frame.
byte radioTransmitBuffer[MAX_UWB_FRAME_SIZE + 2] = {};
byte radioReceiveBuffer[MAX_UWB_FRAME_SIZE] = {};

// Allow one extra byte so a maximum-sized line followed by CRLF is accepted.
byte serialBuffer[MAX_APP_PAYLOAD_SIZE + 1] = {};
size_t serialLength = 0;
bool discardSerialLine = false;

OutboundMessage outbound;
TransmitPurpose transmitPurpose = TransmitPurpose::None;
bool radioTransmitActive = false;
uint32_t radioTransmitStartedAt = 0;

uint16_t nextSequence = 0;
uint16_t deliveredSequences[DUPLICATE_HISTORY_SIZE] = {};
size_t deliveredSequenceCount = 0;
size_t deliveredSequenceCursor = 0;

volatile bool interruptSent = false;
volatile bool interruptReceived = false;
volatile bool interruptReceiveFailed = false;
volatile bool interruptRadioError = false;


bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}


void handleSent() {
  interruptSent = true;
}


void handleReceived() {
  interruptReceived = true;
}


void handleReceiveFailed() {
  interruptReceiveFailed = true;
}


void handleRadioError() {
  interruptRadioError = true;
}


void clearInterruptFlags() {
  noInterrupts();
  interruptSent = false;
  interruptReceived = false;
  interruptReceiveFailed = false;
  interruptRadioError = false;
  interrupts();
}


void configureRadio() {
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(LOCAL_NODE_ID);
  DW1000.setNetworkId(UWB_NETWORK_ID);
  DW1000.enableMode(DW1000.MODE_LONGDATA_FAST_ACCURACY);
  // setDefaults() selected channel 5 while still in 16 MHz PRF mode. Select
  // it again after the 64 MHz mode so the matching preamble code is applied.
  DW1000.setChannel(DW1000.CHANNEL_5);
  // RX/TX transitions are managed by this state machine. Disable the
  // hardware auto-reenable bit set by setDefaults() to avoid a double re-arm.
  DW1000.setReceiverAutoReenable(false);
  DW1000.commitConfiguration();

  // The application controls every RX/TX transition. This avoids an automatic
  // receive restart racing with a required acknowledgement transmission.
  DW1000.receivePermanently(false);
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleReceiveFailed);
  DW1000.attachErrorHandler(handleRadioError);
}


void armReceiver() {
  if (radioTransmitActive) {
    return;
  }
  DW1000.newReceive();
  DW1000.startReceive();
}


void recoverRadio() {
  radioTransmitActive = false;
  transmitPurpose = TransmitPurpose::None;
  clearInterruptFlags();

  // A hardware reset clears a wedged TX/RX state. Restore the complete common
  // configuration before returning to the default receive state.
  // select() performs a full hardware reset and reloads the LDE microcode,
  // which a bare reset() would discard.
  DW1000.select(PIN_SS);
  configureRadio();
  armReceiver();

  Serial.print(F("[UWB READY] node="));
  Serial.print(LOCAL_NODE_ID);
  Serial.print(F(" peer="));
  Serial.println(REMOTE_NODE_ID);
}


size_t buildFrame(FrameType type, uint16_t sequence,
                  const byte *payload, uint8_t payloadLength) {
  memset(radioTransmitBuffer, 0, sizeof(radioTransmitBuffer));
  radioTransmitBuffer[0] = FRAME_MAGIC_0;
  radioTransmitBuffer[1] = FRAME_MAGIC_1;
  radioTransmitBuffer[2] = PROTOCOL_VERSION;
  radioTransmitBuffer[3] = static_cast<uint8_t>(type);
  radioTransmitBuffer[4] = LOCAL_NODE_ID;
  radioTransmitBuffer[5] = REMOTE_NODE_ID;
  radioTransmitBuffer[6] = static_cast<uint8_t>(sequence & 0xFF);
  radioTransmitBuffer[7] = static_cast<uint8_t>(sequence >> 8);
  radioTransmitBuffer[8] = payloadLength;

  if (payloadLength > 0 && payload != nullptr) {
    memcpy(radioTransmitBuffer + FRAME_HEADER_SIZE, payload, payloadLength);
  }
  return FRAME_HEADER_SIZE + payloadLength;
}


bool startRadioTransmit(FrameType type, uint16_t sequence,
                        const byte *payload, uint8_t payloadLength,
                        TransmitPurpose purpose) {
  bool receiveEventPending = false;
  noInterrupts();
  receiveEventPending = interruptReceived || interruptReceiveFailed ||
                        interruptRadioError;
  interrupts();

  // Give a just-arrived RX event priority over a scheduled transmission so
  // newTransmit() does not discard a DATA or ACK still in the RX buffer.
  if (radioTransmitActive || receiveEventPending ||
      payloadLength > MAX_APP_PAYLOAD_SIZE) {
    return false;
  }

  const size_t frameLength = buildFrame(type, sequence, payload, payloadLength);
  interruptSent = false;
  DW1000.newTransmit();
  DW1000.setData(radioTransmitBuffer, frameLength);

  transmitPurpose = purpose;
  radioTransmitActive = true;
  radioTransmitStartedAt = millis();
  DW1000.startTransmit();
  return true;
}


bool parseFrame(const byte *frame, size_t frameLength, ParsedFrame &parsed) {
  if (frameLength < FRAME_HEADER_SIZE || frameLength > MAX_UWB_FRAME_SIZE) {
    return false;
  }
  if (frame[0] != FRAME_MAGIC_0 || frame[1] != FRAME_MAGIC_1 ||
      frame[2] != PROTOCOL_VERSION) {
    return false;
  }

  const uint8_t rawType = frame[3];
  if (rawType != static_cast<uint8_t>(FrameType::Data) &&
      rawType != static_cast<uint8_t>(FrameType::Ack)) {
    return false;
  }

  const uint8_t payloadLength = frame[8];
  if (payloadLength > MAX_APP_PAYLOAD_SIZE ||
      frameLength != FRAME_HEADER_SIZE + payloadLength) {
    return false;
  }
  if (rawType == static_cast<uint8_t>(FrameType::Ack) && payloadLength != 0) {
    return false;
  }
  if (rawType == static_cast<uint8_t>(FrameType::Data) && payloadLength == 0) {
    return false;
  }

  parsed.type = static_cast<FrameType>(rawType);
  parsed.source = frame[4];
  parsed.destination = frame[5];
  parsed.sequence = static_cast<uint16_t>(frame[6]) |
                    (static_cast<uint16_t>(frame[7]) << 8);
  parsed.payloadLength = payloadLength;
  parsed.payload = frame + FRAME_HEADER_SIZE;
  return true;
}


bool wasDelivered(uint16_t sequence) {
  for (size_t i = 0; i < deliveredSequenceCount; ++i) {
    if (deliveredSequences[i] == sequence) {
      return true;
    }
  }
  return false;
}


void rememberDelivered(uint16_t sequence) {
  deliveredSequences[deliveredSequenceCursor] = sequence;
  deliveredSequenceCursor =
      (deliveredSequenceCursor + 1) % DUPLICATE_HISTORY_SIZE;
  if (deliveredSequenceCount < DUPLICATE_HISTORY_SIZE) {
    deliveredSequenceCount++;
  }
}


void finishOutboundSuccess() {
  outbound.active = false;
  outbound.waitingForAck = false;
  outbound.sendScheduled = false;
  Serial.println(F("[UWB ACK]"));
}


void finishOutboundFailure(const __FlashStringHelper *reason) {
  outbound.active = false;
  outbound.waitingForAck = false;
  outbound.sendScheduled = false;
  Serial.print(F("[UWB ERROR] "));
  Serial.println(reason);
}


void scheduleRetry() {
  outbound.waitingForAck = false;
  if (outbound.attempts >= MAX_SEND_ATTEMPTS) {
    finishOutboundFailure(F("ack timeout"));
    return;
  }

  const uint32_t backoff =
      static_cast<uint32_t>(random(RETRY_BACKOFF_MIN_MS,
                                   RETRY_BACKOFF_MAX_MS + 1));
  Serial.print(F("[UWB RETRY] attempt="));
  Serial.println(outbound.attempts + 1);
  outbound.sendScheduled = true;
  outbound.deadline = millis() + backoff;
}


void beginOutboundAttempt() {
  if (!outbound.active || radioTransmitActive ||
      outbound.attempts >= MAX_SEND_ATTEMPTS) {
    return;
  }

  if (startRadioTransmit(FrameType::Data, outbound.sequence,
                         outbound.payload, outbound.payloadLength,
                         TransmitPurpose::Data)) {
    outbound.attempts++;
    outbound.sendScheduled = false;
    outbound.waitingForAck = false;
  }
}


void queueOutbound(const byte *payload, uint8_t payloadLength) {
  outbound.active = true;
  outbound.waitingForAck = false;
  outbound.sendScheduled = true;
  outbound.sequence = nextSequence++;
  outbound.payloadLength = payloadLength;
  outbound.attempts = 0;
  memcpy(outbound.payload, payload, payloadLength);

  // A small random initial delay also prevents two user-initiated messages
  // from repeatedly colliding when both nodes start at nearly the same time.
  outbound.deadline = millis() + static_cast<uint32_t>(random(1, 6));
}


void processReceivedFrame() {
  const uint16_t frameLength = DW1000.getDataLength();
  if (frameLength == 0 || frameLength > MAX_UWB_FRAME_SIZE) {
    armReceiver();
    return;
  }

  DW1000.getData(radioReceiveBuffer, frameLength);
  ParsedFrame frame{};
  if (!parseFrame(radioReceiveBuffer, frameLength, frame) ||
      frame.source != REMOTE_NODE_ID || frame.destination != LOCAL_NODE_ID) {
    armReceiver();
    return;
  }

  if (frame.type == FrameType::Ack) {
    if (outbound.active && frame.sequence == outbound.sequence) {
      finishOutboundSuccess();
    }
    armReceiver();
    return;
  }

  const bool duplicate = wasDelivered(frame.sequence);
  if (!duplicate) {
    rememberDelivered(frame.sequence);
  }

  // ACK every valid DATA frame, including duplicates whose previous ACK was
  // lost. Forward only the first copy to the host.
  if (!startRadioTransmit(FrameType::Ack, frame.sequence, nullptr, 0,
                          TransmitPurpose::Ack)) {
    armReceiver();
  }

  if (!duplicate) {
    Serial.write(frame.payload, frame.payloadLength);
    Serial.write('\n');
  }
}


void processRadioEvents() {
  if (interruptRadioError) {
    interruptRadioError = false;
    const bool dataWasTransmitting =
        radioTransmitActive && transmitPurpose == TransmitPurpose::Data;
    recoverRadio();
    if (dataWasTransmitting && outbound.active) {
      scheduleRetry();
    }
    return;
  }

  if (interruptSent) {
    interruptSent = false;
    const TransmitPurpose completedPurpose = transmitPurpose;
    radioTransmitActive = false;
    transmitPurpose = TransmitPurpose::None;

    if (completedPurpose == TransmitPurpose::Data && outbound.active) {
      outbound.waitingForAck = true;
      outbound.deadline = millis() + ACK_TIMEOUT_MS;
    }
    armReceiver();
  }

  if (interruptReceived && !radioTransmitActive) {
    interruptReceived = false;
    processReceivedFrame();
  }

  if (interruptReceiveFailed && !radioTransmitActive) {
    interruptReceiveFailed = false;
    armReceiver();
  }
}


void processTransmitWatchdog() {
  if (!radioTransmitActive ||
      !deadlineReached(millis(), radioTransmitStartedAt + TX_WATCHDOG_MS)) {
    return;
  }

  const bool dataWasTransmitting = transmitPurpose == TransmitPurpose::Data;
  recoverRadio();
  if (dataWasTransmitting && outbound.active) {
    scheduleRetry();
  }
}


void processOutboundState() {
  if (!outbound.active) {
    return;
  }

  const uint32_t now = millis();
  if (outbound.waitingForAck && deadlineReached(now, outbound.deadline)) {
    scheduleRetry();
  }

  if (outbound.active && outbound.sendScheduled && !radioTransmitActive &&
      deadlineReached(now, outbound.deadline)) {
    beginOutboundAttempt();
  }
}


void finishSerialLine() {
  if (!discardSerialLine && serialLength > 0 &&
      serialBuffer[serialLength - 1] == '\r') {
    serialLength--;
  }
  if (serialLength > MAX_APP_PAYLOAD_SIZE) {
    discardSerialLine = true;
  }

  if (discardSerialLine) {
    Serial.println(F("[UWB ERROR] payload too long"));
  } else if (serialLength > 0) {
    queueOutbound(serialBuffer, static_cast<uint8_t>(serialLength));
  }

  serialLength = 0;
  discardSerialLine = false;
}


void processSerialInput() {
  // The host protocol is stop-and-wait. Leave additional bytes in the USB
  // serial buffer until the current message has a receiver ACK or fails.
  if (outbound.active) {
    return;
  }

  while (Serial.available() > 0 && !outbound.active) {
    const byte incoming = static_cast<byte>(Serial.read());
    if (incoming == '\n') {
      finishSerialLine();
    } else if (!discardSerialLine) {
      if (serialLength < sizeof(serialBuffer)) {
        serialBuffer[serialLength++] = incoming;
      } else {
        discardSerialLine = true;
      }
    }
  }
}


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);
  randomSeed(esp_random());
  nextSequence = static_cast<uint16_t>(esp_random());

  // PIN_SPI_SS=2 preserves the existing SPI.begin() call. The actual DWM1000
  // chip select used by the driver is PIN_SS=5.
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SPI_SS);
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  configureRadio();
  armReceiver();
}


void loop() {
  processRadioEvents();
  processTransmitWatchdog();
  processOutboundState();
  processSerialInput();
  delay(1);
}
