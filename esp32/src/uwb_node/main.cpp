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
constexpr size_t MAX_APP_PAYLOAD_SIZE = MAX_UWB_FRAME_SIZE - FRAME_HEADER_SIZE;

constexpr uint8_t FRAME_MAGIC_0 = 0xA1;
constexpr uint8_t FRAME_MAGIC_1 = 0x52;
constexpr uint8_t PROTOCOL_VERSION = 1;

constexpr uint8_t MAX_SEND_ATTEMPTS = 5;
constexpr uint32_t ACK_TIMEOUT_MS = 250;
constexpr uint32_t TX_TIMEOUT_MS = 300;
constexpr uint32_t RETRY_BACKOFF_MIN_MS = 15;
constexpr uint32_t RETRY_BACKOFF_MAX_MS = 70;
constexpr size_t DUPLICATE_HISTORY_SIZE = 32;

static_assert(MAX_APP_PAYLOAD_SIZE == 111,
              "The radio header must leave 111 payload bytes");
static_assert(LOCAL_NODE_ID > 0 && REMOTE_NODE_ID > 0,
              "Node IDs must be non-zero");
static_assert(LOCAL_NODE_ID != REMOTE_NODE_ID,
              "Local and peer IDs must differ");

enum class FrameType : uint8_t {
  Data = 1,
  Ack = 2,
};

struct ParsedFrame {
  FrameType type;
  uint8_t source;
  uint8_t destination;
  uint16_t sequence;
  uint8_t payloadLength;
  const byte *payload;
};

// DW1000 v0.9 adds two FCS bytes inside setData(). Keep two readable safety
// bytes after the application frame, matching the known-good legacy sender.
byte txBuffer[MAX_UWB_FRAME_SIZE + 2] = {};
byte rxBuffer[MAX_UWB_FRAME_SIZE] = {};

// One extra byte allows 111 payload bytes followed by CR before LF.
byte serialBuffer[MAX_APP_PAYLOAD_SIZE + 1] = {};
size_t serialLength = 0;
bool discardSerialLine = false;

volatile bool sentEvent = false;
volatile bool receivedEvent = false;
volatile bool receiveFailedEvent = false;

uint16_t nextSequence = 0;
uint16_t deliveredSequences[DUPLICATE_HISTORY_SIZE] = {};
size_t deliveredSequenceCount = 0;
size_t deliveredSequenceCursor = 0;

bool outboundActive = false;
uint16_t outboundSequence = 0;
bool outboundAckSeen = false;


bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}


void handleSent() {
  sentEvent = true;
}


void handleReceived() {
  receivedEvent = true;
}


void handleReceiveFailed() {
  receiveFailedEvent = true;
}


void clearEvents() {
  noInterrupts();
  sentEvent = false;
  receivedEvent = false;
  receiveFailedEvent = false;
  interrupts();
}


void startReceiver() {
  // Follow the library's own MessagePingPong/legacy receiver pattern. Once
  // permanent receive is enabled, startTransmit() automatically returns the
  // DW1000 to RX after each DATA or ACK transmission.
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}


void configureRadio() {
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(LOCAL_NODE_ID);
  DW1000.setNetworkId(UWB_NETWORK_ID);

  // This is the mode that just passed the real Host -> Jetson hardware test.
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();

  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleReceiveFailed);
  startReceiver();
}


void announceReady() {
  Serial.print(F("[UWB READY] node="));
  Serial.print(LOCAL_NODE_ID);
  Serial.print(F(" peer="));
  Serial.println(REMOTE_NODE_ID);
}


void recoverRadio() {
  clearEvents();
  DW1000.select(PIN_SS);
  configureRadio();
  announceReady();
}


size_t buildFrame(FrameType type, uint16_t sequence,
                  const byte *payload, uint8_t payloadLength) {
  memset(txBuffer, 0, sizeof(txBuffer));
  txBuffer[0] = FRAME_MAGIC_0;
  txBuffer[1] = FRAME_MAGIC_1;
  txBuffer[2] = PROTOCOL_VERSION;
  txBuffer[3] = static_cast<uint8_t>(type);
  txBuffer[4] = LOCAL_NODE_ID;
  txBuffer[5] = REMOTE_NODE_ID;
  txBuffer[6] = static_cast<uint8_t>(sequence & 0xFF);
  txBuffer[7] = static_cast<uint8_t>((sequence >> 8) & 0xFF);
  txBuffer[8] = payloadLength;

  if (payloadLength > 0 && payload != nullptr) {
    memcpy(txBuffer + FRAME_HEADER_SIZE, payload, payloadLength);
  }
  return FRAME_HEADER_SIZE + payloadLength;
}


bool parseFrame(size_t frameLength, ParsedFrame &frame) {
  if (frameLength < FRAME_HEADER_SIZE || frameLength > MAX_UWB_FRAME_SIZE) {
    return false;
  }
  if (rxBuffer[0] != FRAME_MAGIC_0 || rxBuffer[1] != FRAME_MAGIC_1 ||
      rxBuffer[2] != PROTOCOL_VERSION) {
    return false;
  }

  const uint8_t rawType = rxBuffer[3];
  if (rawType != static_cast<uint8_t>(FrameType::Data) &&
      rawType != static_cast<uint8_t>(FrameType::Ack)) {
    return false;
  }

  const uint8_t payloadLength = rxBuffer[8];
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

  frame.type = static_cast<FrameType>(rawType);
  frame.source = rxBuffer[4];
  frame.destination = rxBuffer[5];
  frame.sequence = static_cast<uint16_t>(rxBuffer[6]) |
                   (static_cast<uint16_t>(rxBuffer[7]) << 8);
  frame.payloadLength = payloadLength;
  frame.payload = rxBuffer + FRAME_HEADER_SIZE;
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


bool radioSend(FrameType type, uint16_t sequence,
               const byte *payload, uint8_t payloadLength) {
  if (payloadLength > MAX_APP_PAYLOAD_SIZE) {
    return false;
  }

  const size_t frameLength = buildFrame(type, sequence, payload, payloadLength);

  noInterrupts();
  sentEvent = false;
  interrupts();

  // Keep TX switching as close as possible to the working legacy sender and
  // the library's MessagePingPong example.
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000.setData(txBuffer, frameLength);
  DW1000.startTransmit();

  const uint32_t deadline = millis() + TX_TIMEOUT_MS;
  while (true) {
    bool sent = false;
    noInterrupts();
    sent = sentEvent;
    if (sent) {
      sentEvent = false;
    }
    interrupts();

    if (sent) {
      return true;
    }
    if (deadlineReached(millis(), deadline)) {
      Serial.println(F("[UWB TX TIMEOUT]"));
      recoverRadio();
      return false;
    }
    delay(1);
  }
}


void sendAck(uint16_t sequence) {
  // ACK delivery itself is best effort. If it is lost, the sender retransmits
  // the same DATA sequence and this node will ACK the duplicate again.
  radioSend(FrameType::Ack, sequence, nullptr, 0);
}


void processReceivedFrame() {
  const uint16_t frameLength = DW1000.getDataLength();
  if (frameLength == 0 || frameLength > MAX_UWB_FRAME_SIZE) {
    return;
  }

  DW1000.getData(rxBuffer, frameLength);

  ParsedFrame frame{};
  if (!parseFrame(frameLength, frame)) {
    return;
  }
  if (frame.source != REMOTE_NODE_ID || frame.destination != LOCAL_NODE_ID) {
    return;
  }

  if (frame.type == FrameType::Ack) {
    if (outboundActive && frame.sequence == outboundSequence) {
      outboundAckSeen = true;
    }
    return;
  }

  const bool duplicate = wasDelivered(frame.sequence);
  if (!duplicate) {
    rememberDelivered(frame.sequence);
    Serial.write(frame.payload, frame.payloadLength);
    Serial.write('\n');
  }

  sendAck(frame.sequence);
}


void pumpRadio() {
  bool received = false;
  bool receiveFailed = false;

  noInterrupts();
  received = receivedEvent;
  receiveFailed = receiveFailedEvent;
  receivedEvent = false;
  receiveFailedEvent = false;
  interrupts();

  if (received) {
    processReceivedFrame();
  }

  // DW1000 v0.9 automatically re-arms permanent RX inside its interrupt
  // handler. Do not reset/reconfigure the radio for ordinary RX failures.
  (void)receiveFailed;
}


bool waitForAck(uint16_t sequence, uint32_t timeoutMs) {
  const uint32_t deadline = millis() + timeoutMs;
  while (!deadlineReached(millis(), deadline)) {
    pumpRadio();
    if (outboundActive && outboundSequence == sequence && outboundAckSeen) {
      return true;
    }
    delay(1);
  }
  pumpRadio();
  return outboundActive && outboundSequence == sequence && outboundAckSeen;
}


bool waitBackoffForLateAck(uint16_t sequence, uint32_t delayMs) {
  const uint32_t deadline = millis() + delayMs;
  while (!deadlineReached(millis(), deadline)) {
    pumpRadio();
    if (outboundActive && outboundSequence == sequence && outboundAckSeen) {
      return true;
    }
    delay(1);
  }
  return outboundActive && outboundSequence == sequence && outboundAckSeen;
}


void sendPayloadWithAck(const byte *payload, uint8_t payloadLength) {
  const uint16_t sequence = nextSequence++;
  outboundActive = true;
  outboundSequence = sequence;
  outboundAckSeen = false;

  for (uint8_t attempt = 1; attempt <= MAX_SEND_ATTEMPTS; ++attempt) {
    if (attempt > 1) {
      Serial.print(F("[UWB RETRY] attempt="));
      Serial.println(attempt);
      const uint32_t backoff = static_cast<uint32_t>(
          random(RETRY_BACKOFF_MIN_MS, RETRY_BACKOFF_MAX_MS + 1));
      if (waitBackoffForLateAck(sequence, backoff)) {
        break;
      }
    }

    // Service any peer DATA before changing this radio from RX to TX.
    pumpRadio();
    if (outboundAckSeen) {
      break;
    }

    if (!radioSend(FrameType::Data, sequence, payload, payloadLength)) {
      continue;
    }

    if (waitForAck(sequence, ACK_TIMEOUT_MS)) {
      break;
    }
  }

  if (outboundAckSeen) {
    Serial.println(F("[UWB ACK]"));
  } else {
    Serial.println(F("[UWB ERROR] ack timeout"));
  }

  outboundActive = false;
  outboundAckSeen = false;
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
    sendPayloadWithAck(serialBuffer, static_cast<uint8_t>(serialLength));
  }

  serialLength = 0;
  discardSerialLine = false;
}


void processSerialInput() {
  while (Serial.available() > 0) {
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

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SPI_SS);
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  configureRadio();
  announceReady();
}


void loop() {
  pumpRadio();
  processSerialInput();
  delay(1);
}
