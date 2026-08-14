#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgConfiguration.hpp>
#include <DW1000NgConstants.hpp>
#include <esp_system.h>

#ifndef NODE_ID
#error "NODE_ID must be supplied by platformio.ini"
#endif

#ifndef PEER_ID
#error "PEER_ID must be supplied by platformio.ini"
#endif

// ESP32 VSPI wiring used by the existing, hardware-verified one-way sketches.
constexpr uint8_t PIN_RST = 15;
constexpr uint8_t PIN_SS = 5;
constexpr uint8_t PIN_SCK = 18;
constexpr uint8_t PIN_MISO = 19;
constexpr uint8_t PIN_MOSI = 23;

constexpr uint32_t SERIAL_BAUD_RATE = 460800;
constexpr uint16_t UWB_NETWORK_ID = 10;
constexpr uint8_t LOCAL_NODE_ID = NODE_ID;
constexpr uint8_t REMOTE_NODE_ID = PEER_ID;

// FirmwareLineTransport sends one newline-delimited payload of at most 111 bytes.
constexpr size_t MAX_APP_PAYLOAD_SIZE = 111;

// Compact link frame: magic(2), type(1), src(1), dst(1), sequence(2).
// 7 + 111 = 118 bytes; with the hardware FCS this remains below 127 bytes.
constexpr uint8_t FRAME_MAGIC_0 = 0xA1;
constexpr uint8_t FRAME_MAGIC_1 = 0x52;
constexpr uint8_t FRAME_DATA = 1;
constexpr uint8_t FRAME_ACK = 2;
constexpr size_t LINK_HEADER_SIZE = 7;
constexpr size_t MAX_LINK_FRAME_SIZE =
    LINK_HEADER_SIZE + MAX_APP_PAYLOAD_SIZE;

constexpr uint8_t MAX_SEND_ATTEMPTS = 4;
constexpr uint32_t TX_DONE_TIMEOUT_MS = 500;
constexpr uint32_t ACK_TIMEOUT_MS = 350;
constexpr uint32_t RX_AFTER_TX_DELAY_US = 700;
constexpr uint32_t ACK_TURNAROUND_MS = 6;
constexpr uint32_t POLL_DELAY_US = 200;

static_assert(LOCAL_NODE_ID > 0 && REMOTE_NODE_ID > 0,
              "node IDs must be non-zero");
static_assert(LOCAL_NODE_ID != REMOTE_NODE_ID,
              "local and peer IDs must differ");
static_assert(MAX_LINK_FRAME_SIZE == 118,
              "link framing must preserve the 111-byte payload limit");

// Match the radio mode that was proven with the fixed sender/receiver sketches.
// Interrupts are intentionally not used. DW1000Ng status is polled from loop(),
// so no ESP32 SPI transaction runs inside a GPIO ISR.
device_configuration_t RADIO_CONFIG = {
    false,                         // extendedFrameLength
    false,                         // receiverAutoReenable
    false,                         // smartPower
    true,                          // frameCheck
    false,                         // nlos
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_110KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_2048,
    PreambleCode::CODE_4,
};

struct ParsedFrame {
  uint8_t type = 0;
  uint8_t source = 0;
  uint8_t destination = 0;
  uint16_t sequence = 0;
  uint8_t payloadLength = 0;
  byte payload[MAX_APP_PAYLOAD_SIZE] = {};
};

byte radioTxBuffer[MAX_LINK_FRAME_SIZE] = {};
byte radioRxBuffer[MAX_LINK_FRAME_SIZE] = {};

byte serialBuffer[MAX_APP_PAYLOAD_SIZE + 1] = {};
size_t serialLength = 0;
bool discardSerialLine = false;

bool receiverArmed = false;
uint16_t nextSequence = 0;
bool haveDeliveredSequence = false;
uint16_t lastDeliveredSequence = 0;

bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

void clearRadioStatus() {
  DW1000Ng::clearTransmitStatus();
  DW1000Ng::clearReceiveStatus();
  DW1000Ng::clearReceiveFailedStatus();
  DW1000Ng::clearReceiveTimeoutStatus();
}

void startReceiver() {
  DW1000Ng::forceTRxOff();
  clearRadioStatus();
  DW1000Ng::setWait4Response(0);
  DW1000Ng::startReceive();
  receiverArmed = true;
}

size_t buildFrame(uint8_t type, uint16_t sequence,
                  const byte *payload, uint8_t payloadLength) {
  memset(radioTxBuffer, 0, sizeof(radioTxBuffer));
  radioTxBuffer[0] = FRAME_MAGIC_0;
  radioTxBuffer[1] = FRAME_MAGIC_1;
  radioTxBuffer[2] = type;
  radioTxBuffer[3] = LOCAL_NODE_ID;
  radioTxBuffer[4] = REMOTE_NODE_ID;
  radioTxBuffer[5] = static_cast<uint8_t>(sequence & 0xFF);
  radioTxBuffer[6] = static_cast<uint8_t>((sequence >> 8) & 0xFF);
  if (payloadLength > 0 && payload != nullptr) {
    memcpy(radioTxBuffer + LINK_HEADER_SIZE, payload, payloadLength);
  }
  return LINK_HEADER_SIZE + payloadLength;
}

bool parseFrame(const byte *data, uint16_t length, ParsedFrame &frame) {
  if (length < LINK_HEADER_SIZE || length > MAX_LINK_FRAME_SIZE) {
    return false;
  }
  if (data[0] != FRAME_MAGIC_0 || data[1] != FRAME_MAGIC_1) {
    return false;
  }
  const uint8_t type = data[2];
  if (type != FRAME_DATA && type != FRAME_ACK) {
    return false;
  }
  if (data[3] != REMOTE_NODE_ID || data[4] != LOCAL_NODE_ID) {
    return false;
  }

  const size_t payloadLength = length - LINK_HEADER_SIZE;
  if (type == FRAME_ACK && payloadLength != 0) {
    return false;
  }
  if (type == FRAME_DATA &&
      (payloadLength == 0 || payloadLength > MAX_APP_PAYLOAD_SIZE)) {
    return false;
  }

  frame.type = type;
  frame.source = data[3];
  frame.destination = data[4];
  frame.sequence = static_cast<uint16_t>(data[5]) |
                   (static_cast<uint16_t>(data[6]) << 8);
  frame.payloadLength = static_cast<uint8_t>(payloadLength);
  if (payloadLength > 0) {
    memcpy(frame.payload, data + LINK_HEADER_SIZE, payloadLength);
  }
  return true;
}

// Read one completed RX frame while the radio is kept stopped afterward.
// The caller either sends an ACK or explicitly re-arms RX.
bool pollReceivedFrame(ParsedFrame &frame) {
  if (DW1000Ng::isReceiveDone()) {
    const uint16_t length = DW1000Ng::getReceivedDataLength();
    bool valid = false;
    if (length >= LINK_HEADER_SIZE && length <= MAX_LINK_FRAME_SIZE) {
      memset(radioRxBuffer, 0, sizeof(radioRxBuffer));
      DW1000Ng::getReceivedData(radioRxBuffer, length);
      valid = parseFrame(radioRxBuffer, length, frame);
    }
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::forceTRxOff();
    receiverArmed = false;
    if (!valid) {
      startReceiver();
    }
    return valid;
  }

  if (DW1000Ng::isReceiveFailed()) {
    DW1000Ng::clearReceiveFailedStatus();
    startReceiver();
  } else if (DW1000Ng::isReceiveTimeout()) {
    DW1000Ng::clearReceiveTimeoutStatus();
    startReceiver();
  }
  return false;
}

// Transmit one link frame. For DATA, hardware WAIT4RESP opens RX immediately
// after TX; for ACK, RX is re-armed after local transmission completes.
bool transmitFrame(uint8_t type, uint16_t sequence,
                   const byte *payload, uint8_t payloadLength,
                   bool expectResponse) {
  if (payloadLength > MAX_APP_PAYLOAD_SIZE) {
    return false;
  }

  const size_t frameLength =
      buildFrame(type, sequence, payload, payloadLength);

  DW1000Ng::forceTRxOff();
  receiverArmed = false;
  clearRadioStatus();
  DW1000Ng::setWait4Response(
      expectResponse ? RX_AFTER_TX_DELAY_US : 0);
  DW1000Ng::setTransmitData(
      radioTxBuffer, static_cast<uint16_t>(frameLength));
  DW1000Ng::startTransmit();

  const uint32_t deadline = millis() + TX_DONE_TIMEOUT_MS;
  while (!DW1000Ng::isTransmitDone()) {
    if (deadlineReached(millis(), deadline)) {
      DW1000Ng::forceTRxOff();
      DW1000Ng::setWait4Response(0);
      startReceiver();
      return false;
    }
    delayMicroseconds(POLL_DELAY_US);
  }
  DW1000Ng::clearTransmitStatus();

  if (expectResponse) {
    // WAIT4RESP has already placed the hardware into RX after the programmed
    // delay. Do not force TRX off here, or the peer ACK can be discarded.
    receiverArmed = true;
  } else {
    startReceiver();
  }
  return true;
}

bool sendAck(uint16_t sequence) {
  delay(ACK_TURNAROUND_MS);
  return transmitFrame(FRAME_ACK, sequence, nullptr, 0, false);
}

void handleDataFrame(const ParsedFrame &frame) {
  const bool duplicate =
      haveDeliveredSequence && frame.sequence == lastDeliveredSequence;
  if (!duplicate) {
    haveDeliveredSequence = true;
    lastDeliveredSequence = frame.sequence;
    Serial.write(frame.payload, frame.payloadLength);
    Serial.write('\n');
  }

  // A duplicate means the previous ACK was lost. ACK it again, but do not
  // forward the payload to the USB host twice.
  if (!sendAck(frame.sequence)) {
    startReceiver();
  }
}

bool waitForAck(uint16_t sequence) {
  const uint32_t deadline = millis() + ACK_TIMEOUT_MS;
  while (!deadlineReached(millis(), deadline)) {
    ParsedFrame frame{};
    if (pollReceivedFrame(frame)) {
      if (frame.type == FRAME_ACK && frame.sequence == sequence) {
        startReceiver();
        return true;
      }
      if (frame.type == FRAME_DATA) {
        handleDataFrame(frame);
      } else {
        startReceiver();
      }
    }
    delayMicroseconds(POLL_DELAY_US);
  }

  DW1000Ng::forceTRxOff();
  receiverArmed = false;
  startReceiver();
  return false;
}

uint32_t contentionDelayMs(uint8_t attempt) {
  // Host has deterministic priority when both Python bridges submit a line at
  // the same instant. Jetson remains listening longer and can ACK Host first.
  const uint32_t base = LOCAL_NODE_ID == 1 ? 5 : 65;
  const uint32_t span = LOCAL_NODE_ID == 1 ? 25 : 55;
  return base + static_cast<uint32_t>(random(0, span + 1)) +
         static_cast<uint32_t>(attempt) * 12;
}

void listenDuringBackoff(uint32_t durationMs) {
  if (!receiverArmed) {
    startReceiver();
  }
  const uint32_t deadline = millis() + durationMs;
  while (!deadlineReached(millis(), deadline)) {
    ParsedFrame frame{};
    if (pollReceivedFrame(frame)) {
      if (frame.type == FRAME_DATA) {
        handleDataFrame(frame);
      } else {
        startReceiver();
      }
    }
    delayMicroseconds(POLL_DELAY_US);
  }
}

bool sendApplicationPayload(const byte *payload, uint8_t payloadLength) {
  const uint16_t sequence = nextSequence++;

  for (uint8_t attempt = 1; attempt <= MAX_SEND_ATTEMPTS; ++attempt) {
    if (attempt > 1) {
      Serial.print(F("[UWB RETRY] attempt="));
      Serial.println(attempt);
    }

    listenDuringBackoff(contentionDelayMs(attempt - 1));

    if (!transmitFrame(FRAME_DATA, sequence, payload, payloadLength, true)) {
      continue;
    }
    if (waitForAck(sequence)) {
      Serial.println(F("[UWB ACK]"));
      return true;
    }
  }

  Serial.println(F("[UWB ERROR] ack timeout"));
  startReceiver();
  return false;
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
    sendApplicationPayload(
        serialBuffer, static_cast<uint8_t>(serialLength));
  }

  serialLength = 0;
  discardSerialLine = false;
}

void processSerialInput() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\n') {
      finishSerialLine();
      continue;
    }
    if (discardSerialLine) {
      continue;
    }
    if (serialLength < MAX_APP_PAYLOAD_SIZE + 1) {
      serialBuffer[serialLength++] = static_cast<byte>(incoming);
    } else {
      discardSerialLine = true;
    }
  }
}

void processIdleRadio() {
  ParsedFrame frame{};
  if (!pollReceivedFrame(frame)) {
    return;
  }
  if (frame.type == FRAME_DATA) {
    handleDataFrame(frame);
  } else {
    startReceiver();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  randomSeed(esp_random());
  nextSequence = static_cast<uint16_t>(esp_random());

  // ESP32 VSPI defaults are SCK=18, MISO=19, MOSI=23, SS=5. Calling begin
  // explicitly documents and enforces the wiring before DW1000Ng initializes.
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
  DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
  DW1000Ng::applyConfiguration(RADIO_CONFIG);
  DW1000Ng::setNetworkId(UWB_NETWORK_ID);
  DW1000Ng::setDeviceAddress(LOCAL_NODE_ID);
  startReceiver();

  Serial.print(F("[UWB READY] node="));
  Serial.print(LOCAL_NODE_ID);
  Serial.print(F(" peer="));
  Serial.print(REMOTE_NODE_ID);
  Serial.println(F(" driver=dw1000-ng-polling"));
}

void loop() {
  processIdleRadio();
  processSerialInput();
  delayMicroseconds(POLL_DELAY_US);
}
