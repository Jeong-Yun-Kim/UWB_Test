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

// Wiring verified by the legacy fixed sender/receiver tests in both physical
// directions on the current ESP32 + DWM1000 hardware.
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

// IMPORTANT: the legacy hardware tests established an asymmetric DW1000
// address contract. A frame is reliable when the radio currently transmitting
// uses short address 1 and the radio currently receiving uses short address 2.
// Reversing those short addresses (TX=2, RX=1) failed even with the legacy
// one-way firmware. Logical Host/Jetson identity therefore stays in our own
// link header below; the DW1000 PANADR short address follows the RADIO ROLE.
constexpr uint16_t TX_DEVICE_ADDRESS = 1;
constexpr uint16_t RX_DEVICE_ADDRESS = 2;

// USB-side contract used by AI Rescue Box FirmwareLineTransport.
constexpr size_t MAX_APP_PAYLOAD_SIZE = 111;

// Seven link bytes leave all 111 application bytes available. DW1000 v0.9's
// byte API adds two FCS bytes while copying, hence the extra two backing bytes.
constexpr uint8_t FRAME_MAGIC_0 = 0xA1;
constexpr uint8_t FRAME_MAGIC_1 = 0x52;
constexpr uint8_t FRAME_DATA = 1;
constexpr uint8_t FRAME_ACK = 2;
constexpr size_t LINK_HEADER_SIZE = 7;
constexpr size_t MAX_LINK_FRAME_SIZE = LINK_HEADER_SIZE + MAX_APP_PAYLOAD_SIZE;
constexpr size_t TX_BUFFER_SIZE = MAX_LINK_FRAME_SIZE + 2;

constexpr uint8_t MAX_RETRIES = 4;
constexpr uint8_t MAX_SEND_ATTEMPTS = 1 + MAX_RETRIES;
constexpr uint32_t ACK_TIMEOUT_MS = 1200;
constexpr uint32_t TX_WATCHDOG_MS = 1500;
constexpr uint32_t ACK_TURNAROUND_MS = 25;
constexpr uint32_t RETRY_BACKOFF_MIN_MS = 40;
constexpr uint32_t RETRY_BACKOFF_MAX_MS = 140;
constexpr size_t DUPLICATE_HISTORY_SIZE = 32;

static_assert(LOCAL_NODE_ID > 0 && REMOTE_NODE_ID > 0,
              "Node IDs must be non-zero");
static_assert(LOCAL_NODE_ID != REMOTE_NODE_ID,
              "Local and peer IDs must be different");
static_assert(MAX_LINK_FRAME_SIZE == 118,
              "link framing must preserve the 111-byte application limit");

struct ParsedFrame {
  uint8_t type = 0;
  uint8_t source = 0;
  uint8_t destination = 0;
  uint16_t sequence = 0;
  const byte *payload = nullptr;
  uint8_t payloadLength = 0;
};

enum class TxKind : uint8_t {
  None,
  Data,
  Ack,
};

byte radioTxBuffer[TX_BUFFER_SIZE] = {};
byte radioRxBuffer[MAX_LINK_FRAME_SIZE] = {};

byte serialBuffer[MAX_APP_PAYLOAD_SIZE + 1] = {};
size_t serialLength = 0;
bool discardSerialLine = false;

byte outboundPayload[MAX_APP_PAYLOAD_SIZE] = {};
uint8_t outboundLength = 0;
bool outboundActive = false;
bool outboundWaitingForAck = false;
bool outboundSendScheduled = false;
uint8_t outboundAttempts = 0;
uint16_t outboundSequence = 0;
uint32_t outboundDeadline = 0;

bool pendingAck = false;
uint16_t pendingAckSequence = 0;
uint32_t pendingAckReadyAt = 0;

uint16_t nextSequence = 0;
uint16_t deliveredSequences[DUPLICATE_HISTORY_SIZE] = {};
size_t deliveredSequenceCount = 0;
size_t deliveredSequenceCursor = 0;

TxKind txKind = TxKind::None;
bool txActive = false;
bool receiverArmed = false;
uint32_t txStartedAt = 0;

volatile bool irqSent = false;
volatile bool irqReceived = false;
volatile bool irqReceiveFailed = false;


bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}


void handleSent() {
  irqSent = true;
}


void handleReceived() {
  irqReceived = true;
}


void handleReceiveFailed() {
  irqReceiveFailed = true;
}


void clearIrqFlags() {
  noInterrupts();
  irqSent = false;
  irqReceived = false;
  irqReceiveFailed = false;
  interrupts();
}


// Rebuild the complete radio configuration from a hardware reset. The short
// address is selected by the current RADIO ROLE, not by Host/Jetson identity.
void configureFreshRadio(uint16_t deviceAddress) {
  clearIrqFlags();
  DW1000.select(PIN_SS);

  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(deviceAddress);
  DW1000.setNetworkId(UWB_NETWORK_ID);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

  // setDefaults() enables RX auto-reenable in the cached SYS_CFG. Override it
  // BEFORE commitConfiguration(); doing this afterwards does not write the bit
  // to hardware in arduino-dw1000 v0.9.
  DW1000.receivePermanently(false);
  DW1000.setReceiverAutoReenable(false);
  DW1000.commitConfiguration();

  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleReceiveFailed);
}


void enterReceiveMode() {
  txActive = false;
  txKind = TxKind::None;
  receiverArmed = false;

  // Empirically proven legacy receiver address.
  configureFreshRadio(RX_DEVICE_ADDRESS);
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(false);
  DW1000.startReceive();
  receiverArmed = true;
}


void announceReady() {
  Serial.print(F("[UWB READY] node="));
  Serial.print(LOCAL_NODE_ID);
  Serial.print(F(" peer="));
  Serial.print(REMOTE_NODE_ID);
  Serial.print(F(" tx_addr="));
  Serial.print(TX_DEVICE_ADDRESS);
  Serial.print(F(" rx_addr="));
  Serial.println(RX_DEVICE_ADDRESS);
}


void recoverToReceive() {
  pendingAck = false;
  enterReceiveMode();
  Serial.println(F("[UWB RECOVERED]"));
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
  radioTxBuffer[6] = static_cast<uint8_t>(sequence >> 8);

  if (payloadLength > 0 && payload != nullptr) {
    memcpy(radioTxBuffer + LINK_HEADER_SIZE, payload, payloadLength);
  }
  return LINK_HEADER_SIZE + payloadLength;
}


bool parseFrame(const byte *frame, size_t frameLength, ParsedFrame &parsed) {
  if (frameLength < LINK_HEADER_SIZE || frameLength > MAX_LINK_FRAME_SIZE) {
    return false;
  }
  if (frame[0] != FRAME_MAGIC_0 || frame[1] != FRAME_MAGIC_1) {
    return false;
  }
  if (frame[2] != FRAME_DATA && frame[2] != FRAME_ACK) {
    return false;
  }
  if (frame[3] != REMOTE_NODE_ID || frame[4] != LOCAL_NODE_ID) {
    return false;
  }

  const size_t payloadLength = frameLength - LINK_HEADER_SIZE;
  if (frame[2] == FRAME_ACK && payloadLength != 0) {
    return false;
  }
  if (frame[2] == FRAME_DATA &&
      (payloadLength == 0 || payloadLength > MAX_APP_PAYLOAD_SIZE)) {
    return false;
  }

  parsed.type = frame[2];
  parsed.source = frame[3];
  parsed.destination = frame[4];
  parsed.sequence = static_cast<uint16_t>(frame[5]) |
                    (static_cast<uint16_t>(frame[6]) << 8);
  parsed.payload = frame + LINK_HEADER_SIZE;
  parsed.payloadLength = static_cast<uint8_t>(payloadLength);
  return true;
}


bool wasDelivered(uint16_t sequence) {
  for (size_t index = 0; index < deliveredSequenceCount; ++index) {
    if (deliveredSequences[index] == sequence) {
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


bool startFreshTransmit(uint8_t type, uint16_t sequence,
                        const byte *payload, uint8_t payloadLength,
                        TxKind kind) {
  if (txActive || payloadLength > MAX_APP_PAYLOAD_SIZE) {
    return false;
  }

  const size_t frameLength = buildFrame(type, sequence, payload, payloadLength);

  // Every DATA and ACK transmitter uses the empirically proven legacy sender
  // short address, regardless of whether this physical node is Host or Jetson.
  receiverArmed = false;
  configureFreshRadio(TX_DEVICE_ADDRESS);
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000.setData(radioTxBuffer, static_cast<uint16_t>(frameLength));

  txKind = kind;
  txActive = true;
  txStartedAt = millis();
  DW1000.startTransmit();
  return true;
}


void finishOutboundSuccess() {
  outboundActive = false;
  outboundWaitingForAck = false;
  outboundSendScheduled = false;
  outboundAttempts = 0;
  Serial.println(F("[UWB ACK]"));
}


void finishOutboundFailure() {
  outboundActive = false;
  outboundWaitingForAck = false;
  outboundSendScheduled = false;
  outboundAttempts = 0;
  Serial.println(F("[UWB ERROR] ack timeout"));
  recoverToReceive();
}


void scheduleRetry() {
  outboundWaitingForAck = false;
  if (outboundAttempts >= MAX_SEND_ATTEMPTS) {
    finishOutboundFailure();
    return;
  }

  Serial.print(F("[UWB RETRY] attempt="));
  Serial.println(outboundAttempts + 1);
  outboundSendScheduled = true;
  outboundDeadline = millis() +
      static_cast<uint32_t>(random(RETRY_BACKOFF_MIN_MS,
                                   RETRY_BACKOFF_MAX_MS + 1));
}


void queueOutbound(const byte *payload, uint8_t payloadLength) {
  memcpy(outboundPayload, payload, payloadLength);
  outboundLength = payloadLength;
  outboundSequence = nextSequence++;
  outboundActive = true;
  outboundWaitingForAck = false;
  outboundSendScheduled = true;
  outboundAttempts = 0;
  outboundDeadline = millis() + static_cast<uint32_t>(random(5, 25));
}


void processReceivedPacket() {
  const uint16_t frameLength = DW1000.getDataLength();
  if (frameLength == 0 || frameLength > MAX_LINK_FRAME_SIZE) {
    enterReceiveMode();
    return;
  }

  DW1000.getData(radioRxBuffer, frameLength);
  ParsedFrame frame{};
  if (!parseFrame(radioRxBuffer, frameLength, frame)) {
    enterReceiveMode();
    return;
  }

  if (frame.type == FRAME_ACK) {
    if (outboundActive && frame.sequence == outboundSequence) {
      finishOutboundSuccess();
    }
    enterReceiveMode();
    return;
  }

  const bool duplicate = wasDelivered(frame.sequence);
  if (!duplicate) {
    rememberDelivered(frame.sequence);
    Serial.write(frame.payload, frame.payloadLength);
    Serial.write('\n');
  }

  // Do not re-enter RX before ACK. The next radio role is a fresh transmitter.
  pendingAck = true;
  pendingAckSequence = frame.sequence;
  pendingAckReadyAt = millis() + ACK_TURNAROUND_MS;
}


void processRadioEvents() {
  bool sent = false;
  bool received = false;
  bool receiveFailed = false;

  noInterrupts();
  sent = irqSent;
  received = irqReceived;
  receiveFailed = irqReceiveFailed;
  irqSent = false;
  irqReceived = false;
  irqReceiveFailed = false;
  interrupts();

  if (sent) {
    const TxKind completedKind = txKind;
    txActive = false;
    txKind = TxKind::None;

    if (completedKind == TxKind::Data && outboundActive) {
      // A DATA sender immediately becomes the proven address-2 receiver so it
      // can hear the peer's address-1 ACK.
      enterReceiveMode();
      outboundWaitingForAck = true;
      outboundDeadline = millis() + ACK_TIMEOUT_MS;
    } else {
      // ACK sender also returns to the proven receive role.
      enterReceiveMode();
    }
  }

  if (received) {
    receiverArmed = false;
    processReceivedPacket();
  }

  if (receiveFailed) {
    receiverArmed = false;
    enterReceiveMode();
  }
}


void processPendingAck() {
  if (!pendingAck || txActive ||
      !deadlineReached(millis(), pendingAckReadyAt)) {
    return;
  }

  if (startFreshTransmit(FRAME_ACK, pendingAckSequence, nullptr, 0,
                         TxKind::Ack)) {
    pendingAck = false;
  }
}


void processTxWatchdog() {
  if (!txActive ||
      !deadlineReached(millis(), txStartedAt + TX_WATCHDOG_MS)) {
    return;
  }

  const bool dataWasActive = txKind == TxKind::Data && outboundActive;
  Serial.println(F("[UWB TX TIMEOUT]"));
  enterReceiveMode();

  if (dataWasActive && outboundActive) {
    scheduleRetry();
  }
}


bool receiveEventPending() {
  bool pending = false;
  noInterrupts();
  pending = irqReceived || irqReceiveFailed;
  interrupts();
  return pending;
}


void processOutbound() {
  if (!outboundActive) {
    return;
  }

  const uint32_t now = millis();
  if (outboundWaitingForAck && deadlineReached(now, outboundDeadline)) {
    scheduleRetry();
  }

  if (!outboundActive || !outboundSendScheduled || txActive || pendingAck ||
      !deadlineReached(now, outboundDeadline)) {
    return;
  }

  // Give a just-completed RX event priority so a reset does not discard it.
  if (receiveEventPending()) {
    return;
  }

  if (startFreshTransmit(FRAME_DATA, outboundSequence,
                         outboundPayload, outboundLength, TxKind::Data)) {
    outboundAttempts++;
    outboundSendScheduled = false;
    outboundWaitingForAck = false;
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
  if (outboundActive) {
    return;
  }

  while (Serial.available() > 0 && !outboundActive) {
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


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  randomSeed(esp_random());
  nextSequence = static_cast<uint16_t>(esp_random());

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SPI_SS);
  DW1000.begin(PIN_IRQ, PIN_RST);

  enterReceiveMode();
  announceReady();
}


void loop() {
  processRadioEvents();
  processTxWatchdog();
  processPendingAck();
  processOutbound();
  processSerialInput();
  delay(1);
}
