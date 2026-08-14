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

// Wiring verified with the legacy UWB sender/receiver firmware.
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

// AI Rescue Box currently sends at most 111 application bytes per USB line.
// DATA remains raw over RF because this exact raw path was verified with the
// legacy sender/receiver firmware on the current hardware.
constexpr size_t MAX_APP_PAYLOAD_SIZE = 111;
constexpr size_t MAX_RF_PAYLOAD_SIZE = 120;

// RF-only acknowledgement. This is never forwarded to the USB host.
constexpr char RF_ACK_TOKEN[] = "~AI_RB_ACK~";
constexpr size_t RF_ACK_TOKEN_LENGTH = sizeof(RF_ACK_TOKEN) - 1;

constexpr uint8_t MAX_RETRIES = 4;
constexpr uint8_t MAX_SEND_ATTEMPTS = 1 + MAX_RETRIES;
constexpr uint32_t ACK_TIMEOUT_MS = 300;
constexpr uint32_t TX_WATCHDOG_MS = 1000;
constexpr uint32_t RETRY_BACKOFF_MIN_MS = 20;
constexpr uint32_t RETRY_BACKOFF_MAX_MS = 100;

static_assert(LOCAL_NODE_ID > 0 && REMOTE_NODE_ID > 0,
              "Node IDs must be non-zero");
static_assert(LOCAL_NODE_ID != REMOTE_NODE_ID,
              "Local and peer IDs must be different");
static_assert(RF_ACK_TOKEN_LENGTH <= MAX_RF_PAYLOAD_SIZE,
              "ACK token must fit in one UWB frame");

// DW1000 v0.9 accesses the hardware FCS bytes around the requested payload.
// Keep two accessible safety bytes after every transmit payload.
byte radioTxBuffer[MAX_RF_PAYLOAD_SIZE + 2] = {};
byte radioRxBuffer[MAX_RF_PAYLOAD_SIZE] = {};

byte serialBuffer[MAX_APP_PAYLOAD_SIZE + 1] = {};
size_t serialLength = 0;
bool discardSerialLine = false;

byte outboundPayload[MAX_APP_PAYLOAD_SIZE] = {};
uint8_t outboundLength = 0;
bool outboundActive = false;
bool outboundWaitingForAck = false;
bool outboundSendScheduled = false;
uint8_t outboundAttempts = 0;
uint32_t outboundDeadline = 0;

bool pendingAck = false;

volatile bool irqSent = false;
volatile bool irqReceived = false;
volatile bool irqReceiveFailed = false;

enum class TxKind : uint8_t {
  None,
  Data,
  Ack,
};

TxKind txKind = TxKind::None;
bool txActive = false;
uint32_t txStartedAt = 0;


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


void startPermanentReceiver() {
  DW1000.newReceive();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}


void configureRadio() {
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(LOCAL_NODE_ID);
  DW1000.setNetworkId(UWB_NETWORK_ID);

  // Same radio mode as the legacy sender/receiver pair that works in both
  // directions on this exact ESP32 + DWM1000 hardware.
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();

  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleReceiveFailed);

  // IMPORTANT: leave permanent receive enabled for the lifetime of the radio.
  // DW1000::startTransmit() automatically re-enters RX when this flag is true.
  // This is the same turn-around pattern used by the library MessagePingPong
  // example and avoids the broken manual RX->TX->RX transition seen earlier.
  startPermanentReceiver();
}


void announceReady() {
  Serial.print(F("[UWB READY] node="));
  Serial.print(LOCAL_NODE_ID);
  Serial.print(F(" peer="));
  Serial.println(REMOTE_NODE_ID);
}


void recoverRadio() {
  txActive = false;
  txKind = TxKind::None;
  pendingAck = false;
  clearIrqFlags();

  DW1000.select(PIN_SS);
  configureRadio();
  announceReady();
}


bool startRawTransmit(const byte *payload, size_t payloadLength, TxKind kind) {
  if (txActive || payload == nullptr || payloadLength == 0 ||
      payloadLength > MAX_RF_PAYLOAD_SIZE) {
    return false;
  }

  memset(radioTxBuffer, 0, sizeof(radioTxBuffer));
  memcpy(radioTxBuffer, payload, payloadLength);

  noInterrupts();
  irqSent = false;
  interrupts();

  // Do NOT disable receivePermanently here. With it left enabled, the DW1000
  // library starts RX again automatically immediately after starting TX.
  DW1000.newTransmit();
  DW1000.setData(radioTxBuffer, static_cast<uint16_t>(payloadLength));

  txKind = kind;
  txActive = true;
  txStartedAt = millis();
  DW1000.startTransmit();
  return true;
}


bool isAckToken(const byte *payload, size_t payloadLength) {
  return payloadLength == RF_ACK_TOKEN_LENGTH &&
         memcmp(payload, RF_ACK_TOKEN, RF_ACK_TOKEN_LENGTH) == 0;
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
  outboundActive = true;
  outboundWaitingForAck = false;
  outboundSendScheduled = true;
  outboundAttempts = 0;
  outboundDeadline = millis() + static_cast<uint32_t>(random(3, 20));
}


void processReceivedPacket() {
  const uint16_t payloadLength = DW1000.getDataLength();
  if (payloadLength == 0 || payloadLength > MAX_RF_PAYLOAD_SIZE) {
    return;
  }

  DW1000.getData(radioRxBuffer, payloadLength);

  if (isAckToken(radioRxBuffer, payloadLength)) {
    if (outboundActive) {
      finishOutboundSuccess();
    }
    return;
  }

  // Raw DATA path: exactly what the verified legacy receiver delivered.
  Serial.write(radioRxBuffer, payloadLength);
  Serial.write('\n');

  // Queue a short RF ACK. It gets priority over any local DATA retry.
  pendingAck = true;
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

    // No manual receiver restart here. Because permanent RX stayed enabled,
    // DW1000::startTransmit() already put the hardware back into RX.
    if (completedKind == TxKind::Data && outboundActive) {
      outboundWaitingForAck = true;
      outboundDeadline = millis() + ACK_TIMEOUT_MS;
    }
  }

  // A receive interrupt can be pending together with sent. Handle sent first
  // so txActive is cleared, then consume the received packet.
  if (received && !txActive) {
    processReceivedPacket();
  }

  // The library automatically re-arms permanent RX after failed frames.
  (void)receiveFailed;
}


void processPendingAck() {
  if (!pendingAck || txActive) {
    return;
  }

  if (startRawTransmit(reinterpret_cast<const byte *>(RF_ACK_TOKEN),
                       RF_ACK_TOKEN_LENGTH, TxKind::Ack)) {
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
  recoverRadio();

  if (dataWasActive && outboundActive) {
    scheduleRetry();
  }
}


void processOutbound() {
  if (!outboundActive) {
    return;
  }

  const uint32_t now = millis();

  if (outboundWaitingForAck && deadlineReached(now, outboundDeadline)) {
    scheduleRetry();
  }

  if (outboundActive && outboundSendScheduled && !txActive && !pendingAck &&
      deadlineReached(now, outboundDeadline)) {
    if (startRawTransmit(outboundPayload, outboundLength, TxKind::Data)) {
      outboundAttempts++;
      outboundSendScheduled = false;
      outboundWaitingForAck = false;
    }
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
  // USB-side stop-and-wait: leave later lines queued while one line is waiting
  // for the peer RF ACK. This is what FirmwareLineTransport expects.
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

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SPI_SS);
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  configureRadio();

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
