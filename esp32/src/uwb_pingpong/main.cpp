#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>

#ifndef PINGPONG_INITIATOR
#error "PINGPONG_INITIATOR must be supplied by platformio.ini"
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
constexpr uint32_t RESPONSE_DELAY_MS = 80;
constexpr uint32_t WATCHDOG_MS = 2500;
constexpr size_t MAX_PAYLOAD_SIZE = 64;

volatile bool sentEvent = false;
volatile bool receivedEvent = false;
volatile bool receiveFailedEvent = false;

bool waitingForReply = false;
uint32_t lastActivityMs = 0;
uint32_t counter = 0;
char rxBuffer[MAX_PAYLOAD_SIZE + 1] = {};
char txBuffer[MAX_PAYLOAD_SIZE + 2] = {};

void handleSent() {
  sentEvent = true;
}

void handleReceived() {
  receivedEvent = true;
}

void handleReceiveFailed() {
  receiveFailedEvent = true;
}

void startReceiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void sendMessage(const char *prefix, uint32_t value) {
  const int written = snprintf(txBuffer, MAX_PAYLOAD_SIZE + 1, "%s:%lu", prefix,
                               static_cast<unsigned long>(value));
  if (written <= 0 || written > static_cast<int>(MAX_PAYLOAD_SIZE)) {
    Serial.println(F("[PP ERROR] format"));
    return;
  }

  // Match the upstream MessagePingPong pattern: permanent receive remains
  // enabled and startTransmit() automatically returns the DW1000 to RX.
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000.setData(reinterpret_cast<byte *>(txBuffer),
                 static_cast<uint16_t>(written));
  sentEvent = false;
  DW1000.startTransmit();
  lastActivityMs = millis();

  Serial.print(F("[PP TX] "));
  Serial.println(txBuffer);
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SPI_SS);
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);

  DW1000.newConfiguration();
  DW1000.setDefaults();
  // Frame filtering is disabled by setDefaults(), matching the upstream
  // MessagePingPong example. The short address is therefore not used to route
  // these raw test frames.
  DW1000.setDeviceAddress(1);
  DW1000.setNetworkId(UWB_NETWORK_ID);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();

  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleReceiveFailed);
  startReceiver();

#if PINGPONG_INITIATOR
  Serial.println(F("[PP READY] initiator"));
  delay(500);
  waitingForReply = true;
  sendMessage("PING", counter);
#else
  Serial.println(F("[PP READY] responder"));
#endif
}

void loop() {
  if (receiveFailedEvent) {
    receiveFailedEvent = false;
    Serial.println(F("[PP RX FAIL]"));
  }

  if (sentEvent) {
    sentEvent = false;
    Serial.println(F("[PP TX DONE]"));
  }

  if (receivedEvent) {
    receivedEvent = false;
    const uint16_t length = DW1000.getDataLength();
    if (length == 0 || length > MAX_PAYLOAD_SIZE) {
      Serial.print(F("[PP RX BADLEN] "));
      Serial.println(length);
    } else {
      memset(rxBuffer, 0, sizeof(rxBuffer));
      DW1000.getData(reinterpret_cast<byte *>(rxBuffer), length);
      rxBuffer[length] = '\0';
      lastActivityMs = millis();

      Serial.print(F("[PP RX] "));
      Serial.println(rxBuffer);

#if PINGPONG_INITIATOR
      if (strncmp(rxBuffer, "PONG:", 5) == 0) {
        waitingForReply = false;
        counter++;
        delay(RESPONSE_DELAY_MS);
        waitingForReply = true;
        sendMessage("PING", counter);
      }
#else
      if (strncmp(rxBuffer, "PING:", 5) == 0) {
        const uint32_t value = strtoul(rxBuffer + 5, nullptr, 10);
        delay(RESPONSE_DELAY_MS);
        sendMessage("PONG", value);
      }
#endif
    }
  }

#if PINGPONG_INITIATOR
  if (waitingForReply && millis() - lastActivityMs > WATCHDOG_MS) {
    Serial.print(F("[PP TIMEOUT] counter="));
    Serial.println(counter);
    delay(RESPONSE_DELAY_MS);
    sendMessage("PING", counter);
  }
#endif

  delay(1);
}
