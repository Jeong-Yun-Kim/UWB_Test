#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>
#include <Preferences.h>

constexpr uint8_t PIN_RST = 15;
constexpr uint8_t PIN_IRQ = 27;
constexpr uint8_t PIN_SS = 5;
constexpr uint8_t PIN_SCK = 18;
constexpr uint8_t PIN_MISO = 19;
constexpr uint8_t PIN_MOSI = 23;
constexpr uint8_t PIN_SPI_SS = 2;

constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint16_t UWB_NETWORK_ID = 10;
constexpr uint16_t TX_DEVICE_ADDRESS = 1;
constexpr uint16_t RX_DEVICE_ADDRESS = 2;
constexpr size_t MAX_PAYLOAD_SIZE = 120;

Preferences preferences;
bool txRole = false;

char serialBuffer[MAX_PAYLOAD_SIZE + 2] = {};
size_t serialLength = 0;
bool discardCurrentLine = false;
bool transmitInProgress = false;
volatile bool transmitDone = false;

byte receivedData[MAX_PAYLOAD_SIZE] = {};
volatile bool packetReceived = false;

void handleSent() {
  transmitDone = true;
}

void handleReceived() {
  packetReceived = true;
}

void handleReceiveFailed() {
  // Registering the callback lets receivePermanently(true) re-arm RX.
}

void configureCommon(uint16_t address) {
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(address);
  DW1000.setNetworkId(UWB_NETWORK_ID);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
}

void startFixedReceiver() {
  configureCommon(RX_DEVICE_ADDRESS);
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleReceiveFailed);
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void startFixedSender() {
  configureCommon(TX_DEVICE_ADDRESS);
  DW1000.attachSentHandler(handleSent);
}

void changeRole(bool makeTx) {
  preferences.begin("uwb-role", false);
  preferences.putBool("tx", makeTx);
  preferences.end();
  Serial.print(F("[RS REBOOT] next_role="));
  Serial.println(makeTx ? F("TX") : F("RX"));
  Serial.flush();
  delay(100);
  ESP.restart();
}

void sendBufferedLine() {
  if (!discardCurrentLine && serialLength > 0 &&
      serialBuffer[serialLength - 1] == '\r') {
    serialLength--;
  }
  if (serialLength > MAX_PAYLOAD_SIZE) {
    discardCurrentLine = true;
  }

  if (discardCurrentLine) {
    Serial.println(F("[RS ERROR] payload too long"));
  } else if (serialLength == 0) {
    // Ignore empty lines.
  } else {
    serialBuffer[serialLength] = '\0';

    if (strcmp(serialBuffer, "ROLE TX") == 0) {
      changeRole(true);
    } else if (strcmp(serialBuffer, "ROLE RX") == 0) {
      changeRole(false);
    } else if (!txRole) {
      Serial.println(F("[RS ERROR] RX role; use ROLE TX first"));
    } else if (transmitInProgress) {
      Serial.println(F("[RS ERROR] transmitter busy"));
    } else {
      serialBuffer[serialLength + 1] = '\0';
      DW1000.newTransmit();
      DW1000.setData(reinterpret_cast<byte *>(serialBuffer), serialLength);
      transmitDone = false;
      transmitInProgress = true;
      DW1000.startTransmit();
    }
  }

  serialLength = 0;
  discardCurrentLine = false;
}

void processSerial() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n') {
      sendBufferedLine();
    } else if (!discardCurrentLine) {
      if (serialLength < MAX_PAYLOAD_SIZE + 1) {
        serialBuffer[serialLength++] = incoming;
      } else {
        discardCurrentLine = true;
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  preferences.begin("uwb-role", true);
  txRole = preferences.getBool("tx", false);
  preferences.end();

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SPI_SS);
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);

  if (txRole) {
    startFixedSender();
    Serial.println(F("[RS READY] role=TX addr=1"));
  } else {
    startFixedReceiver();
    Serial.println(F("[RS READY] role=RX addr=2"));
  }
}

void loop() {
  if (transmitDone) {
    transmitDone = false;
    transmitInProgress = false;
    Serial.println(F("[RS SENT]"));
  }

  if (packetReceived) {
    packetReceived = false;
    const uint16_t payloadLength = DW1000.getDataLength();
    if (payloadLength > 0 && payloadLength <= MAX_PAYLOAD_SIZE) {
      DW1000.getData(receivedData, payloadLength);
      Serial.print(F("[RS RX] "));
      Serial.write(receivedData, payloadLength);
      Serial.write('\n');
    }
  }

  processSerial();
  delay(1);
}
