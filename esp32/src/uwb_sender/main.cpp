#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>

// Keep this wiring identical to the existing UWB anchor main.cpp.
constexpr uint8_t PIN_RST = 15;
constexpr uint8_t PIN_IRQ = 27;
constexpr uint8_t PIN_SS = 5;
constexpr uint8_t PIN_SCK = 18;
constexpr uint8_t PIN_MISO = 19;
constexpr uint8_t PIN_MOSI = 23;
constexpr uint8_t PIN_SPI_SS = 2;

constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr size_t MAX_PAYLOAD_SIZE = 120;

// The DW1000 0.9 byte API accounts for two hardware FCS bytes while copying.
// Reserve those two bytes so the library never reads past this buffer.
char serialBuffer[MAX_PAYLOAD_SIZE + 2] = {};
size_t serialLength = 0;
bool discardCurrentLine = false;
bool transmitInProgress = false;
volatile bool transmitDone = false;


void handleSent() {
  // This callback runs from the DW1000 interrupt. Only set a flag here.
  transmitDone = true;
}


void sendBufferedLine() {
  // Treat only a final CR as part of a CRLF delimiter. Preserve any CR that
  // appears inside the payload.
  if (!discardCurrentLine && serialLength > 0 &&
      serialBuffer[serialLength - 1] == '\r') {
    serialLength--;
  }
  if (serialLength > MAX_PAYLOAD_SIZE) {
    discardCurrentLine = true;
  }

  if (discardCurrentLine) {
    Serial.println(F("[UWB ERROR] payload too long"));
  } else if (serialLength == 0) {
    // Ignore empty lines.
  } else if (transmitInProgress) {
    Serial.println(F("[UWB ERROR] transmitter busy"));
  } else {
    serialBuffer[serialLength] = '\0';
    serialBuffer[serialLength + 1] = '\0';
    DW1000.newTransmit();
    DW1000.setData(reinterpret_cast<byte *>(serialBuffer), serialLength);

    transmitDone = false;
    transmitInProgress = true;
    DW1000.startTransmit();
  }

  serialLength = 0;
  discardCurrentLine = false;
}


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  // PIN_SPI_SS=2 matches the existing SPI.begin() call. The DWM1000 chip
  // select actually used by the driver is PIN_SS=5 below.
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SPI_SS);
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);

  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(1);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
  DW1000.attachSentHandler(handleSent);
}


void loop() {
  if (transmitDone) {
    transmitDone = false;
    transmitInProgress = false;
    // This means that the local DW1000 finished transmitting. It is not a
    // receiver acknowledgement.
    Serial.println(F("[UWB SENT]"));
  }

  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\n') {
      sendBufferedLine();
    } else if (!discardCurrentLine) {
      // Keep one extra byte temporarily so 120-byte payload + CRLF is valid.
      if (serialLength < MAX_PAYLOAD_SIZE + 1) {
        serialBuffer[serialLength++] = incoming;
      } else {
        // Keep discarding until the next newline. No fragmentation is used.
        discardCurrentLine = true;
      }
    }
  }

  delay(1);
}
