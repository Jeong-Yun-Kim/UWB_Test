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
constexpr uint16_t MAX_PAYLOAD_SIZE = 120;

byte receivedData[MAX_PAYLOAD_SIZE];
volatile bool packetReceived = false;


void handleReceived() {
  // This callback runs from the DW1000 interrupt. Only set a flag here.
  packetReceived = true;
}


void handleReceiveFailed() {
  // Intentionally empty. In DW1000 0.9, registering this callback lets
  // receivePermanently(true) re-arm RX after a failed frame.
}


void startReceiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
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
  DW1000.setDeviceAddress(2);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();

  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleReceiveFailed);
  startReceiver();
}


void loop() {
  if (packetReceived) {
    packetReceived = false;

    const uint16_t payloadLength = DW1000.getDataLength();
    if (payloadLength > 0 && payloadLength <= MAX_PAYLOAD_SIZE) {
      DW1000.getData(receivedData, payloadLength);

      // The receiver USB Serial is payload-only. Add one newline so that the
      // Python program can read one complete UWB frame per line.
      Serial.write(receivedData, payloadLength);
      Serial.write('\n');
    }
  }

  delay(1);
}
