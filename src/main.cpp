#include <Arduino.h>

HardwareSerial Rx(PB5, PB6, NC, NC);

void setup() {
  Serial.begin(115200);
  Rx.setRx(PB5);
  Rx.setTx(PB6);
  Rx.begin(115200);
}

void loop() {
  while (Rx.available()) {
    uint8_t b = Rx.read();
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(' ');
  }
}