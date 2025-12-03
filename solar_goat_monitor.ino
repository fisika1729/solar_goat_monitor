#include <Arduino.h>
#include "ble_serial.h"
#include "rfid.h"
#include "simcom.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println(F("\n=== UHF RFID + ADC + BUZZER + SIMA7680C (ESP32-C3) ==="));

  // Start BLE debug (shows up as "ESP32C3-RFID")
  bleDebugBegin("ESP32C3-RFID");

  // RFID + ADC + buzzer
  rfid_setup();

  // SIM module
  sim_setup();
}

void loop() {
  rfid_loop();
  sim_loop();
  // BLE is event-driven, nothing explicit required here
  delay(1);
}



