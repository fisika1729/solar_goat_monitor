#pragma once
#include <Arduino.h>

/*
   SIMA7680C / SIMCom AT module for ESP32-C3

   - Uses UART0 via HardwareSerial(0) on pins 11/12.
   - No power-control pins (assumes module is always powered).
*/

HardwareSerial simSerial(0);   // UART0

#define SIM_RX_PIN     12      // ESP32-C3 RX from SIM TX
#define SIM_TX_PIN     11      // ESP32-C3 TX to SIM RX

#define SIM_BAUDRATE       115200
#define SIM_AT_TIMEOUT_MS  5000

// ===== LOW-LEVEL HELPERS =====

static bool simWaitFor(const char *expect, uint32_t timeoutMs) {
  uint32_t start = millis();
  String buffer;

  while (millis() - start < timeoutMs) {
    while (simSerial.available()) {
      char c = (char)simSerial.read();
      buffer += c;
      if (c == '\n') {
        String line = buffer;
        buffer = "";
        line.trim();
        if (!line.length()) continue;

        Serial.print(F("[SIM RX] "));
        Serial.println(line);

        if (line.indexOf(expect) >= 0) return true;
        if (line.indexOf(F("ERROR")) >= 0 ||
            line.indexOf(F("+CME ERROR")) >= 0) {
          return false;
        }
      }
    }
  }
  return false;
}

static bool simSendAT(const char *cmd, const char *expect, uint32_t timeoutMs = SIM_AT_TIMEOUT_MS) {
  Serial.print(F("[SIM TX] "));
  Serial.println(cmd);

  while (simSerial.available()) simSerial.read();

  simSerial.print(cmd);
  simSerial.print("\r\n");

  if (!expect || !strlen(expect)) {
    delay(100);
    return true;
  }

  bool ok = simWaitFor(expect, timeoutMs);
  Serial.print(F("[SIM] cmd '"));
  Serial.print(cmd);
  Serial.print(F("' -> "));
  Serial.println(ok ? F("OK") : F("FAIL"));
  return ok;
}

static bool simInit() {
  Serial.println(F("[SIM] Initializing SIMCom"));

  // Basic "AT" sync
  for (int i = 0; i < 5; ++i) {
    if (simSendAT("AT", "OK", 1000)) break;
    delay(500);
    if (i == 4) {
      Serial.println(F("[SIM] No response to AT"));
      return false;
    }
  }

  simSendAT("ATE0", "OK", 1000);          // echo off
  simSendAT("ATI", "OK", 2000);           // module info
  simSendAT("AT+CPIN?", "READY", 5000);   // SIM ready
  simSendAT("AT+CREG?", "0,1", 5000);     // home network
  simSendAT("AT+CREG?", "0,5", 5000);     // roaming
  simSendAT("AT+CGREG?", "0,1", 5000);    // data reg
  simSendAT("AT+CGREG?", "0,5", 5000);
  simSendAT("AT+CSQ", "OK", 2000);        // signal quality

  // TODO: Add APN / MQTT / HTTP config from your Motor_board_ESP2.ino

  Serial.println(F("[SIM] Basic init complete"));
  return true;
}

static void simHandleUrc() {
  while (simSerial.available()) {
    String line = simSerial.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    Serial.print(F("[SIM URC] "));
    Serial.println(line);
  }
}

// ===== PUBLIC API =====

inline void sim_setup() {
  simSerial.begin(SIM_BAUDRATE, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  Serial.println(F("[SIM] UART started (UART0 @ 11/12)"));

  if (simInit()) {
    Serial.println(F("[SIM] SIMCom init OK"));
  } else {
    Serial.println(F("[SIM] SIMCom init FAILED"));
  }
}

inline void sim_loop() {
  simHandleUrc();
}
