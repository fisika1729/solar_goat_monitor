/*
   UHF RFID Reader + ADC Monitor - ESP32-C3 Arduino

   Features:
   - Serial1 UART to UHF RFID reader (UHF protocol: A5 5A ... CHK 0D 0A)
   - RESET command (0x68) with response 0x69 (like Python _reset_reader)
   - Continuous inventory (0x82 / 0x83), EPC + RSSI parsing
   - ADC measurement with resistor divider for up to ~14.5 V

   Make sure to:
   - Set RFID_RX_PIN, RFID_TX_PIN, RFID_POWER_PIN, ADC_PIN correctly.
   - Ensure hardware divider keeps ADC pin <= 3.3 V.
*/

#include <Arduino.h>

// ===================== USER CONFIG =====================

// UART pins (CHANGE to match your wiring)
#define RFID_RX_PIN        18   // ESP32-C3 RX from reader TX
#define RFID_TX_PIN        19   // ESP32-C3 TX to reader RX

// Power control pin (set to -1 if not used)
#define RFID_POWER_PIN     3   // GPIO controlling reader VCC / EN
// If the module is always powered, set this to -1 and ignore power control

// UART parameters
#define RFID_BAUDRATE      115200

// Heartbeat timeout
#define HEARTBEAT_TIMEOUT_MS   3000

// Max tags to keep in buffer
#define MAX_TAGS           16

// ===================== ADC CONFIG =====================

// ADC pin (must be ADC-capable on ESP32-C3)
#define ADC_PIN            0    // e.g., GPIO4 (ADC1_CH4) – change if needed

// ADC characteristics
#define ADC_RES_BITS       12
#define ADC_MAX_READING    ((1 << ADC_RES_BITS) - 1)
#define ADC_VREF           3.3f

// Voltage divider: Vin ---[R1]---+---[R2]--- GND, ADC at node +
// Example: R1 = 36k, R2 = 10k → ~15.2 V max at 3.3 V ADC
#define R1_VALUE           36000.0f
#define R2_VALUE           10000.0f

// ===================== PROTOCOL CONSTANTS =====================

#define UHF_CMD_GET_FIRMWARE_VERSION        0x02
#define UHF_CMD_SET_REGION                  0x07
#define UHF_CMD_GET_HARDWARE_VERSION        0x20
#define UHF_CMD_GET_CW_SETTING              0x26
#define UHF_CMD_CONTINUOUS_INVENTORY        0x82
#define UHF_CMD_CONTINUOUS_INVENTORY_RESP   0x83
#define UHF_CMD_STOP_INVENTORY              0x8C
#define UHF_CMD_STOP_INVENTORY_RESP         0x8D
#define UHF_CMD_RESET_RESPONSE              0x69
#define UHF_CMD_SET_POWER                   0x2F
#define UHF_CMD_RESET                       0x68
#define UHF_CMD_HEARTBEAT                   0x0F
#define UHF_CMD_HEARTBEAT_SWITCH            0x0C

#define UHF_RESP_SUCCESS                    0x00
#define UHF_RESP_INVENTORY_SUCCESS          0x01

// Heartbeat switch params template
static const uint8_t HEARTBEAT_SWITCH_PARAMS_TEMPLATE[7] = {
  0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ===================== TAG STRUCT =====================

struct UhfTag {
  String epc;
  uint8_t antenna;
  float rssiDbm;
};

UhfTag g_tags[MAX_TAGS];
size_t g_tagCount = 0;
unsigned long g_lastUartRxMs = 0;
unsigned long g_lastAdcPrintMs = 0;

// ===================== UTILS =====================

uint8_t calcChecksum(const uint8_t *data, size_t len) {
  uint8_t csum = 0;
  for (size_t i = 0; i < len; ++i) {
    csum ^= data[i];
  }
  return csum;
}

// Blocking read with timeout (ms)
bool readBytesWithTimeout(Stream &s, uint8_t *buf, size_t len, uint32_t timeoutMs) {
  unsigned long start = millis();
  size_t received = 0;

  while (received < len) {
    if (s.available()) {
      buf[received++] = (uint8_t)s.read();
    } else {
      if (millis() - start > timeoutMs) {
        return false;
      }
      delay(1);
    }
  }
  return true;
}

// ===================== POWER CONTROL =====================

void rfidPowerOn() {
  if (RFID_POWER_PIN >= 0) {
    digitalWrite(RFID_POWER_PIN, HIGH);
    Serial.println(F("[RFID] Power ON"));
  }
}

void rfidPowerOff() {
  if (RFID_POWER_PIN >= 0) {
    digitalWrite(RFID_POWER_PIN, LOW);
    Serial.println(F("[RFID] Power OFF"));
  }
}

// ===================== ADC MEASUREMENT =====================

// Reads ADC, converts to Vin using divider formula
float readInputVoltage() {
  int raw = analogRead(ADC_PIN);  // 0 .. 4095
  float vAdc = ((float)raw / (float)ADC_MAX_READING) * ADC_VREF;

  // Vin = Vadc * (R1 + R2) / R2
  float vin = vAdc * ((R1_VALUE + R2_VALUE) / R2_VALUE);

  return vin;
}

// ===================== COMMAND SEND =====================

/*
   sendCommand:
   Builds frame: A5 5A LEN_H LEN_L CMD PARAMS... CHK 0D 0A
   Reads one response frame and returns payload in respBuf:
   respBuf = [CMD, data...], length in respLen (without checksum/0D0A)
*/
bool sendCommand(uint8_t cmd,
                 const uint8_t *params, size_t paramsLen,
                 uint8_t *respBuf, size_t respBufSize, size_t &respLen,
                 int retries = 3) {

  // Length is total frame bytes (header+length+cmd+params+chk+end)
  uint16_t frameLen = 2 + 2 + 1 + paramsLen + 1 + 2;
  if (frameLen > 256) {
    Serial.println(F("[RFID] Frame too big"));
    return false;
  }

  uint8_t frame[256];

  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = (frameLen >> 8) & 0xFF;
  frame[3] = frameLen & 0xFF;
  frame[4] = cmd;
  if (paramsLen && params) {
    memcpy(&frame[5], params, paramsLen);
  }

  uint8_t chk = calcChecksum(&frame[2], 2 + 1 + paramsLen);
  frame[5 + paramsLen] = chk;
  frame[6 + paramsLen] = 0x0D;
  frame[7 + paramsLen] = 0x0A;

  int toWrite = 8 + paramsLen;

  for (int attempt = 0; attempt < retries; ++attempt) {
    while (Serial1.available()) Serial1.read();

    Serial.print(F("[RFID] Send CMD=0x"));
    Serial.print(cmd, HEX);
    Serial.print(F(" attempt "));
    Serial.println(attempt + 1);

    Serial1.write(frame, toWrite);
    Serial1.flush();

    delay(40);  // small wait

    uint8_t header[4];
    if (!readBytesWithTimeout(Serial1, header, 4, 50)) {
      Serial.println(F("[RFID] No response header"));
      continue;
    }

    if (header[0] != 0xA5 || header[1] != 0x5A) {
      Serial.print(F("[RFID] Bad header: "));
      Serial.print(header[0], HEX);
      Serial.print(' ');
      Serial.println(header[1], HEX);
      continue;
    }

    uint16_t rxFrameLen = ((uint16_t)header[2] << 8) | header[3];
    if (rxFrameLen < 7) {
      Serial.print(F("[RFID] Invalid rx len: "));
      Serial.println(rxFrameLen);
      continue;
    }

    uint16_t remaining = rxFrameLen - 4;
    if (remaining > 252) remaining = 252;
    uint8_t respRaw[252];

    if (!readBytesWithTimeout(Serial1, respRaw, remaining, 50)) {
      Serial.println(F("[RFID] Incomplete frame"));
      continue;
    }

    if (remaining < 3) {
      Serial.println(F("[RFID] Frame too short"));
      continue;
    }

    uint8_t rxChk = respRaw[remaining - 3];

    uint8_t chkBuf[2 + 249];
    size_t chkLen = 0;
    chkBuf[chkLen++] = header[2];
    chkBuf[chkLen++] = header[3];
    memcpy(&chkBuf[chkLen], respRaw, remaining - 3);
    chkLen += (remaining - 3);

    uint8_t calc = calcChecksum(chkBuf, chkLen);
    if (calc != rxChk) {
      Serial.print(F("[RFID] Checksum fail calc="));
      Serial.print(calc, HEX);
      Serial.print(F(" rx="));
      Serial.println(rxChk, HEX);
      continue;
    }

    size_t payloadLen = remaining - 3;   // strip chk+0D+0A
    if (payloadLen > respBufSize) {
      Serial.println(F("[RFID] respBuf too small"));
      return false;
    }

    memcpy(respBuf, respRaw, payloadLen);
    respLen = payloadLen;
    g_lastUartRxMs = millis();

    Serial.print(F("[RFID] CMD 0x"));
    Serial.print(cmd, HEX);
    Serial.print(F(" resp OK, len="));
    Serial.println(payloadLen);

    return true;
  }

  Serial.print(F("[RFID] CMD 0x"));
  Serial.print(cmd, HEX);
  Serial.println(F(" failed after retries"));
  return false;
}

// ===================== HIGH-LEVEL COMMANDS =====================

bool resetReader() {
  uint8_t resp[32];
  size_t respLen = 0;

  Serial.println(F("[RFID] Sending RESET (0x68)"));

  // This uses the same generic frame builder as _send_command(0x68) in Python.
  if (!sendCommand(UHF_CMD_RESET, nullptr, 0, resp, sizeof(resp), respLen, 3)) {
    Serial.println(F("[RFID] Reset command send failed"));
    return false;
  }

  // Expect 0x69 as first byte (CMD_RESET_RESPONSE)
  if (respLen > 0 && resp[0] == UHF_CMD_RESET_RESPONSE) {
    Serial.println(F("[RFID] Reader reset successful"));
    delay(1000);  // allow module to reboot
    return true;
  }

  Serial.print(F("[RFID] Unexpected reset response: "));
  if (respLen > 0) {
    Serial.print(F("0x"));
    Serial.println(resp[0], HEX);
  } else {
    Serial.println(F("NO DATA"));
  }
  return false;
}

bool setPowerMax() {
  Serial.println(F("[RFID] Setting power levels..."));
  const uint8_t powerLevels[][3] = {
    {0x01, 0x1E, 0x1E}, // 30 dBm
    {0x01, 0x1A, 0x1A}, // 26 dBm
    {0x01, 0x14, 0x14}, // 20 dBm
  };

  uint8_t resp[32];
  size_t respLen;

  for (int i = 0; i < 3; ++i) {
    Serial.print(F("[RFID] Trying power "));
    Serial.println(powerLevels[i][1], DEC);

    if (!sendCommand(UHF_CMD_SET_POWER,
                     powerLevels[i], sizeof(powerLevels[i]),
                     resp, sizeof(resp), respLen, 3)) {
      continue;
    }

    if (respLen > 0 && resp[0] == UHF_RESP_SUCCESS) {
      Serial.print(F("[RFID] Power set to "));
      Serial.println(powerLevels[i][1], DEC);
      return true;
    }
  }

  Serial.println(F("[RFID] All set power attempts failed"));
  return false;
}

bool stopInventory() {
  uint8_t resp[32];
  size_t respLen;

  if (!sendCommand(UHF_CMD_STOP_INVENTORY,
                   nullptr, 0,
                   resp, sizeof(resp), respLen, 3)) {
    return false;
  }

  if (!respLen) return false;

  if (resp[0] == UHF_CMD_STOP_INVENTORY_RESP) {
    if (respLen > 1 && resp[1] == UHF_RESP_INVENTORY_SUCCESS) {
      Serial.println(F("[RFID] Inventory stopped (specific)"));
      delay(500);
      return true;
    }
  } else if (resp[0] == UHF_RESP_SUCCESS) {
    Serial.println(F("[RFID] Inventory stopped (generic)"));
    delay(500);
    return true;
  }

  Serial.print(F("[RFID] Unexpected stop resp: 0x"));
  Serial.println(resp[0], HEX);
  return false;
}

bool setHeartbeatSwitch(bool enable) {
  uint8_t params[7];
  memcpy(params, HEARTBEAT_SWITCH_PARAMS_TEMPLATE, sizeof(params));
  params[1] = enable ? 0x01 : 0x00;

  uint8_t resp[32];
  size_t respLen;

  if (!sendCommand(UHF_CMD_HEARTBEAT_SWITCH,
                   params, sizeof(params),
                   resp, sizeof(resp), respLen, 3)) {
    return false;
  }

  if (!respLen) return false;

  if (resp[0] == UHF_RESP_SUCCESS || resp[0] == UHF_CMD_HEARTBEAT_SWITCH) {
    Serial.print(F("[RFID] Heartbeat switch "));
    Serial.println(enable ? F("ON") : F("OFF"));
    return true;
  }

  Serial.print(F("[RFID] Unexpected HB resp: 0x"));
  Serial.println(resp[0], HEX);
  return false;
}

bool startInventory(uint16_t count) {
  uint8_t params[2] = {
    (uint8_t)((count >> 8) & 0xFF),
    (uint8_t)(count & 0xFF)
  };

  uint8_t resp[64];
  size_t respLen;

  if (!sendCommand(UHF_CMD_CONTINUOUS_INVENTORY,
                   params, sizeof(params),
                   resp, sizeof(resp), respLen, 3)) {
    return false;
  }

  if (!respLen) return false;

  if (resp[0] == UHF_CMD_CONTINUOUS_INVENTORY_RESP ||
      resp[0] == UHF_RESP_SUCCESS) {
    Serial.print(F("[RFID] Inventory started (count="));
    Serial.print(count);
    Serial.println(')');
    return true;
  }

  Serial.print(F("[RFID] Unexpected inv resp: 0x"));
  Serial.println(resp[0], HEX);
  return false;
}

// ===================== TAG PARSING =====================

void addTagIfNew(const UhfTag &tag) {
  if (g_tagCount > 0 && g_tags[g_tagCount - 1].epc == tag.epc) {
    return;
  }

  if (g_tagCount < MAX_TAGS) {
    g_tags[g_tagCount++] = tag;
  } else {
    g_tags[g_tagCount - 1] = tag;
  }

  Serial.print(F("[TAG] EPC="));
  Serial.print(tag.epc);
  Serial.print(F(" RSSI=-"));
  Serial.print(tag.rssiDbm, 3);
  Serial.print(F(" dBm ANT="));
  Serial.println(tag.antenna);
}

void processContinuousInventoryResponse(const uint8_t *resp, size_t respLen) {
  if (respLen < 5) return;

  uint8_t epcLen = (resp[1] >> 3) * 2;
  if (epcLen == 0) return;
  if (respLen < (size_t)(2 + epcLen)) return;

  // Optional EPC prefix filter (52 54); comment out if not needed
  if (!(resp[3] == 0x52 && resp[4] == 0x54)) {
    // return;
  }

  const uint8_t *epcPtr = &resp[3];
  size_t epcBytes = epcLen;

  uint16_t rssiRaw = 0;
  uint8_t antenna = 0;

  if (respLen >= (size_t)(2 + epcLen + 3)) {
    size_t rssiHighIdx = 2 + epcLen + 1;
    size_t rssiLowIdx  = 2 + epcLen + 2;
    size_t antIdx      = 2 + epcLen + 3;
    if (antIdx < respLen) {
      uint8_t rssiHigh = resp[rssiHighIdx];
      uint8_t rssiLow  = resp[rssiLowIdx];
      rssiRaw = ((uint16_t)rssiHigh << 8) | rssiLow;
      antenna = resp[antIdx];
    }
  }

  UhfTag tag;
  tag.epc.reserve(epcBytes * 2);

  for (size_t i = 0; i < epcBytes; ++i) {
    if (epcPtr + i >= resp + respLen) break;
    if (epcPtr[i] < 0x10) tag.epc += '0';
    tag.epc += String(epcPtr[i], HEX);
  }
  tag.epc.toUpperCase();

  tag.antenna = antenna;
  tag.rssiDbm = rssiRaw / 1000.0f;

  addTagIfNew(tag);
}

void readTagsLoop() {
  while (Serial1.available() >= 4) {
    uint8_t header[4];

    if (!readBytesWithTimeout(Serial1, header, 4, 10)) {
      return;
    }

    if (header[0] != 0xA5 || header[1] != 0x5A) {
      Serial.print(F("[RFID] Bad header in loop: "));
      Serial.print(header[0], HEX);
      Serial.print(' ');
      Serial.println(header[1], HEX);
      continue;
    }

    uint16_t frameLen = ((uint16_t)header[2] << 8) | header[3];
    if (frameLen < 7) {
      Serial.print(F("[RFID] Bad frameLen "));
      Serial.println(frameLen);
      continue;
    }

    uint16_t remaining = frameLen - 4;
    if (remaining > 256) remaining = 256;
    uint8_t buf[256];

    if (!readBytesWithTimeout(Serial1, buf, remaining, 20)) {
      Serial.println(F("[RFID] Short read in loop"));
      continue;
    }

    if (remaining < 3) continue;
    uint8_t rxChk = buf[remaining - 3];

    uint8_t chkBuf[2 + 253];
    size_t chkLen = 0;
    chkBuf[chkLen++] = header[2];
    chkBuf[chkLen++] = header[3];
    memcpy(&chkBuf[chkLen], buf, remaining - 3);
    chkLen += (remaining - 3);
    uint8_t calc = calcChecksum(chkBuf, chkLen);

    if (calc != rxChk) {
      Serial.println(F("[RFID] Checksum fail in loop"));
      continue;
    }

    g_lastUartRxMs = millis();

    uint8_t cmd = buf[0];
    size_t payloadLen = remaining - 3;

    if (cmd == UHF_CMD_CONTINUOUS_INVENTORY_RESP) {
      processContinuousInventoryResponse(buf, payloadLen);
    } else if (cmd == UHF_CMD_HEARTBEAT) {
      Serial.println(F("[RFID] Heartbeat frame"));
    }
  }
}

// ===================== SETUP & LOOP =====================

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(F("\n=== UHF RFID Reader (ESP32-C3) + ADC + RESET ==="));

  if (RFID_POWER_PIN >= 0) {
    pinMode(RFID_POWER_PIN, OUTPUT);
    rfidPowerOff();
  }

  // ADC init
  analogReadResolution(ADC_RES_BITS);

  delay(500);

  // UART init
  Serial1.begin(RFID_BAUDRATE, SERIAL_8N1, RFID_RX_PIN, RFID_TX_PIN);
  Serial.println(F("[RFID] Serial1 started"));

  // Power cycle + reset like Python
  rfidPowerOff();
  delay(1000);
  rfidPowerOn();
  delay(500);

  // Send RESET
  resetReader();

  // Stop any inventory, then heartbeat, power, inventory
  stopInventory();
  setHeartbeatSwitch(true);
  setPowerMax();

  if (!startInventory(1000)) {
    Serial.println(F("[RFID] Failed to start inventory"));
  }

  g_lastUartRxMs = millis();
  g_lastAdcPrintMs = millis();
}

void loop() {
  // RFID frames: tags + heartbeat
  readTagsLoop();

  // Heartbeat timeout check
  if (millis() - g_lastUartRxMs > HEARTBEAT_TIMEOUT_MS) {
    Serial.println(F("[RFID] Heartbeat timeout!"));
    g_lastUartRxMs = millis();

    // Simple re-init: power cycle + reset + restart inventory
    rfidPowerOff();
    delay(500);
    rfidPowerOn();
    delay(500);
    resetReader();
    stopInventory();
    setHeartbeatSwitch(true);
    setPowerMax();
    startInventory(1000);
  }

  // ADC print every 1 second
  if (millis() - g_lastAdcPrintMs > 1000) {
    g_lastAdcPrintMs = millis();
    float vin = readInputVoltage();
    Serial.print(F("[ADC] Vin ≈ "));
    Serial.print(vin, 2);
    Serial.println(F(" V"));
  }

  delay(1);
}
