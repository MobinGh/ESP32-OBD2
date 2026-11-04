
#include <Arduino.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error This firmware requires ESP32 Bluetooth Classic SPP.
#endif

BluetoothSerial obd;

uint8_t peer[6] = {
  0x01, 0x23, 0x45, 0x67, 0x89, 0xBA
};

const uint32_t RECONNECT_MS = 15000;
const uint32_t COMMAND_MS = 8000;
const uint32_t SPEED_MS = 3000;

enum Phase {
  OFFLINE,
  IDENTIFY,
  ECHO_OFF,
  LINEFEED_OFF,
  SPACES_OFF,
  SPEED,
  READY
};

Phase phase = OFFLINE;

bool btStarted = false;
bool waiting = false;
bool hadConnection = false;

uint32_t attempt = 0;
uint32_t nextConnect = 0;
uint32_t commandAt = 0;
uint32_t lastSpeed = 0;

String buffer;
String command;

void event(
  const char *step,
  const char *status,
  const String &detail
) {
  Serial.printf(
    "[TRY %lu][%s][%s][%lu ms] %s\n",
    (unsigned long)attempt,
    step,
    status,
    (unsigned long)millis(),
    detail.c_str()
  );
}

void scheduleReconnect(const String &why) {
  event("SESSION", "FAIL", why);

  waiting = false;
  phase = OFFLINE;
  command = "";
  buffer = "";

  obd.disconnect();

  nextConnect = millis() + RECONNECT_MS;

  event(
    "RETRY",
    "INFO",
    "Next attempt in 15 seconds"
  );
}

void sendOBD(const char *s, Phase nextPhase) {
  if (!obd.connected()) {
    scheduleReconnect(
      "Bluetooth disconnected before command"
    );
    return;
  }

  while (obd.available()) {
    obd.read();
  }

  command = s;
  buffer = "";
  phase = nextPhase;
  waiting = true;
  commandAt = millis();

  event("COMMAND", "START", command);

  obd.print(command);
  obd.write('\r');
}

bool isError(String s) {
  s.toUpperCase();

  return s.indexOf("?") >= 0 ||
         s.indexOf("ERROR") >= 0 ||
         s.indexOf("NO DATA") >= 0 ||
         s.indexOf("UNABLE") >= 0 ||
         s.indexOf("STOPPED") >= 0 ||
         s.indexOf("BUS ERROR") >= 0;
}

bool speedFromResponse(String raw, int &speed) {
  raw.toUpperCase();

  raw.replace(" ", "");
  raw.replace("\r", "");
  raw.replace("\n", "");
  raw.replace("\t", "");

  // Find a positive response to PID 010D
  int pos = raw.indexOf("410D");

  if (pos < 0 || pos + 6 > (int)raw.length()) {
    return false;
  }

  String byteText = raw.substring(
    pos + 4,
    pos + 6
  );

  for (int i = 0; i < 2; i++) {
    if (!isxdigit((unsigned char)byteText[i])) {
      return false;
    }
  }

  speed = (int)strtol(
    byteText.c_str(),
    nullptr,
    16
  );

  return true;
}

void completed(String raw) {
  waiting = false;
  raw.trim();

  event(
    "COMMAND",
    "RX",
    command + " => " + raw
  );

  if (phase == IDENTIFY) {
    if (raw.length() == 0 || isError(raw)) {
      scheduleReconnect(
        "ATI returned empty/error response"
      );
      return;
    }

    event(
      "03 ATI",
      "PASS",
      "Adapter replied: " + raw
    );

    sendOBD("ATE0", ECHO_OFF);
    return;
  }

  if (
    phase == ECHO_OFF ||
    phase == LINEFEED_OFF ||
    phase == SPACES_OFF
  ) {
    String upper = raw;
    upper.toUpperCase();

    if (
      isError(raw) ||
      upper.indexOf("OK") < 0
    ) {
      scheduleReconnect(
        "ELM setup rejected " +
        command + ": " + raw
      );
      return;
    }

    event(
      "04 SETUP",
      "PASS",
      command + " => OK"
    );

    if (phase == ECHO_OFF) {
      sendOBD("ATL0", LINEFEED_OFF);
    } else if (phase == LINEFEED_OFF) {
      sendOBD("ATS0", SPACES_OFF);
    } else {
      phase = READY;
      lastSpeed = millis() - SPEED_MS;

      event(
        "04 SETUP",
        "PASS",
        "All setup commands completed"
      );
    }

    return;
  }

  if (phase == SPEED) {
    int kmh = -1;

    if (speedFromResponse(raw, kmh)) {
      event(
        "05 ECU SPEED",
        "PASS",
        String(kmh) + " km/h"
      );
    } else {
      event(
        "05 ECU SPEED",
        "FAIL",
        "No valid 410Dxx reply: " + raw
      );
    }

    lastSpeed = millis();
    phase = READY;
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println(
    "\nESP32 OBD2 Diagnostic v2.0"
  );

  Serial.println(
    "[01 BT INIT][START]"
  );

  btStarted = obd.begin(
    "ESP32-OBD2",
    true
  );

  event(
    "01 BT INIT",
    btStarted ? "PASS" : "FAIL",
    btStarted ?
      "Bluetooth Classic master ready" :
      "Bluetooth startup failed"
  );

  nextConnect = millis();
}

void loop() {
  if (!btStarted) {
    delay(1000);
    return;
  }

  if (!obd.connected()) {
    if (hadConnection) {
      hadConnection = false;

      scheduleReconnect(
        "Bluetooth link lost"
      );
    }

    if (
      (int32_t)(millis() - nextConnect) >= 0
    ) {
      attempt++;

      event(
        "02 BT CONNECT",
        "START",
        "Connecting to captured Classic MAC"
      );

      if (obd.connect(peer)) {
        hadConnection = true;

        event(
          "02 BT CONNECT",
          "PASS",
          "SPP connected"
        );

        delay(350);
        sendOBD("ATI", IDENTIFY);
      } else {
        event(
          "02 BT CONNECT",
          "FAIL",
          "SPP failed; check address, pairing and adapter availability"
        );

        obd.disconnect();

        nextConnect =
          millis() + RECONNECT_MS;
      }
    }

    delay(20);
    return;
  }

  while (obd.available()) {
    char c = (char)obd.read();

    if (c == '>') {
      String response = buffer;
      buffer = "";

      if (waiting) {
        completed(response);
      }
    } else if (buffer.length() < 512) {
      buffer += c;
    } else {
      scheduleReconnect(
        "ELM response exceeded 512 bytes"
      );
      return;
    }
  }

  if (
    waiting &&
    millis() - commandAt >= COMMAND_MS
  ) {
    event(
      "COMMAND",
      "TIMEOUT",
      command + " partial=" + buffer
    );

    scheduleReconnect(
      "ELM command timeout"
    );

    return;
  }

  if (
    phase == READY &&
    !waiting &&
    millis() - lastSpeed >= SPEED_MS
  ) {
    sendOBD("010D", SPEED);
  }

  delay(10);
}
