
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <esp_spp_api.h>
#include <esp_gap_bt_api.h>
#include <ctype.h>

#if !defined(CONFIG_BT_ENABLED) || \
    !defined(CONFIG_BLUEDROID_ENABLED) || \
    !defined(CONFIG_BT_SPP_ENABLED)
#error "Original ESP32 with Classic Bluetooth required"
#endif

BluetoothSerial obd;

// Confirmed Classic Bluetooth address
uint8_t targetMac[6] = {
  0x01, 0x23, 0x45, 0x67, 0x89, 0xBA
};

// RFCOMM channel extracted from SDP capture
const int RFCOMM_CHANNEL = 2;

const uint32_t RETRY_MS = 20000;
const uint32_t COMMAND_TIMEOUT_MS = 8000;
const uint32_t SPEED_INTERVAL_MS = 3000;

enum Stage {
  DISCONNECTED,
  IDENTIFY,
  ECHO_OFF,
  LINEFEED_OFF,
  SPACES_OFF,
  READY,
  SPEED
};

Stage stage = DISCONNECTED;

bool bluetoothReady = false;
bool linkWasOpen = false;
bool waiting = false;

uint32_t attemptNumber = 0;
uint32_t nextAttemptAt = 0;
uint32_t commandSentAt = 0;
uint32_t lastSpeedAt = 0;

String rxBuffer;
String activeCommand;

void logMessage(
  const char *step,
  const char *status,
  const String &message
) {
  Serial.printf(
    "[TRY %lu][%s][%s][%lu ms] %s\n",
    (unsigned long)attemptNumber,
    step,
    status,
    (unsigned long)millis(),
    message.c_str()
  );
}

void sppCallback(
  esp_spp_cb_event_t event,
  esp_spp_cb_param_t *param
) {
  switch (event) {

    case ESP_SPP_INIT_EVT:
      Serial.printf(
        "[SPP][INIT] status=%d\n",
        (int)param->init.status
      );
      break;

    case ESP_SPP_DISCOVERY_COMP_EVT:
      Serial.printf(
        "[SPP][DISCOVERY] status=%d channels=%d\n",
        (int)param->disc_comp.status,
        (int)param->disc_comp.scn_num
      );
      break;

    case ESP_SPP_CL_INIT_EVT:
      Serial.printf(
        "[SPP][CLIENT_INIT] status=%d\n",
        (int)param->cl_init.status
      );
      break;

    case ESP_SPP_OPEN_EVT:
      Serial.printf(
        "[SPP][OPEN] status=%d handle=%lu\n",
        (int)param->open.status,
        (unsigned long)param->open.handle
      );
      break;

    case ESP_SPP_CLOSE_EVT:
      Serial.printf(
        "[SPP][CLOSE] status=%d handle=%lu\n",
        (int)param->close.status,
        (unsigned long)param->close.handle
      );
      break;

    default:
      break;
  }
}

void authenticationCallback(bool success) {
  Serial.printf(
    "[BT AUTH][%s]\n",
    success ? "PASS" : "FAIL"
  );
}

void resetSession() {
  waiting = false;
  linkWasOpen = false;
  stage = DISCONNECTED;
  rxBuffer = "";
  activeCommand = "";
}

void disconnectAndRetry(const String &reason) {
  logMessage("SESSION", "FAIL", reason);

  resetSession();
  obd.disconnect();

  nextAttemptAt = millis() + RETRY_MS;

  logMessage(
    "RETRY",
    "INFO",
    "Retry scheduled in 20 seconds"
  );
}

bool hasError(String s) {
  s.toUpperCase();

  return s.indexOf("ERROR") >= 0 ||
         s.indexOf("NO DATA") >= 0 ||
         s.indexOf("UNABLE") >= 0 ||
         s.indexOf("STOPPED") >= 0 ||
         s.indexOf("BUS ERROR") >= 0 ||
         s.indexOf("?") >= 0;
}

void sendCommand(const char *cmd, Stage nextStage) {
  if (!obd.connected()) {
    disconnectAndRetry(
      "Bluetooth link lost before command"
    );
    return;
  }

  rxBuffer = "";
  activeCommand = cmd;
  stage = nextStage;
  waiting = true;
  commandSentAt = millis();

  logMessage("ELM TX", "START", activeCommand);

  size_t written = obd.print(cmd);
  written += obd.write('\r');

  if (written != strlen(cmd) + 1) {
    disconnectAndRetry(
      "Incomplete Bluetooth write"
    );
  }
}

bool parseSpeed(String data, int &kmh) {
  data.toUpperCase();
  data.replace(" ", "");
  data.replace("\r", "");
  data.replace("\n", "");
  data.replace("\t", "");

  int pos = data.indexOf("410D");

  if (pos < 0 || pos + 6 > (int)data.length()) {
    return false;
  }

  char a = data[pos + 4];
  char b = data[pos + 5];

  if (!isxdigit((unsigned char)a) ||
      !isxdigit((unsigned char)b)) {
    return false;
  }

  kmh = (int)strtol(
    data.substring(pos + 4, pos + 6).c_str(),
    nullptr,
    16
  );

  return true;
}

void handleResponse(String data) {
  waiting = false;
  data.trim();

  logMessage(
    "ELM RX",
    "DATA",
    activeCommand + " => " + data
  );

  if (stage == IDENTIFY) {
    String identity = data;
    identity.toUpperCase();
    identity.replace("ATI", "");
    identity.trim();

    bool valid =
      identity.indexOf("ELM327") >= 0 ||
      identity.indexOf("OBDII") >= 0 ||
      identity.indexOf("OBD2") >= 0;

    if (!valid || hasError(data)) {
      disconnectAndRetry(
        "Invalid ATI response: " + data
      );
      return;
    }

    logMessage("03 IDENTIFY", "PASS", identity);
    sendCommand("ATE0", ECHO_OFF);
    return;
  }

  if (stage == ECHO_OFF ||
      stage == LINEFEED_OFF ||
      stage == SPACES_OFF) {

    String upper = data;
    upper.toUpperCase();

    if (upper.indexOf("OK") < 0 ||
        hasError(data)) {
      disconnectAndRetry(
        "Setup rejected: " +
        activeCommand + " => " + data
      );
      return;
    }

    logMessage("04 SETUP", "PASS", activeCommand);

    if (stage == ECHO_OFF) {
      sendCommand("ATL0", LINEFEED_OFF);
    }
    else if (stage == LINEFEED_OFF) {
      sendCommand("ATS0", SPACES_OFF);
    }
    else {
      stage = READY;
      lastSpeedAt = millis() - SPEED_INTERVAL_MS;

      logMessage(
        "04 SETUP",
        "PASS",
        "All ELM commands accepted"
      );
    }
    return;
  }

  if (stage == SPEED) {
    int kmh = -1;

    if (parseSpeed(data, kmh)) {
      logMessage(
        "05 ECU SPEED",
        "PASS",
        String(kmh) + " km/h"
      );
    }
    else {
      logMessage(
        "05 ECU SPEED",
        "FAIL",
        "Invalid speed response: " + data
      );
    }

    lastSpeedAt = millis();
    stage = READY;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("================================");
  Serial.println(" ESP32 OBD2 DIRECT RFCOMM v4.1");
  Serial.println("================================");

  Serial.println(
    "[TARGET] 01:23:45:67:89:BA"
  );

  Serial.println(
    "[RFCOMM] Direct channel 2"
  );

  obd.register_callback(sppCallback);
  obd.onAuthComplete(authenticationCallback);

  bluetoothReady = obd.begin(
    "ESP32-OBD2",
    true
  );

  logMessage(
    "01 BT INIT",
    bluetoothReady ? "PASS" : "FAIL",
    bluetoothReady ?
      "Bluetooth Classic initialized" :
      "Bluetooth initialization failed"
  );

  nextAttemptAt = millis();
}

void loop() {
  if (!bluetoothReady) {
    delay(1000);
    return;
  }

  if (!obd.connected()) {
    if (linkWasOpen) {
      disconnectAndRetry(
        "Bluetooth connection lost"
      );
      return;
    }

    if ((int32_t)(millis() - nextAttemptAt) >= 0) {
      attemptNumber++;

      logMessage(
        "02 BT CONNECT",
        "START",
        "Direct RFCOMM channel 2"
      );

      uint32_t startTime = millis();

      bool connected = obd.connect(
        targetMac,
        RFCOMM_CHANNEL
      );

      uint32_t elapsed = millis() - startTime;

      if (connected && obd.connected()) {
        linkWasOpen = true;

        logMessage(
          "02 BT CONNECT",
          "PASS",
          "Connected in " +
          String(elapsed) + " ms"
        );

        delay(300);
        sendCommand("ATI", IDENTIFY);
      }
      else {
        logMessage(
          "02 BT CONNECT",
          "FAIL",
          "Direct channel connection failed after " +
          String(elapsed) + " ms"
        );

        obd.disconnect();
        nextAttemptAt = millis() + RETRY_MS;
      }
    }

    delay(20);
    return;
  }

  while (obd.available()) {
    int value = obd.read();

    if (value < 0) break;

    char c = (char)value;

    if (c == '>') {
      String complete = rxBuffer;
      rxBuffer = "";

      if (waiting) {
        handleResponse(complete);
        break;
      }
    }
    else if (rxBuffer.length() < 512) {
      rxBuffer += c;
    }
    else {
      disconnectAndRetry(
        "ELM response buffer overflow"
      );
      return;
    }
  }

  if (!obd.connected()) return;

  if (waiting &&
      millis() - commandSentAt >= COMMAND_TIMEOUT_MS) {

    logMessage(
      "ELM",
      "TIMEOUT",
      activeCommand + " partial=" + rxBuffer
    );

    disconnectAndRetry(
      "ELM command timeout"
    );
    return;
  }

  if (stage == READY &&
      !waiting &&
      millis() - lastSpeedAt >= SPEED_INTERVAL_MS) {

    sendCommand("010D", SPEED);
  }

  delay(10);
}
