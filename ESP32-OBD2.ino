
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <esp_spp_api.h>
#include <ctype.h>

#if !defined(CONFIG_BT_ENABLED) || \
    !defined(CONFIG_BLUEDROID_ENABLED) || \
    !defined(CONFIG_BT_SPP_ENABLED)
#error "ESP32 Bluetooth Classic SPP required"
#endif

BluetoothSerial obd;

// Classic Bluetooth address from HCI analysis
uint8_t targetMac[6] = {
  0x01, 0x23, 0x45, 0x67, 0x89, 0xBA
};

const uint32_t RETRY_INTERVAL = 20000;
const uint32_t COMMAND_TIMEOUT = 8000;
const uint32_t SPEED_INTERVAL = 3000;

enum Stage {
  ST_DISCONNECTED,
  ST_IDENTIFY,
  ST_ECHO_OFF,
  ST_LINEFEED_OFF,
  ST_SPACES_OFF,
  ST_READY,
  ST_SPEED
};

Stage stage = ST_DISCONNECTED;

bool bluetoothInitialized = false;
bool linkEstablished = false;
bool awaitingResponse = false;

uint32_t attemptNumber = 0;
uint32_t nextAttemptTime = 0;
uint32_t commandStartTime = 0;
uint32_t lastSpeedTime = 0;

String receivedData;
String currentCommand;

void logEvent(
  const char* step,
  const char* status,
  const String& detail
) {
  Serial.printf(
    "[TRY %lu][%s][%s][%lu ms] %s\n",
    (unsigned long)attemptNumber,
    step,
    status,
    (unsigned long)millis(),
    detail.c_str()
  );
}

// Low-level SPP diagnostic events
void sppCallback(
  esp_spp_cb_event_t event,
  esp_spp_cb_param_t* param
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

void clearCommandState() {
  awaitingResponse = false;
  receivedData = "";
  currentCommand = "";
  stage = ST_DISCONNECTED;
}

void disconnectAndRetry(const String& reason) {
  logEvent("SESSION", "FAIL", reason);

  clearCommandState();
  linkEstablished = false;

  obd.disconnect();

  nextAttemptTime = millis() + RETRY_INTERVAL;

  logEvent(
    "RETRY",
    "INFO",
    "Next attempt scheduled in 20 seconds"
  );
}

bool responseHasError(String data) {
  data.toUpperCase();

  return data.indexOf("ERROR") >= 0 ||
         data.indexOf("NO DATA") >= 0 ||
         data.indexOf("UNABLE") >= 0 ||
         data.indexOf("STOPPED") >= 0 ||
         data.indexOf("BUS ERROR") >= 0 ||
         data.indexOf("?") >= 0;
}

void sendCommand(const char* command, Stage nextStage) {
  if (!obd.connected()) {
    disconnectAndRetry(
      "Bluetooth disconnected before command"
    );
    return;
  }

  receivedData = "";
  currentCommand = command;
  stage = nextStage;

  awaitingResponse = true;
  commandStartTime = millis();

  logEvent("ELM TX", "START", currentCommand);

  size_t written = obd.print(command);
  written += obd.write('\r');

  if (written != strlen(command) + 1) {
    disconnectAndRetry("Bluetooth write incomplete");
  }
}

bool decodeSpeed(String data, int& speed) {
  data.toUpperCase();

  data.replace(" ", "");
  data.replace("\r", "");
  data.replace("\n", "");
  data.replace("\t", "");

  int position = data.indexOf("410D");

  if (position < 0 ||
      position + 6 > (int)data.length()) {
    return false;
  }

  char first = data[position + 4];
  char second = data[position + 5];

  if (!isxdigit((unsigned char)first) ||
      !isxdigit((unsigned char)second)) {
    return false;
  }

  String value = data.substring(
    position + 4,
    position + 6
  );

  speed = (int)strtol(
    value.c_str(),
    nullptr,
    16
  );

  return true;
}

void handleResponse(String data) {
  awaitingResponse = false;
  data.trim();

  logEvent(
    "ELM RX",
    "DATA",
    currentCommand + " => " + data
  );

  if (stage == ST_IDENTIFY) {
    String upper = data;
    upper.toUpperCase();

    // Remove echoed command before validating identity
    upper.replace("ATI", "");
    upper.trim();

    bool identified =
      upper.indexOf("ELM327") >= 0 ||
      upper.indexOf("OBDII") >= 0 ||
      upper.indexOf("OBD2") >= 0;

    if (!identified || responseHasError(data)) {
      disconnectAndRetry(
        "ATI did not return a valid adapter identity"
      );
      return;
    }

    logEvent("03 IDENTIFY", "PASS", upper);
    sendCommand("ATE0", ST_ECHO_OFF);
    return;
  }

  if (stage == ST_ECHO_OFF ||
      stage == ST_LINEFEED_OFF ||
      stage == ST_SPACES_OFF) {

    String upper = data;
    upper.toUpperCase();

    bool accepted =
      upper.indexOf("OK") >= 0 &&
      !responseHasError(upper);

    if (!accepted) {
      disconnectAndRetry(
        "Setup command rejected: " +
        currentCommand + " => " + data
      );
      return;
    }

    logEvent(
      "04 SETUP",
      "PASS",
      currentCommand
    );

    if (stage == ST_ECHO_OFF) {
      sendCommand("ATL0", ST_LINEFEED_OFF);
    }
    else if (stage == ST_LINEFEED_OFF) {
      sendCommand("ATS0", ST_SPACES_OFF);
    }
    else {
      stage = ST_READY;
      lastSpeedTime = millis() - SPEED_INTERVAL;

      logEvent(
        "04 SETUP",
        "PASS",
        "All adapter settings accepted"
      );
    }

    return;
  }

  if (stage == ST_SPEED) {
    int speed = -1;

    if (decodeSpeed(data, speed)) {
      logEvent(
        "05 ECU SPEED",
        "PASS",
        String(speed) + " km/h"
      );
    }
    else {
      logEvent(
        "05 ECU SPEED",
        "FAIL",
        "Invalid or missing 410D response: " + data
      );
    }

    lastSpeedTime = millis();
    stage = ST_READY;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==================================");
  Serial.println("ESP32 OBD2 SPP DIAGNOSTIC v3.1");
  Serial.println("==================================");

  Serial.println(
    "[TARGET] 01:23:45:67:89:BA"
  );

  Serial.println(
    "[INFO] Bluetooth Classic / SPP"
  );

  Serial.println(
    "[INFO] Automatic PIN retries disabled"
  );

  logEvent(
    "01 BT INIT",
    "START",
    "Initializing Bluetooth"
  );

  obd.register_callback(sppCallback);

  bluetoothInitialized = obd.begin(
    "ESP32-OBD2",
    true
  );

  logEvent(
    "01 BT INIT",
    bluetoothInitialized ? "PASS" : "FAIL",
    bluetoothInitialized ?
      "Bluetooth Classic initialized" :
      "Bluetooth initialization failed"
  );

  nextAttemptTime = millis();
}

void loop() {
  if (!bluetoothInitialized) {
    delay(1000);
    return;
  }

  if (!obd.connected()) {

    if (linkEstablished) {
      disconnectAndRetry(
        "Bluetooth connection lost"
      );
      return;
    }

    if ((int32_t)(millis() - nextAttemptTime) >= 0) {
      attemptNumber++;

      logEvent(
        "02 BT CONNECT",
        "START",
        "Connecting to target via SPP"
      );

      uint32_t startTime = millis();

      bool connected = obd.connect(targetMac);

      uint32_t elapsed = millis() - startTime;

      if (connected && obd.connected()) {
        linkEstablished = true;

        logEvent(
          "02 BT CONNECT",
          "PASS",
          "SPP connected in " +
          String(elapsed) + " ms"
        );

        delay(300);

        sendCommand("ATI", ST_IDENTIFY);
      }
      else {
        logEvent(
          "02 BT CONNECT",
          "FAIL",
          "Connection failed after " +
          String(elapsed) +
          " ms. Check SPP event codes."
        );

        obd.disconnect();

        nextAttemptTime =
          millis() + RETRY_INTERVAL;
      }
    }

    delay(20);
    return;
  }

  while (obd.available()) {
    int value = obd.read();

    if (value < 0) break;

    char character = (char)value;

    if (character == '>') {
      String completeResponse = receivedData;
      receivedData = "";

      if (awaitingResponse) {
        handleResponse(completeResponse);

        // A new command may have been sent.
        // Process its response in the next loop.
        break;
      }
    }
    else {
      if (receivedData.length() < 512) {
        receivedData += character;
      }
      else {
        disconnectAndRetry(
          "ELM response exceeded buffer limit"
        );
        return;
      }
    }
  }

  if (!obd.connected()) {
    return;
  }

  if (awaitingResponse &&
      millis() - commandStartTime >= COMMAND_TIMEOUT) {

    logEvent(
      "ELM",
      "TIMEOUT",
      currentCommand +
      " partial=" + receivedData
    );

    disconnectAndRetry(
      "ELM command response timeout"
    );

    return;
  }

  if (stage == ST_READY &&
      !awaitingResponse &&
      millis() - lastSpeedTime >= SPEED_INTERVAL) {

    sendCommand("010D", ST_SPEED);
  }

  delay(10);
}
