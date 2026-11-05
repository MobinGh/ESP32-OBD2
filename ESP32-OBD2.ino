
#include <Arduino.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || \
    !defined(CONFIG_BLUEDROID_ENABLED) || \
    !defined(CONFIG_BT_SPP_ENABLED)
#error "ESP32 Bluetooth Classic is required"
#endif

BluetoothSerial SerialBT;

const uint32_t SCAN_DURATION_MS = 12000;
const uint32_t PAUSE_MS = 8000;

uint32_t scanNumber = 0;
bool bluetoothReady = false;

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("====================================");
  Serial.println(" ESP32 BLUETOOTH CLASSIC SCANNER");
  Serial.println(" Version 4.0");
  Serial.println("====================================");

  Serial.println("[01 INIT][START]");

  bluetoothReady = SerialBT.begin(
    "ESP32-OBD-SCANNER",
    true
  );

  if (!bluetoothReady) {
    Serial.println(
      "[01 INIT][FAIL] Bluetooth initialization failed"
    );
    return;
  }

  Serial.println(
    "[01 INIT][PASS] Bluetooth Classic ready"
  );
}

void loop() {
  if (!bluetoothReady) {
    delay(1000);
    return;
  }

  scanNumber++;

  Serial.println();
  Serial.println("====================================");
  Serial.printf(
    "[SCAN %lu][START]\n",
    (unsigned long)scanNumber
  );
  Serial.println(
    "[INFO] Searching for Classic Bluetooth devices..."
  );

  uint32_t started = millis();

  BTScanResults* results = SerialBT.discover(
    SCAN_DURATION_MS
  );

  uint32_t elapsed = millis() - started;

  if (results == nullptr) {
    Serial.println(
      "[SCAN][FAIL] Discovery returned no results object"
    );
  } else {
    Serial.printf(
      "[SCAN][COMPLETE] Duration: %lu ms\n",
      (unsigned long)elapsed
    );

    Serial.println(
      "[DEVICES] Found devices:"
    );

    results->dump(&Serial);
  }

  Serial.printf(
    "[SCAN %lu][END]\n",
    (unsigned long)scanNumber
  );

  Serial.println(
    "[INFO] Next scan in 8 seconds"
  );

  delay(PAUSE_MS);
}
