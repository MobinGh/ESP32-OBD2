
#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

const char* obdName = "OBDII";
const char* pins[] = {"1234", "0000"};

bool connected = false;

bool tryConnect(const char* pin) {
  Serial.println();
  Serial.print("Trying PIN: ");
  Serial.println(pin);

  SerialBT.disconnect();
  SerialBT.end();
  delay(1000);

  if (!SerialBT.begin("ServiceMonitor", true)) {
    Serial.println("Bluetooth init failed");
    return false;
  }

  SerialBT.setPin(pin, 4);
  delay(500);

  // Try a fresh connection with this PIN
  bool ok = SerialBT.connect(obdName);

  if (ok) {
    Serial.println("Bluetooth connected!");
    Serial.print("Successful PIN attempt: ");
    Serial.println(pin);

    delay(1000);
    SerialBT.print("ATI\r");
  } else {
    Serial.println("Connection failed");
  }

  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("ESP32 OBDII Bluetooth Test");

  for (int i = 0; i < 2; i++) {
    if (tryConnect(pins[i])) {
      connected = true;
      break;
    }
    delay(1500);
  }

  if (!connected) {
    Serial.println("Both connection attempts failed!");
  }
}

void loop() {
  if (connected) {
    while (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
  }
  delay(10);
}
