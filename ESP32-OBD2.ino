
#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("Starting Bluetooth...");

  if (!SerialBT.begin("ServiceMonitor", true)) {
    Serial.println("Bluetooth initialization failed");
    return;
  }

  Serial.println("Connecting to OBDII...");

  SerialBT.setPin("1234", 4);

  if (SerialBT.connect("OBDII")) {
    Serial.println("Bluetooth connected!");

    delay(1000);
    SerialBT.print("ATI\r");
  } else {
    Serial.println("Connection failed!");
  }
}

void loop() {
  while (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
}
