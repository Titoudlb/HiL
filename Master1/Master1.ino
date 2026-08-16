#include <Wire.h>

#define SDA_PIN 6
#define SCL_PIN 7

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- I2C Scanner ---");

  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
}

void loop() {
  byte error, address;
  int devicesFound = 0;

  Serial.println("Scan en cours...");

  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0) {
      Serial.print("Device trouve a l'adresse 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devicesFound++;
    }
  }

  if (devicesFound == 0) {
    Serial.println("Aucun device I2C trouve.");
  } else {
    Serial.print(devicesFound);
    Serial.println(" device(s) trouve(s).");
  }

  Serial.println("---");
  delay(3000);
}