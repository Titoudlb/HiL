#include <Wire.h>
#include <string.h>

#define SLAVE_SDA_PIN 4
#define SLAVE_SCL_PIN 5
#define KX134_I2C_ADDR 0x1E

// Adresses de registres (KX134-1211, Technical Reference Manual)
#define REG_XOUT_L    0x08
#define REG_XOUT_H    0x09
#define REG_YOUT_L    0x0A
#define REG_YOUT_H    0x0B
#define REG_ZOUT_L    0x0C
#define REG_ZOUT_H    0x0D
#define REG_WHO_AM_I  0x13
#define REG_CNTL1     0x1B
#define REG_CNTL2     0x1C
#define REG_INS2       0x17
#define INS2_DRDY_BIT  0x10

#define WHO_AM_I_KX134   0x46   // confirme (lib SparkFun) : KX132=0x3D, KX134=0x46
#define CNTL2_RESET_VAL  0x3F   // valeur de reset datasheet (bit7 SRST=0 au repos)
#define CNTL2_SRST_BIT   0x80

volatile uint8_t registers[256];
volatile uint8_t regPointer = 0;
volatile unsigned long lastI2CActivity = 0;

void resetToDefaults() {
  memset((void*)registers, 0, sizeof(registers));
  registers[REG_WHO_AM_I] = WHO_AM_I_KX134;
  registers[REG_CNTL2]    = CNTL2_RESET_VAL;
  registers[REG_INS2]     = INS2_DRDY_BIT;   // toujours "donnee prete"
}

void onI2CReceive(int numBytes) {
  lastI2CActivity = millis();
  if (numBytes <= 0) return;
  regPointer = Wire.read();
  numBytes--;

  Serial.print("[WRITE] pointeur mis a jour -> 0x");
  Serial.println(regPointer, HEX);

  while (numBytes > 0) {
    uint8_t val = Wire.read();
    Serial.print("[WRITE] reg 0x");
    Serial.print(regPointer, HEX);
    Serial.print(" = 0x");
    Serial.println(val, HEX);
    registers[regPointer] = val;

    // SRST (bit7 CNTL2) : le vrai capteur garde ce bit a 1 pendant le reboot.
    // On simule un reboot instantane -> retour a l'etat par defaut,
    // SRST redescend a 0 automatiquement (0x3F a bit7=0).
    if (regPointer == REG_CNTL2 && (val & CNTL2_SRST_BIT)) {
      Serial.println("[RESET] SRST recu -> reinit registres");
      resetToDefaults();
    }

    regPointer++;
    numBytes--;
  }
}

void onI2CRequest() {
  lastI2CActivity = millis();
  Serial.print("[READ]  depuis reg 0x");
  Serial.print(regPointer, HEX);
  Serial.print(" -> 0x");
  Serial.println(registers[regPointer], HEX);

  for (int i = 0; i < 8 && (regPointer + i) < 256; i++) {
    Wire.write(registers[regPointer + i]);
  }
}

void startI2CSlave() {
  Wire.setSDA(SLAVE_SDA_PIN);
  Wire.setSCL(SLAVE_SCL_PIN);
  Wire.begin(KX134_I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  lastI2CActivity = millis();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("--- Emulateur KX134 (mode debug, data a 0) ---");

  resetToDefaults();
  startI2CSlave();

  Serial.println("Pret, en attente du master...");
}

void loop() {
  if (millis() - lastI2CActivity > 3000) {
    Wire.end();
    delay(2);
    startI2CSlave();
  }
  delay(50);
}