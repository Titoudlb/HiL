#include <Wire.h>
#include <string.h>

#define SLAVE_SDA_PIN 4
#define SLAVE_SCL_PIN 5
#define BMP580_I2C_ADDR 0x47

#define REG_CHIP_ID     0x01
#define REG_REV_ID      0x02
#define REG_INT_SOURCE  0x15
#define REG_TEMP_XLSB   0x1D
#define REG_TEMP_LSB    0x1E
#define REG_TEMP_MSB    0x1F
#define REG_PRESS_XLSB  0x20
#define REG_PRESS_LSB   0x21
#define REG_PRESS_MSB   0x22
#define REG_INT_STATUS  0x27
#define REG_STATUS      0x28
#define REG_DSP_CONFIG  0x30
#define REG_DSP_IIR     0x31
#define REG_OSR_CONFIG  0x36
#define REG_ODR_CONFIG  0x37
#define REG_OSR_EFF     0x38
#define REG_CMD         0x7E

#define CHIP_ID_BMP580  0x50
#define REV_ID_VALUE    0x32
#define STATUS_NVM_RDY_BIT  0x02
#define INT_STATUS_DRDY_BIT 0x01
#define INT_STATUS_POR_BIT  0x10

// Valeurs de reset REELLES d'apres le datasheet Bosch (Register Map, chap. 7)
// La plupart des registres sont a 0x00 au reset, mais pas tous :
#define ODR_CONFIG_RESET_VAL  0x70  // deep_dis=0, odr=defaut, pwr_mode=STANDBY(00)
#define DSP_CONFIG_RESET_VAL  0x03  // bits reserves 1:0 cables a 1 sur le vrai capteur

volatile uint8_t registers[256];
volatile uint8_t regPointer = 0;
volatile unsigned long lastI2CActivity = 0;

float simTemperature = 22.5;
float simPressure    = 101325.0;

void updateSimulatedData() {
  int32_t rawTemp  = (int32_t)(simTemperature * 65536.0);
  int32_t rawPress = (int32_t)(simPressure * 64.0);
  registers[REG_TEMP_XLSB]  = rawTemp & 0xFF;
  registers[REG_TEMP_LSB]   = (rawTemp >> 8) & 0xFF;
  registers[REG_TEMP_MSB]   = (rawTemp >> 16) & 0xFF;
  registers[REG_PRESS_XLSB] = rawPress & 0xFF;
  registers[REG_PRESS_LSB]  = (rawPress >> 8) & 0xFF;
  registers[REG_PRESS_MSB]  = (rawPress >> 16) & 0xFF;
  registers[REG_STATUS]     = STATUS_NVM_RDY_BIT;
  registers[REG_INT_STATUS] = INT_STATUS_DRDY_BIT | INT_STATUS_POR_BIT;
}

// Reinitialise TOUS les registres a leurs vraies valeurs de reset silicium.
// Appelee au boot (setup) ET a chaque soft-reset (ecriture de 0xB6 dans CMD).
void resetToDefaults() {
  memset((void*)registers, 0, sizeof(registers));
  registers[REG_CHIP_ID]    = CHIP_ID_BMP580;
  registers[REG_REV_ID]     = REV_ID_VALUE;
  registers[REG_ODR_CONFIG] = ODR_CONFIG_RESET_VAL;
  registers[REG_DSP_CONFIG] = DSP_CONFIG_RESET_VAL;
  updateSimulatedData();
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

    if (regPointer == REG_CMD && val == 0xB6) {
      Serial.println("[RESET] Soft-reset (0xB6) recu -> reinit registres (valeurs datasheet)");
      resetToDefaults();
    }

    regPointer++;
    numBytes--;
  }
  // Maintient OSR_EFF synchronise avec OSR_CONFIG + force le bit "config valide"
  registers[REG_OSR_EFF] = registers[REG_OSR_CONFIG] | 0x80;
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
  Wire.begin(BMP580_I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  lastI2CActivity = millis();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("--- Emulateur BMP580 (mode debug) ---");

  resetToDefaults();

  startI2CSlave();
  Serial.println("Pret, en attente du master...");
}

void loop() {
  updateSimulatedData();
  if (millis() - lastI2CActivity > 3000) {
    Wire.end();
    delay(2);
    startI2CSlave();
  }
  delay(50);
}