#include <Wire.h>
#include <string.h>

// ---- Config ----
#define SLAVE_SDA_PIN 4      // GP4, I2C0 par defaut sur le Pico 2 - adapte si besoin
#define SLAVE_SCL_PIN 5      // GP5
#define BMP580_I2C_ADDR 0x47 // 0x47 (SDO=VDDIO) ou 0x46 (SDO=GND) selon ce qu'attend ton master

// ---- Registres cles du BMP580 (identiques BMP581/585 sauf CHIP_ID) ----
#define REG_CHIP_ID    0x01
#define REG_REV_ID     0x02
#define REG_STATUS     0x28
#define REG_TEMP_XLSB  0x1D
#define REG_TEMP_LSB   0x1E
#define REG_TEMP_MSB   0x1F
#define REG_PRESS_XLSB 0x20
#define REG_PRESS_LSB  0x21
#define REG_PRESS_MSB  0x22

#define CHIP_ID_BMP580 0x50
#define REV_ID_VALUE   0x32

volatile uint8_t registers[256];
volatile uint8_t regPointer = 0;

float simTemperature = 22.5;      // en °C, modifie/fais varier pour ton scenario HIL
float simPressure    = 101325.0;  // en Pa

void updateSimulatedData() {
  // Encodage BMP580 standard : temp = raw/2^16 (°C), pression = raw/2^6 (Pa)
  // A revalider avec le datasheet officiel si ton driver est strict sur la precision
  int32_t rawTemp  = (int32_t)(simTemperature * 65536.0);
  int32_t rawPress = (int32_t)(simPressure * 64.0);

  registers[REG_TEMP_XLSB]  = rawTemp & 0xFF;
  registers[REG_TEMP_LSB]   = (rawTemp >> 8) & 0xFF;
  registers[REG_TEMP_MSB]   = (rawTemp >> 16) & 0xFF;

  registers[REG_PRESS_XLSB] = rawPress & 0xFF;
  registers[REG_PRESS_LSB]  = (rawPress >> 8) & 0xFF;
  registers[REG_PRESS_MSB]  = (rawPress >> 16) & 0xFF;

  registers[REG_STATUS] = 0x60; // bits drdy_temp/drdy_press a 1 - verifie l'exact mapping si ton driver check ca finement
}

void onI2CReceive(int numBytes) {
  if (numBytes <= 0) return;
  regPointer = Wire.read();  // 1er octet = adresse registre ciblee par le master
  numBytes--;
  while (numBytes > 0) {     // octets suivants = ecriture dans les registres (config, CMD...)
    registers[regPointer] = Wire.read();
    regPointer++;
    numBytes--;
  }
}

void onI2CRequest() {
  // On propose un bloc de plusieurs octets depuis le pointeur courant.
  // Le master s'arrete tout seul (NACK) une fois qu'il a recu ce qu'il voulait,
  // que ce soit 1 octet (ex: CHIP_ID) ou 6 (burst temp+pression).
  for (int i = 0; i < 8 && (regPointer + i) < 256; i++) {
    Wire.write(registers[regPointer + i]);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("--- Emulateur BMP580 (I2C slave) ---");

  memset((void*)registers, 0, sizeof(registers));
  registers[REG_CHIP_ID] = CHIP_ID_BMP580;
  registers[REG_REV_ID]  = REV_ID_VALUE;
  updateSimulatedData();

  Wire.setSDA(SLAVE_SDA_PIN);
  Wire.setSCL(SLAVE_SCL_PIN);
  Wire.begin(BMP580_I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  Serial.print("Actif sur adresse 0x");
  Serial.println(BMP580_I2C_ADDR, HEX);
}

void loop() {
  // Fais varier tes valeurs ici pour simuler ton scenario HIL
  // simPressure += 0.5; simTemperature += 0.01;
  updateSimulatedData();
  delay(50);
}