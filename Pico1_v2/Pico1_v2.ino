#include <Wire.h>
#include <string.h>

// ---- Registres BMP580 (communs aux deux capteurs emules) ----
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
#define STATUS_NVM_RDY_BIT   0x02
#define INT_STATUS_DRDY_BIT  0x01
#define INT_STATUS_POR_BIT   0x10
#define ODR_CONFIG_RESET_VAL 0x70
#define DSP_CONFIG_RESET_VAL 0x03

#define PICO_ID   1        // 1 sur le premier Pico, 2 sur le deuxieme
#define LED_PIN   LED_BUILTIN
#define BLINK_MS  150      // duree ON et OFF de chaque flash
#define PAUSE_MS  1000     // pause avant de recommencer le motif



class Bmp580Emulator {
public:
  void begin(TwoWire &bus, uint8_t sda, uint8_t scl, uint8_t addr,
             const char* label, void (*onRecv)(int), void (*onReq)()) {
    wire    = &bus;
    sdaPin  = sda;
    sclPin  = scl;
    address = addr;
    name    = label;
    resetToDefaults();
    startSlave(onRecv, onReq);
  }

  void poll() {
    updateSimulatedData();
    if (millis() - lastActivity > 3000) {
      wire->end();
      delay(2);
      startSlave(savedRecv, savedReq);
    }
  }

  void handleReceive(int numBytes) {
    lastActivity = millis();
    if (numBytes <= 0) return;
    regPointer = wire->read();
    numBytes--;

    Serial.print("["); Serial.print(name); Serial.print("] [WRITE] pointeur -> 0x");
    Serial.println(regPointer, HEX);

    while (numBytes > 0) {
      uint8_t val = wire->read();
      Serial.print("["); Serial.print(name); Serial.print("] [WRITE] reg 0x");
      Serial.print(regPointer, HEX);
      Serial.print(" = 0x");
      Serial.println(val, HEX);
      registers[regPointer] = val;

      if (regPointer == REG_CMD && val == 0xB6) {
        Serial.print("["); Serial.print(name); Serial.println("] [RESET] soft-reset recu -> reinit registres");
        resetToDefaults();
      }

      regPointer++;
      numBytes--;
    }
    registers[REG_OSR_EFF] = registers[REG_OSR_CONFIG] | 0x80;
  }

  void handleRequest() {
    lastActivity = millis();
    Serial.print("["); Serial.print(name); Serial.print("] [READ] depuis reg 0x");
    Serial.print(regPointer, HEX);
    Serial.print(" -> 0x");
    Serial.println(registers[regPointer], HEX);

    for (int i = 0; i < 8 && (regPointer + i) < 256; i++) {
      wire->write(registers[regPointer + i]);
    }
  }

private:
  TwoWire *wire;
  uint8_t sdaPin, sclPin, address;
  const char* name;
  void (*savedRecv)(int);
  void (*savedReq)();

  volatile uint8_t registers[256];
  volatile uint8_t regPointer = 0;
  volatile unsigned long lastActivity = 0;
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

  void resetToDefaults() {
    memset((void*)registers, 0, sizeof(registers));
    registers[REG_CHIP_ID]    = CHIP_ID_BMP580;
    registers[REG_REV_ID]     = REV_ID_VALUE;
    registers[REG_ODR_CONFIG] = ODR_CONFIG_RESET_VAL;
    registers[REG_DSP_CONFIG] = DSP_CONFIG_RESET_VAL;
    updateSimulatedData();
  }

  void startSlave(void (*onRecv)(int), void (*onReq)()) {
    savedRecv = onRecv;
    savedReq  = onReq;
    wire->setSDA(sdaPin);
    wire->setSCL(sclPin);
    wire->begin(address);
    wire->onReceive(savedRecv);
    wire->onRequest(savedReq);
    lastActivity = millis();
  }
};

// ---- Deux instances : une par adresse I2C ----
Bmp580Emulator bmpA; // 0x47, sur Wire  (I2C0) -> GPIO4/5
Bmp580Emulator bmpB; // 0x46, sur Wire1 (I2C1) -> GPIO6/7

// Wire.onReceive/onRequest exigent des fonctions C classiques (pas des methodes),
// donc on redirige chacune vers l'instance concernee.
void onReceiveA(int n) { bmpA.handleReceive(n); }
void onRequestA()      { bmpA.handleRequest(); }
void onReceiveB(int n) { bmpB.handleReceive(n); }
void onRequestB()      { bmpB.handleRequest(); }

void updateIdentityBlink() {
  static unsigned long lastChange = 0;
  static int flashCount = 0;
  static bool ledState = false;

  unsigned long now = millis();
  unsigned long interval = ledState ? BLINK_MS
                          : (flashCount >= PICO_ID ? PAUSE_MS : BLINK_MS);

  if (now - lastChange < interval) return;
  lastChange = now;

  if (flashCount >= PICO_ID) flashCount = 0; // fin de pause -> nouveau cycle

  ledState = !ledState;
  digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  if (!ledState) flashCount++; // on compte le flash a chaque retour a LOW
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("--- Emulateur 2x BMP580 (mode debug) ---");

  bmpA.begin(Wire,  4, 5, 0x47, "BMP47", onReceiveA, onRequestA);
  bmpB.begin(Wire1, 6, 7, 0x46, "BMP46", onReceiveB, onRequestB);

  Serial.println("Pret, en attente du master...");
}

void loop() {
  bmpA.poll();
  bmpB.poll();
  updateIdentityBlink();
  delay(50);
}