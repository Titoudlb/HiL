#include <Wire.h>
#include <string.h>
#include <math.h>

class Bmp580Emulator; // forward declaration - necessaire a cause de l'auto-generation
                       // de prototypes par l'IDE Arduino

// ================== Registres BMP580 ==================
#define REG_CHIP_ID     0x01
#define REG_REV_ID      0x02
#define REG_TEMP_XLSB   0x1D
#define REG_TEMP_LSB    0x1E
#define REG_TEMP_MSB    0x1F
#define REG_PRESS_XLSB  0x20
#define REG_PRESS_LSB   0x21
#define REG_PRESS_MSB   0x22
#define REG_INT_STATUS  0x27
#define REG_STATUS      0x28
#define REG_DSP_CONFIG  0x30
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

// ================== Registres KX134 ==================
#define REG_XOUT_L    0x08
#define REG_XOUT_H    0x09
#define REG_YOUT_L    0x0A
#define REG_YOUT_H    0x0B
#define REG_ZOUT_L    0x0C
#define REG_ZOUT_H    0x0D
#define REG_INS2      0x17
#define REG_WHO_AM_I  0x13
#define REG_CNTL1     0x1B
#define REG_CNTL2     0x1C

#define WHO_AM_I_KX134   0x46
#define CNTL2_RESET_VAL  0x3F
#define CNTL2_SRST_BIT   0x80
#define INS2_DRDY_BIT    0x10

// ================== Identite (clignotement LED) ==================
#define PICO_ID   2        // ce Pico = KX134 + BMP3
#define LED_PIN   LED_BUILTIN
#define BLINK_MS  150
#define PAUSE_MS  1000

// ================== Profil de vol partage ==================
struct FlightProfile {
  float boostAccel = 80.0;
  float boostT = 3.0;
  float drogueRate = 20.0;
  float restT = 0;
  unsigned long t0 = 0;
  bool running = false;

  float apogeeTime() const {
    float vBoost = boostAccel * boostT;
    return boostT + vBoost / 9.81;
  }

  float apogeeAltitude() const {
    float vBoost   = boostAccel * boostT;
    float altBoost = 0.5 * boostAccel * boostT * boostT;
    return altBoost + (vBoost * vBoost) / (2 * 9.81);
  }

  float altitude(float t) const {
    const float g = 9.81;
    if (t < 0) return 0;
    if (t < boostT) return 0.5 * boostAccel * t * t;

    float aT = apogeeTime();
    if (t < aT) {
      float vBoost   = boostAccel * boostT;
      float altBoost = 0.5 * boostAccel * boostT * boostT;
      float dt = t - boostT;
      return altBoost + vBoost * dt - 0.5 * g * dt * dt;
    }

    float descAlt = apogeeAltitude() - drogueRate * (t - aT);
    return descAlt > 0 ? descAlt : 0;
  }

  float elapsed() const {
    if (!running) return -1e6;
    return (millis() - t0) / 1000.0 - restT;
  }
};

FlightProfile flightProfile;

float altitudeToPressure(float alt_m) {
  return 101325.0 * pow(1.0 - 0.0065 * alt_m / 288.15, 5.255);
}

// ================== Classe BMP580 ==================
class Bmp580Emulator {
public:
  void begin(TwoWire &bus, uint8_t sda, uint8_t scl, uint8_t addr,
             const char* label, void (*onRecv)(int), void (*onReq)()) {
    wire = &bus; sdaPin = sda; sclPin = scl; address = addr; name = label;
    resetToDefaults();
    startSlave(onRecv, onReq);
  }

  void poll(float altitude_m, float t) {
    updateSimulatedData(altitude_m, t);
    if (millis() - lastActivity > 3000) {
      wire->end(); delay(2); startSlave(savedRecv, savedReq);
    }
  }

  void setFault(float bias, float start, float duration) {
    faultBiasPa = bias; faultStart = start; faultDuration = duration;
    faultActive = true; faultPersistent = false;
  }
  void setPersistentFault(float bias) {
    faultBiasPa = bias; faultActive = true; faultPersistent = true;
  }
  void clearFault() {
    faultActive = false; faultPersistent = false;
    faultBiasPa = 0; faultStart = 0; faultDuration = 0;
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
        Serial.print("["); Serial.print(name); Serial.println("] [RESET] soft-reset recu");
        resetToDefaults();
      }
      regPointer++;
      numBytes--;
    }
    registers[REG_OSR_EFF] = registers[REG_OSR_CONFIG] | 0x80;
  }

  void handleRequest() {
    lastActivity = millis();
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

  void startSlave(void (*onRecv)(int), void (*onReq)()) {
    savedRecv = onRecv; savedReq = onReq;
    wire->setSDA(sdaPin); wire->setSCL(sclPin);
    wire->begin(address);
    wire->onReceive(savedRecv);
    wire->onRequest(savedReq);
    lastActivity = millis();
  }

  volatile uint8_t registers[256];
  volatile uint8_t regPointer = 0;
  volatile unsigned long lastActivity = 0;

  void resetToDefaults() {
    memset((void*)registers, 0, sizeof(registers));
    registers[REG_CHIP_ID]    = CHIP_ID_BMP580;
    registers[REG_REV_ID]     = REV_ID_VALUE;
    registers[REG_ODR_CONFIG] = ODR_CONFIG_RESET_VAL;
    registers[REG_DSP_CONFIG] = DSP_CONFIG_RESET_VAL;
    registers[REG_STATUS]     = STATUS_NVM_RDY_BIT;
    registers[REG_INT_STATUS] = INT_STATUS_DRDY_BIT | INT_STATUS_POR_BIT;
  }

  bool  faultActive     = false;
  bool  faultPersistent = false;
  float faultBiasPa     = 0;
  float faultStart      = 0;
  float faultDuration   = 0;

  void updateSimulatedData(float altitude_m, float t) {
    float pressure = altitudeToPressure(altitude_m);
    if (faultActive) {
      if (faultPersistent || (t >= faultStart && t <= faultStart + faultDuration)) {
        pressure += faultBiasPa;
      }
    }
    float temperature = 22.5;

    int32_t rawTemp  = (int32_t)(temperature * 65536.0);
    int32_t rawPress = (int32_t)(pressure * 64.0);
    registers[REG_TEMP_XLSB]  = rawTemp & 0xFF;
    registers[REG_TEMP_LSB]   = (rawTemp >> 8) & 0xFF;
    registers[REG_TEMP_MSB]   = (rawTemp >> 16) & 0xFF;
    registers[REG_PRESS_XLSB] = rawPress & 0xFF;
    registers[REG_PRESS_LSB]  = (rawPress >> 8) & 0xFF;
    registers[REG_PRESS_MSB]  = (rawPress >> 16) & 0xFF;
    registers[REG_STATUS]     = STATUS_NVM_RDY_BIT;
    registers[REG_INT_STATUS] = INT_STATUS_DRDY_BIT | INT_STATUS_POR_BIT;
  }
};

// ================== Classe KX134 ==================
class Kx134Emulator {
public:
  void begin(TwoWire &bus, uint8_t sda, uint8_t scl, uint8_t addr,
             const char* label, void (*onRecv)(int), void (*onReq)()) {
    wire = &bus; sdaPin = sda; sclPin = scl; address = addr; name = label;
    resetToDefaults();
    startSlave(onRecv, onReq);
  }

  void poll() {
    if (millis() - lastActivity > 3000) {
      wire->end(); delay(2); startSlave(savedRecv, savedReq);
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

      if (regPointer == REG_CNTL2 && (val & CNTL2_SRST_BIT)) {
        Serial.print("["); Serial.print(name); Serial.println("] [RESET] SRST recu");
        resetToDefaults();
      }
      regPointer++;
      numBytes--;
    }
  }

  void handleRequest() {
    lastActivity = millis();
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

  void resetToDefaults() {
    memset((void*)registers, 0, sizeof(registers));
    registers[REG_WHO_AM_I] = WHO_AM_I_KX134;
    registers[REG_CNTL2]    = CNTL2_RESET_VAL;
    registers[REG_INS2]     = INS2_DRDY_BIT;
  }

  void startSlave(void (*onRecv)(int), void (*onReq)()) {
    savedRecv = onRecv; savedReq = onReq;
    wire->setSDA(sdaPin); wire->setSCL(sclPin);
    wire->begin(address);
    wire->onReceive(savedRecv);
    wire->onRequest(savedReq);
    lastActivity = millis();
  }
};

// ================== Instances et cablage ==================
Kx134Emulator  kx;   // 0x1E, sur Wire  (I2C0) -> GPIO4/5
Bmp580Emulator bmp3; // 0x47, sur Wire1 (I2C1) -> GPIO6/7

void onReceiveKX(int n)  { kx.handleReceive(n); }
void onRequestKX()       { kx.handleRequest(); }
void onReceiveBmp3(int n){ bmp3.handleReceive(n); }
void onRequestBmp3()     { bmp3.handleRequest(); }

// ================== Identite (clignotement LED) ==================
void updateIdentityBlink() {
  static unsigned long lastChange = 0;
  static int flashCount = 0;
  static bool ledState = false;

  unsigned long now = millis();
  unsigned long interval = ledState ? BLINK_MS
                          : (flashCount >= PICO_ID ? PAUSE_MS : BLINK_MS);

  if (now - lastChange < interval) return;
  lastChange = now;

  if (flashCount >= PICO_ID) flashCount = 0;

  ledState = !ledState;
  digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  if (!ledState) flashCount++;
}

// ================== Commandes serie ==================
float extractFloat(String cmd, String key, float defaultVal = 0) {
  int idx = cmd.indexOf(key);
  return idx < 0 ? defaultVal : cmd.substring(idx + key.length()).toFloat();
}

void applyFault(Bmp580Emulator &bmp, String cmd) {
  if (cmd.indexOf("CLEAR") > 0) { bmp.clearFault(); return; }
  float bias = extractFloat(cmd, "BIAS=");
  if (cmd.indexOf("PERSISTENT") > 0) {
    bmp.setPersistentFault(bias);
  } else {
    bmp.setFault(bias, extractFloat(cmd, "START="), extractFloat(cmd, "DURATION="));
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("PROFILE")) {
    flightProfile.boostAccel = extractFloat(cmd, "BOOST_ACCEL=", flightProfile.boostAccel);
    flightProfile.boostT     = extractFloat(cmd, "BOOST_T=", flightProfile.boostT);
    flightProfile.drogueRate = extractFloat(cmd, "DROGUE_RATE=", flightProfile.drogueRate);
    flightProfile.restT      = extractFloat(cmd, "REST=", flightProfile.restT);
    Serial.println("[CFG] Profil mis a jour");
  } else if (cmd.startsWith("FAULT")) {
    if (cmd.indexOf("BMP3") > 0) applyFault(bmp3, cmd);
  } else if (cmd.startsWith("START")) {
    flightProfile.t0 = millis();
    flightProfile.running = true;
    Serial.println("[CFG] Vol demarre");
  } else if (cmd.startsWith("RESET")) {
    flightProfile.running = false;
    bmp3.clearFault();
    Serial.println("[CFG] Reset");
  }
}

void parseSerialCommands() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handleCommand(line); line = ""; }
    else if (c != '\r') { line += c; }
  }
}

// ================== setup() / loop() ==================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("--- Emulateur KX134 + BMP580 (profil + defauts) ---");

  pinMode(LED_PIN, OUTPUT);

  kx.begin(Wire,   4, 5, 0x1E, "KX134", onReceiveKX,   onRequestKX);
  bmp3.begin(Wire1, 6, 7, 0x47, "BMP47", onReceiveBmp3, onRequestBmp3);

  Serial.println("Pret, en attente du master...");
}

void loop() {
  parseSerialCommands();
  float t = flightProfile.elapsed();
  float alt = flightProfile.altitude(t);
  kx.poll();
  bmp3.poll(alt, t);
  updateIdentityBlink();
  delay(20);
}