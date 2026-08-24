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

// ================== Identite (clignotement LED) ==================
#define PICO_ID   1        // ce Pico = BMP1 + BMP2
#define LED_PIN   LED_BUILTIN
#define BLINK_MS  150
#define PAUSE_MS  1000

// ================== SEP_MECHD ==================
#define SEPMEC_PIN 2

float sepmecOffset = 0;
bool  sepmecTriggered = false;

// ================== Profil de vol partage ==================
struct FlightProfile {
  float boostAccel = 80.0;  // m/s^2
  float boostT = 3.0;       // s
  float drogueRate = 20.0;  // m/s, vitesse de descente constante sous drogue
  float restT = 0;          // s, temps au repos sur le pas de tir avant le decollage
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

  // t < 0 : encore au sol (repos ou pas encore demarre) -> altitude 0
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

  // t=0 correspond au decollage ; juste apres START, t = -restT (repos), puis
  // remonte vers 0 au moment du decollage reel.
  float elapsed() const {
    if (!running) return -1e6; // jamais demarre - tres negatif, ne matche aucun fault
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

// ================== Instances et cablage ==================
Bmp580Emulator bmpA; // 0x47, sur Wire  (I2C0) -> GPIO4/5
Bmp580Emulator bmpB; // 0x46, sur Wire1 (I2C1) -> GPIO6/7

void onReceiveA(int n) { bmpA.handleReceive(n); }
void onRequestA()      { bmpA.handleRequest(); }
void onReceiveB(int n) { bmpB.handleReceive(n); }
void onRequestB()      { bmpB.handleRequest(); }

// ================== SEP_MECHD ==================
void updateSepMech(float t) {
  if (t < 0 || sepmecTriggered) return;
  if (t >= (flightProfile.apogeeTime() + sepmecOffset)) {
    pinMode(SEPMEC_PIN, OUTPUT);
    digitalWrite(SEPMEC_PIN, HIGH);
    sepmecTriggered = true;
    Serial.println("[SEPMEC] Separation simulee declenchee");
  }
}

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
  } else if (cmd.startsWith("SEPMEC")) {
    sepmecOffset = extractFloat(cmd, "OFFSET=");
    Serial.print("[CFG] SEPMEC offset = "); Serial.println(sepmecOffset);
  } else if (cmd.startsWith("FAULT")) {
    if (cmd.indexOf("BMP1") > 0) applyFault(bmpA, cmd);
    if (cmd.indexOf("BMP2") > 0) applyFault(bmpB, cmd);
  } else if (cmd.startsWith("START")) {
    flightProfile.t0 = millis();
    flightProfile.running = true;
    Serial.println("[CFG] Vol demarre");
  } else if (cmd.startsWith("RESET")) {
    flightProfile.running = false;
    bmpA.clearFault(); bmpB.clearFault();
    sepmecTriggered = false;
    pinMode(SEPMEC_PIN, INPUT);
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
  Serial.println("--- Emulateur 2x BMP580 (profil + defauts + sepmec) ---");

  pinMode(LED_PIN, OUTPUT);
  pinMode(SEPMEC_PIN, INPUT); // Hi-Z par defaut, R7 (10k pulldown) maintient LOW

  bmpA.begin(Wire,  4, 5, 0x47, "BMP47", onReceiveA, onRequestA);
  bmpB.begin(Wire1, 6, 7, 0x46, "BMP46", onReceiveB, onRequestB);

  Serial.println("Pret, en attente du master...");
}

void loop() {
  parseSerialCommands();
  float t = flightProfile.elapsed();
  float alt = flightProfile.altitude(t); // gere deja t<0 (repos/pas demarre) -> 0
  bmpA.poll(alt, t);
  bmpB.poll(alt, t);
  updateSepMech(t);
  updateIdentityBlink();
  delay(20);
}