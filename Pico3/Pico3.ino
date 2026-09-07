#include <Wire.h>

#define GNSS_CONFIG_ADDR 0x50
#define GNSS_READ_ADDR   0x54

volatile unsigned long lastConfigActivity = 0;
volatile unsigned long lastReadActivity   = 0;

// ================== Horloge de vol minimale (pas de trajectoire ici) ==================
unsigned long t0 = 0;
bool  running = false;
float restT = 0;

bool  disconnectArmed  = false;
float disconnectAt     = 0;
unsigned long disconnectArmTime = 0;
bool  gnssDisconnected = false;

void onConfigReceive(int numBytes) {
  lastConfigActivity = millis();
  while (Wire.available()) Wire.read();
}

void onReadRequest() {
  lastReadActivity = millis();
  uint8_t zero[4] = {0, 0, 0, 0};
  Wire1.write(zero, 4);
}

void onReadReceive(int numBytes) {
  while (Wire1.available()) Wire1.read();
}

void startConfigSlave() {
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin(GNSS_CONFIG_ADDR);
  Wire.onReceive(onConfigReceive);
  lastConfigActivity = millis();
}

void startReadSlave() {
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin(GNSS_READ_ADDR);
  Wire1.onReceive(onReadReceive);
  Wire1.onRequest(onReadRequest);
  lastReadActivity = millis();
}

void disconnectGnss() {
  Wire.end();
  Wire1.end();
  gnssDisconnected = true;
  Serial.println("[GNSS] [DISCONNECT] simule - plus de reponse sur 0x50 et 0x54");
}

void reconnectGnss() {
  gnssDisconnected = false;
  startConfigSlave();
  startReadSlave();
}

// ================== Identite (clignotement LED) ==================
#define PICO_ID   3
#define LED_PIN   LED_BUILTIN
#define BLINK_MS  150
#define PAUSE_MS  1000

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

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("PROFILE")) {
    restT = extractFloat(cmd, "REST=", restT);
    Serial.println("[CFG] REST synchronise (pas de trajectoire sur ce Pico)");
  } else if (cmd.startsWith("DISCONNECT")) {
    if (cmd.indexOf("GNSS") > 0) {
      if (cmd.indexOf("CLEAR") > 0) {
        disconnectArmed = false;
        if (gnssDisconnected) reconnectGnss();
      } else {
        disconnectArmed = true;
        disconnectAt = extractFloat(cmd, "AT=");
        disconnectArmTime = millis();
      }
    }
  } else if (cmd.startsWith("START")) {
    t0 = millis();
    running = true;
    Serial.println("[CFG] Vol demarre");
  } else if (cmd.startsWith("RESET")) {
    running = false;
    disconnectArmed = false;
    if (gnssDisconnected) reconnectGnss();
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

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("--- Emulateur LC76G (bypass init + deconnexion simulable) ---");

  pinMode(LED_PIN, OUTPUT);
  startConfigSlave();
  startReadSlave();

  Serial.println("Pret, en attente du master...");
}

void loop() {
  parseSerialCommands();

  if (disconnectArmed && !gnssDisconnected) {
    float sinceArm = (millis() - disconnectArmTime) / 1000.0;
    if (sinceArm >= disconnectAt) {
      disconnectGnss();
    }
  }

  if (!gnssDisconnected) {
    if (millis() - lastConfigActivity > 3000) { Wire.end();  delay(2); startConfigSlave(); }
    if (millis() - lastReadActivity   > 3000) { Wire1.end(); delay(2); startReadSlave();  }
  }

  updateIdentityBlink();
  delay(50);
}