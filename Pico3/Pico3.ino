#include <Wire.h>

#define PICO_ID   3        
#define LED_PIN   LED_BUILTIN
#define BLINK_MS  150      // duree ON et OFF de chaque flash
#define PAUSE_MS  1000     // pause avant de recommencer le motif
#define GNSS_CONFIG_ADDR 0x50  // OBLIGATOIRE : verifiee par check_connection() au boot
#define GNSS_READ_ADDR   0x54  // Optionnelle : evite juste le bruit de log au setup

volatile unsigned long lastConfigActivity = 0;
volatile unsigned long lastReadActivity   = 0;

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

void onConfigReceive(int numBytes) {
  lastConfigActivity = millis();
  Serial.print("[GNSS-CFG] recu ");
  Serial.print(numBytes);
  Serial.println(" octets (contenu sans importance)");
  while (Wire.available()) Wire.read();
}

void onReadRequest() {
  lastReadActivity = millis();
  Serial.println("[GNSS-READ] requestFrom -> 0 octet disponible");
  uint8_t zero[4] = {0, 0, 0, 0};
  Wire1.write(zero, 4); // "rien a lire / rien a ecrire", gere proprement par ton code
}

void onReadReceive(int numBytes) {
  while (Wire1.available()) Wire1.read(); // jamais cense arriver, purge par securite
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

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("--- Emulateur LC76G (bypass init) ---");
  startConfigSlave();
  startReadSlave();
  Serial.println("Pret, en attente du master...");
}

void loop() {
  if (millis() - lastConfigActivity > 3000) { Wire.end();  delay(2); startConfigSlave(); }
  if (millis() - lastReadActivity   > 3000) { Wire1.end(); delay(2); startReadSlave();  }
  updateIdentityBlink();
  delay(50);
}
