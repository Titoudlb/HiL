#include <Wire.h>
#include <math.h>

volatile uint8_t reg_addr = 0; // Garde en mémoire le registre demandé par l'ESP32
uint8_t bme_regs[256];         // Simule la mémoire interne du BME280

void setup() {
  Serial.begin(115200);
  
  // Remplissage de la mémoire virtuelle
  memset(bme_regs, 0, 256);
  
  // 1. Signature du BME280 (Très important, c'est ce que l'ESP vérifie en premier)
  bme_regs[0xD0] = 0x60; 

  // 2. Fausses données de calibration 
  // (Évite que la librairie de l'ESP32 ne fasse des divisions par zéro et plante)
  for(int i = 0x88; i <= 0xA1; i++) bme_regs[i] = 128; 
  for(int i = 0xE1; i <= 0xF0; i++) bme_regs[i] = 128;

  // On configure l'Uno en esclave I2C sur l'adresse 0x76
  Wire.begin(0x76); 
  Wire.onReceive(receiveEvent); // Quand l'ESP écrit/demande un registre
  Wire.onRequest(requestEvent); // Quand l'ESP lit les données

  Serial.println("Arduino prêt : Mode émulation BME280 activé.");
}

void loop() {
  // On fait vivre les données pour la démo (simule des variations physiques)
  static uint32_t counter = 0;
  counter++;

  // Génération de fausses valeurs brutes avec des sinusoides pour faire "naturel"
  uint32_t fake_temp_adc = 512000 + (sin(counter * 0.1) * 10000);
  bme_regs[0xFA] = (fake_temp_adc >> 12) & 0xFF; // msb
  bme_regs[0xFB] = (fake_temp_adc >> 4) & 0xFF;  // lsb
  bme_regs[0xFC] = (fake_temp_adc & 0x0F) << 4;  // xlsb

  uint32_t fake_press_adc = 300000 + (cos(counter * 0.05) * 20000);
  bme_regs[0xF7] = (fake_press_adc >> 12) & 0xFF;
  bme_regs[0xF8] = (fake_press_adc >> 4) & 0xFF;
  bme_regs[0xF9] = (fake_press_adc & 0x0F) << 4;

  uint32_t fake_hum_adc = 25000 + (sin(counter * 0.2) * 5000);
  bme_regs[0xFD] = (fake_hum_adc >> 8) & 0xFF;
  bme_regs[0xFE] = fake_hum_adc & 0xFF;

  delay(100); // Mise à jour des registres internes 10 fois par seconde
}

// Fonction appelée quand l'ESP32 pointe vers un registre spécifique
void receiveEvent(int bytes) {
  if (Wire.available()) {
    reg_addr = Wire.read(); // On stocke l'adresse du registre ciblé
    // On ignore les écritures de configuration potentielles pour garder ça simple
    while (Wire.available()) Wire.read(); 
  }
}

// Fonction appelée quand l'ESP32 réclame les octets
void requestEvent() {
  // La librairie BME lit par blocs de 24 octets max pour la calibration
  int bytes_to_send = 24; 
  if (reg_addr + bytes_to_send > 256) {
    bytes_to_send = 256 - reg_addr; // Sécurité pour ne pas lire hors du tableau
  }
  // On envoie le bout de mémoire virtuelle demandé
  Wire.write(&bme_regs[reg_addr], bytes_to_send); 
}