#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Définition des broches spécifiques à la Heltec V3
#define SDA_PIN 41
#define SCL_PIN 42

Adafruit_BME280 bme; 

void setup() {
  Serial.begin(115200);
  Serial.println(F("Démarrage du système... Recherche du capteur BME280"));

  // L'étape cruciale : on initialise le bus I2C sur TES broches
  Wire.begin(SDA_PIN, SCL_PIN);

  // On passe l'adresse 0x76 et l'objet Wire qu'on vient de configurer
  if (!bme.begin(0x76, &Wire)) {
    Serial.println("Erreur critique : BME280 introuvable. Vérifiez le câblage !");
    while (1) delay(10); 
  }
  Serial.println("BME280 détecté et initialisé avec succès !");
}

void loop() {
  Serial.print("Température = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pression = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidité = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println("---");
  delay(1000); 
}