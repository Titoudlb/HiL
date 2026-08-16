#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>
#include <SparkFun_KX13X.h>

#define SDA_PIN 6
#define SCL_PIN 7
#define SEALEVELPRESSURE_HPA (1013.25)
#define KX134_ADDR 0x1E

Adafruit_BMP5xx bmp;
SparkFun_KX134 kxAccel;
outputData accelData; // struct rempli par getAccelData()

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("--- Master BMP580 + KX134 ---");

  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();

  if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire1)) {
    Serial.println("Capteur BMP5xx non trouve, verifie le cablage !");
    while (1) delay(10);
  }
  Serial.println("BMP5xx trouve !");

  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
  bmp.enablePressure(true);

  // --- KX134, meme bus Wire1 ---
  if (!kxAccel.begin(Wire1, KX134_ADDR)) {
    Serial.println("KX134 non trouve, verifie le cablage !");
    while (1) delay(10);
  }
  Serial.println("KX134 trouve !");

  if (kxAccel.softwareReset()) Serial.println("KX134 reset.");
  delay(5); // le capteur a besoin d'~2ms pour terminer le reset, 5 par securite

  kxAccel.enableAccel(false);   // la config ne se fait qu'accelero desactive
  kxAccel.setRange(SFE_KX134_RANGE16G);
  kxAccel.enableDataEngine();
  kxAccel.enableAccel();
}

void loop() {
  if (bmp.dataReady() && bmp.performReading()) {
    Serial.print("Temperature = "); Serial.print(bmp.temperature); Serial.println(" *C");
    Serial.print("Pression = "); Serial.print(bmp.pressure); Serial.println(" hPa");
    Serial.print("Altitude approx = "); Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)); Serial.println(" m");
  }

  if (kxAccel.dataReady()) {
    kxAccel.getAccelData(&accelData);
    Serial.print("Accel X = "); Serial.print(accelData.xData, 4); Serial.println(" g");
    Serial.print("Accel Y = "); Serial.print(accelData.yData, 4); Serial.println(" g");
    Serial.print("Accel Z = "); Serial.print(accelData.zData, 4); Serial.println(" g");
  }

  Serial.println("---");
  delay(2000);
}