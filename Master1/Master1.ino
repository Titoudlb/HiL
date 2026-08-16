#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>
#include <SparkFun_KX13X.h>

// --- Bus 1 : KX134 + BMP1 (0x47) + BMP2 (0x46) ---
#define SDA1_PIN 6
#define SCL1_PIN 7
#define KX134_ADDR 0x1E
#define BMP2_ADDR  0x46   // valeur brute, cf. note ci-dessus

// --- Bus 2 : BMP3 (0x47), pins par defaut -> AJUSTE si besoin ---
// Wire.setSDA(x); Wire.setSCL(y); avant Wire.begin() si tes pins reels different

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP5xx bmp1; // bus 1, 0x47
Adafruit_BMP5xx bmp2; // bus 1, 0x46
Adafruit_BMP5xx bmp3; // bus 2, 0x47
SparkFun_KX134  kxAccel; // bus 1
outputData accelData;

void configureBmp(Adafruit_BMP5xx &b) {
  b.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  b.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  b.setPowerMode(BMP5XX_POWERMODE_NORMAL);
  b.enablePressure(true);
}

void printBmp(const char* label, Adafruit_BMP5xx &b) {
  if (b.dataReady() && b.performReading()) {
    Serial.print(label);
    Serial.print(" -> T="); Serial.print(b.temperature);
    Serial.print(" *C, P="); Serial.print(b.pressure);
    Serial.print(" hPa, Alt="); Serial.print(b.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
  }
}

void setup() {
  
  Serial.begin(115200);
  delay(2000);
  Serial.println("--- Master : BMP1 + BMP2 + KX134 (bus1) / BMP3 (bus2) ---");
  Wire.setBufferSize(9560);
  Wire1.setBufferSize(9560);
  // Bus 1
  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire1.begin();

  if (!bmp1.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire1)) {
    Serial.println("BMP1 (0x47, bus1) non trouve !"); while (1) delay(10);
  }
  Serial.println("BMP1 trouve !");
  configureBmp(bmp1);

  if (!bmp2.begin(BMP2_ADDR, &Wire1)) {
    Serial.println("BMP2 (0x46, bus1) non trouve !"); while (1) delay(10);
  }
  Serial.println("BMP2 trouve !");
  configureBmp(bmp2);

  if (!kxAccel.begin(Wire1, KX134_ADDR)) {
    Serial.println("KX134 (bus1) non trouve !"); while (1) delay(10);
  }
  Serial.println("KX134 trouve !");
  if (kxAccel.softwareReset()) Serial.println("KX134 reset.");
  delay(5);
  kxAccel.enableAccel(false);
  kxAccel.setRange(SFE_KX134_RANGE16G);
  kxAccel.enableDataEngine();
  kxAccel.enableAccel();

  // Bus 2
  Wire.setSDA(D14);  
  Wire.setSCL(D13);  
  Wire.begin();

  if (!bmp3.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
    Serial.println("BMP3 (0x47, bus2) non trouve !"); while (1) delay(10);
  }
  Serial.println("BMP3 trouve !");
  configureBmp(bmp3);
}

void loop() {
  printBmp("BMP1 (0x47, bus1)", bmp1);
  printBmp("BMP2 (0x46, bus1)", bmp2);
  printBmp("BMP3 (0x47, bus2)", bmp3);

  if (kxAccel.dataReady()) {
    kxAccel.getAccelData(&accelData);
    Serial.print("KX134 (bus1) -> X="); Serial.print(accelData.xData, 4);
    Serial.print(" Y="); Serial.print(accelData.yData, 4);
    Serial.print(" Z="); Serial.print(accelData.zData, 4);
    Serial.println(" g");
  }

  Serial.println("---");
  delay(2000);
}