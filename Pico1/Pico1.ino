#include <Wire.h>

// ==========================================
// 1. CLASSE MÈRE (Sécurisée pour ARM Cortex)
// ==========================================
class SensorEmulator {
protected:
    uint8_t _address;
    
    // CRITIQUE : volatile force le CPU à lire la vraie mémoire RAM à chaque fois
    volatile uint8_t _regPointer; 
    volatile uint8_t _registers[256]; 

public:
    SensorEmulator(uint8_t address) : _address(address), _regPointer(0) {
        // Pas de memset sur du volatile, on utilise une boucle standard
        for(int i = 0; i < 256; i++) _registers[i] = 0; 
    }

    uint8_t getAddress() const { return _address; }

    virtual void onReceive(int numBytes) {
        if (numBytes == 0 || !Wire.available()) return;
        
        // 1. Le Master indique quel registre il veut cibler
        _regPointer = Wire.read(); 
        
        // 2. Si le Master s'arrête là, c'est qu'il prépare une lecture (Repeated Start)
        if (Wire.available() == 0) return;

        // 3. S'il reste des données, c'est une écriture. On lit le premier octet.
        uint8_t data = Wire.read();
        
        // On délègue le traitement à l'enfant (le BMP581)
        handleWrite(_regPointer, data);

        // 4. BOUCLIER FAÇON UNO : On draine le reste du buffer pour éviter tout désalignement
        while (Wire.available()) Wire.read(); 
    }

    virtual void onRequest() {
        // Calcul pour éviter un dépassement de mémoire (Out of Bounds)
        int bytesToSend = 32; 
        if (_regPointer + bytesToSend > 256) {
            bytesToSend = 256 - _regPointer; 
        }
        
        // CRITIQUE : On copie les variables 'volatile' dans un buffer statique propre
        // avant de le donner au contrôleur matériel I2C du Pico.
        uint8_t tempBuffer[32];
        for (int i = 0; i < bytesToSend; i++) {
            tempBuffer[i] = _registers[_regPointer + i];
        }
        
        Wire.write(tempBuffer, bytesToSend);
    }

    // Méthode virtuelle pour gérer les écritures spécifiques du capteur
    virtual void handleWrite(uint8_t reg, uint8_t data) {
        _registers[reg] = data; // Comportement par défaut
    }
};

// ==========================================
// 2. CLASSE ENFANT : BMP581
// ==========================================
class BMP581Emulator : public SensorEmulator {
public:
    BMP581Emulator(uint8_t address = 0x47) : SensorEmulator(address) {
        powerOnReset();
    }

    void powerOnReset() {
        for(int i = 0; i < 256; i++) _registers[i] = 0;

        _registers[0x01] = 0x50; // CHIP_ID (Attendu par Adafruit)
        _registers[0x28] = 0x11; // STATUS : NVM et CMD Ready
        _registers[0x27] = 0x03; // INT_STATUS : Data Ready

        injectTemperature(20.0);
        injectPressure(101325.0);
    }

    // On intercepte la commande de Soft Reset (0xB6 dans le registre 0x7E)
    void handleWrite(uint8_t reg, uint8_t data) override {
        if (reg == 0x7E && data == 0xB6) {
            powerOnReset(); 
        } else {
            _registers[reg] = data; 
        }
    }

    void injectPressure(float pressure_pa) {
        uint32_t raw_p = (uint32_t)(pressure_pa * 64.0f);
        _registers[0x20] = raw_p & 0xFF;         // XLSB
        _registers[0x21] = (raw_p >> 8) & 0xFF;  // LSB
        _registers[0x22] = (raw_p >> 16) & 0xFF; // MSB
    }

    void injectTemperature(float temp_c) {
        uint32_t raw_t = (uint32_t)(temp_c * 65536.0f);
        _registers[0x1D] = raw_t & 0xFF;         // XLSB
        _registers[0x1E] = (raw_t >> 8) & 0xFF;  // LSB
        _registers[0x1F] = (raw_t >> 16) & 0xFF; // MSB
    }
};

// ==========================================
// 3. WRAPPERS & SETUP
// ==========================================

BMP581Emulator myBMP(0x47);

void i2c_receive_handler(int numBytes) {
    myBMP.onReceive(numBytes);
}

void i2c_request_handler() {
    myBMP.onRequest();
}

void setup() {
    // Initialisation du port USB
    Serial.begin(115200);

    uint32_t t = millis();
    while (!Serial && (millis() - t < 3000)) delay(10);

    Serial.println("\n--- DEMARRAGE SYSTEME HiL ---");
    
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin(myBMP.getAddress()); 
    Wire.onReceive(i2c_receive_handler);
    Wire.onRequest(i2c_request_handler);

    pinMode(LED_BUILTIN, OUTPUT);    Serial.println("HiL Pico 2 V3 : I2C Slave actif sur l'adresse 0x47.");
}

void loop() {
    static float fake_pressure = 101325.0;
    static float delta = 1.0;
    
    // Variation de la pression
    fake_pressure += delta;
    if(fake_pressure > 101400.0 || fake_pressure < 101250.0) delta = -delta;
    
    myBMP.injectPressure(fake_pressure); 
    
    // HEARTBEAT : Affichage toutes les secondes pour confirmer que le code tourne
    static uint32_t last_print = 0;
    if (millis() - last_print > 1000) {
        last_print = millis();
        Serial.print("[PICO] Pression injectée : ");
        Serial.print(fake_pressure);
        Serial.println(" Pa");
        
        // Fait clignoter la LED très brièvement
        digitalWrite(LED_BUILTIN, HIGH);
        delay(10);
        digitalWrite(LED_BUILTIN, LOW);
    }
    
    delay(40); // 40ms + 10ms (led) = 50ms (20 Hz)
}