#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- RECORDING STRUCTURE ---
struct Paso {
  int hombro;
  int codo;
  int mano;
  long base; // Stores absolute actual degrees, not arbitrary steps
};

// --- MEMORY ARRAYS (DUAL BANK) ---
// Bank 1 (Triggered by R, M, C keys)
Paso secuencia1[50]; 
int guardados1 = 0;

// Bank 2 (Triggered by V, B, N keys)
Paso secuencia2[50]; 
int guardados2 = 0;

// --- BLE CONFIGURATION ---
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char bleIncomingChar = 0; 

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// --- I2C CONFIGURATION (ESP32-C3) ---
#define SDA_PIN 6
#define SCL_PIN 7
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- ARM CALIBRATION LIMITS ---
#define SERVOMIN  150 
#define SERVOMAX  600 
const int BASE_STOP = 275; // PWM value for base motor deadband center (full stop)

// --- BASE KINEMATICS (QUADRATIC CURVE COMPENSATION) ---
int velocidadGiro = 20; 

// Kinematic Equation: Time = (A * degrees^2) + (B * degrees) + C
// Right Turn Coefficients
float A_DER = -0.02;  // Inertia correction for long travels (reduces time)
float B_DER = 21.0;   // Linear velocity (ms/degree)
float C_DER = 40.0;   // Static friction threshold (base ms delay)

// Left Turn Coefficients (Compensates for mechanical asymmetry)
float A_IZQ = -0.03;  
float B_IZQ = 25.0;   
float C_IZQ = 45.0;

// --- HARDWARE PINS ---
#define PIN_BASE    0
#define PIN_HOMBRO  1
#define PIN_CODO    2
#define PIN_MANO    3

// --- STATE VARIABLES ---
int posHombro = 30;
int posCodo = 20;
int posMano = 40; 
long posBase = 0; // Absolute base position in degrees (0 = Home)

// --- BLE CALLBACKS ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Device Connected");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Device Disconnected");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        bleIncomingChar = rxValue[0];
      }
    }
};

// --- CORE KINEMATICS ---

// Maps angle (0-180) to PWM pulse length for standard servos
void moverServoPosicional(int nServo, int grados) {
  int pulso = map(grados, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(nServo, 0, pulso);
}

// Executes a non-linear compensated rotation for the continuous base servo
void irBaseHasta(long destinoGrados) {
  if (posBase == destinoGrados) return; 

  Serial.print("Moving Base: "); Serial.print(posBase);
  Serial.print(" deg -> "); Serial.print(destinoGrados); Serial.println(" deg");

  long diferencia = destinoGrados - posBase; 
  long gradosAbsolutos = abs(diferencia);
  long tiempoViaje = 0;

  if (diferencia > 0) {
     // Clockwise (Right) Rotation
     tiempoViaje = (A_DER * gradosAbsolutos * gradosAbsolutos) + (B_DER * gradosAbsolutos) + C_DER;
     
     // Prevent negative time values on extreme distance curves
     if (tiempoViaje < 0) tiempoViaje = 0; 
     
     pwm.setPWM(PIN_BASE, 0, BASE_STOP + velocidadGiro);
  } else {
     // Counter-Clockwise (Left) Rotation
     tiempoViaje = (A_IZQ * gradosAbsolutos * gradosAbsolutos) + (B_IZQ * gradosAbsolutos) + C_IZQ;
     
     if (tiempoViaje < 0) tiempoViaje = 0;

     pwm.setPWM(PIN_BASE, 0, BASE_STOP - velocidadGiro);
  }

  // Execute calculated travel time
  delay(tiempoViaje);

  // Active braking to center deadband
  pwm.setPWM(PIN_BASE, 0, BASE_STOP);
  
  posBase = destinoGrados;
}

// Synchronized multi-axis smooth interpolation
void irBrazoSuave(int destHombro, int destCodo, int destMano) {
  while(posHombro != destHombro || posCodo != destCodo || posMano != destMano) {
    if(posHombro < destHombro) posHombro++;
    if(posHombro > destHombro) posHombro--;
    moverServoPosicional(PIN_HOMBRO, posHombro);

    if(posCodo < destCodo) posCodo++;
    if(posCodo > destCodo) posCodo--;
    moverServoPosicional(PIN_CODO, posCodo);

    if(posMano < destMano) posMano++;
    if(posMano > destMano) posMano--;
    moverServoPosicional(PIN_MANO, posMano);

    delay(15); 
  }
}

// --- SMART HOME SEQUENCE ---
void irHomeInteligente() {
  int destHombro = 30;
  int destCodo = 20;
  int destMano = 40; 
  long destBase = 0; 

  Serial.println("Initiating SMART HOME sequence...");

  // 1. Secure payload (close gripper smoothly) before moving arm
  if (posMano != destMano) {
    while (posMano != destMano) {
      if (posMano < destMano) posMano++;
      else posMano--;
      moverServoPosicional(PIN_MANO, posMano);
      delay(15); 
    }
  }

  // 2. Center base using quadratic compensation
  irBaseHasta(destBase);

  int startHombro = posHombro;
  int startCodo = posCodo;
  int startMano = posMano; 

  float diffHombro = destHombro - startHombro;
  float diffCodo = destCodo - startCodo;
  float diffMano = destMano - startMano;

  int maxPasos = max(abs((int)diffHombro), max(abs((int)diffCodo), abs((int)diffMano)));

  // 3. Proportional multi-axis interpolation for linear movement
  if (maxPasos > 0) {
    for (int i = 1; i <= maxPasos; i++) {
      posHombro = startHombro + (diffHombro * i) / maxPasos;
      posCodo = startCodo + (diffCodo * i) / maxPasos;
      posMano = startMano + (diffMano * i) / maxPasos;

      moverServoPosicional(PIN_HOMBRO, posHombro);
      moverServoPosicional(PIN_CODO, posCodo);
      moverServoPosicional(PIN_MANO, posMano);

      delay(15); 
    }
  }

  // 4. Final state enforcement (anti-drift correction)
  posHombro = destHombro;
  posCodo = destCodo;
  posMano = destMano;
  moverServoPosicional(PIN_HOMBRO, posHombro);
  moverServoPosicional(PIN_CODO, posCodo);
  moverServoPosicional(PIN_MANO, posMano);

  Serial.println("SMART HOME sequence complete.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize I2C and PWM Driver
  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  
  // Enforce initial base stop
  pwm.setPWM(PIN_BASE, 0, BASE_STOP); 
  irHomeInteligente();

  // Initialize BLE Stack
  BLEDevice::init("ESP32C3SuperMini-RobotARM");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());

  BLECharacteristic *pCharacteristicTX = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pCharacteristicTX->addDescriptor(new BLE2902());

  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->start(); 
  
  Serial.println("--- DUAL BANK MEMORY SYSTEM INIT (QUADRATIC COMPENSATED BASE) ---");
}

void loop() {
  char letra = 0;

  // Prioritize BLE input over Serial
  if (Serial.available() > 0) letra = Serial.read();
  if (bleIncomingChar != 0) {
    letra = bleIncomingChar;
    bleIncomingChar = 0;
  }

  if (letra != 0) {
    switch (letra) {
      
      case 'h': case 'H': 
        irHomeInteligente();
        break;

      // --- MEMORY BANK 1 CONTROLS ---
      case 'r': case 'R': 
        if (guardados1 < 50) {
          secuencia1[guardados1] = {posHombro, posCodo, posMano, posBase};
          guardados1++;
          Serial.print("Bank 1 Saved: Step "); Serial.println(guardados1);
        } else Serial.println("Bank 1 is full");
        break;

      case 'm': case 'M': 
        Serial.println("Playing Bank 1...");
        for (int i = 0; i < guardados1; i++) {
          irBaseHasta(secuencia1[i].base); 
          irBrazoSuave(secuencia1[i].hombro, secuencia1[i].codo, secuencia1[i].mano);
          delay(400); 
        }
        Serial.println("Bank 1 sequence complete");
        break;

      case 'c': case 'C': 
        guardados1 = 0;
        Serial.println("Bank 1 cleared");
        break;

      // --- MEMORY BANK 2 CONTROLS ---
      case 'v': case 'V': 
        if (guardados2 < 50) {
          secuencia2[guardados2] = {posHombro, posCodo, posMano, posBase};
          guardados2++;
          Serial.print("Bank 2 Saved: Step "); Serial.println(guardados2);
        } else Serial.println("Bank 2 is full");
        break;

      case 'b': case 'B': 
        Serial.println("Playing Bank 2...");
        for (int i = 0; i < guardados2; i++) {
          irBaseHasta(secuencia2[i].base); 
          irBrazoSuave(secuencia2[i].hombro, secuencia2[i].codo, secuencia2[i].mano);
          delay(400); 
        }
        Serial.println("Bank 2 sequence complete");
        break;

      case 'n': case 'N': 
        guardados2 = 0;
        Serial.println("Bank 2 cleared");
        break;

      // --- MANUAL OVERRIDE CONTROLS ---
      case 'q': case 'Q': 
        irBaseHasta(posBase - 5); // Base Left (-5 deg)
        break;
        
      case 'e': case 'E': 
        irBaseHasta(posBase + 5); // Base Right (+5 deg)
        break;

      case 'w': case 'W': posHombro -= 3; break;
      case 's': case 'S': posHombro += 3; break;
      case 'd': case 'D': posCodo -= 3; break;
      case 'a': case 'A': posCodo += 3; break;

      case 'o': case 'O': 
        posMano = 40; 
        moverServoPosicional(PIN_MANO, posMano); 
        break;
      case 'p': case 'P': 
        posMano = 130; 
        moverServoPosicional(PIN_MANO, posMano); 
        break;
    }

    // Apply physical constraints to positional servos
    posHombro = constrain(posHombro, 0, 100); 
    posCodo = constrain(posCodo, 0, 90);     

    // Update positions (excluding memory, manual base, and home commands)
    if (letra != 'r' && letra != 'm' && letra != 'c' && 
        letra != 'v' && letra != 'b' && letra != 'n' &&
        letra != 'q' && letra != 'e' && letra != 'h' && letra != 'H') {
      moverServoPosicional(PIN_HOMBRO, posHombro);
      moverServoPosicional(PIN_CODO, posCodo);
    }
  }

  // Handle BLE Connection State
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); 
      pServer->startAdvertising(); 
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }
  
  delay(10);
}
