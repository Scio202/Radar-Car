#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <LSM303.h>
#include <Stepper.h>

// =============================================
// Hardware Pin Definitions
// =============================================

// Stepper Motor Pins
#define STEPPER_IN1 18
#define STEPPER_IN2 17
#define STEPPER_IN3 16
#define STEPPER_IN4 5

// DC Motor Control Pins
#define MOTOR_L_A 19
#define MOTOR_L_B 23
#define MOTOR_R_A 27
#define MOTOR_R_B 13

// Ultrasonic Sensor Pins
#define TRIG_PIN 25
#define ECHO_PIN 26

// I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

// Buzzer Pin
#define BUZZER_PIN 14

// =============================================
// System Constants
// =============================================
const uint16_t STEPS_PER_REVOLUTION = 2048;
const uint8_t POSITION_COUNT = 64;
const uint16_t STEPS_PER_POSITION = STEPS_PER_REVOLUTION / POSITION_COUNT;
const uint8_t MAX_SPEED_RPM = 15;
const uint16_t MAX_DISTANCE_CM = 400;

// =============================================
// BLE Configuration
// =============================================
#define SERVICE_UUID_CONTROL "36c5ada5-8eb3-4256-b474-e9f5ae28e93a"
#define CHAR_UUID_MOTOR "bfdf57a4-de50-4ca3-af11-4effa090cc11"
#define CHAR_UUID_DISTANCE "87a90c14-bb71-4109-9124-abb093d0814f"

// =============================================
// Global Variables
// =============================================

// BLE Variables
BLEServer* pServer = NULL;
BLECharacteristic* pMotorCharacteristic = NULL;
BLECharacteristic* pDistanceCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Hardware Instances
LSM303 compass;
Stepper stepper(STEPS_PER_REVOLUTION, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

// System State Variables
bool isAlignedToNorth = false;
uint8_t currentPosition = 0;
unsigned long lastMeasurementTime = 0;
int leftMotorPower = 0;
int rightMotorPower = 0;
unsigned long buzzerStartTime = 0;
bool shouldBeep = false;

// =============================================
// Function Forward Declarations
// =============================================
void controlMotors(int leftPower, int rightPower);
void fastAlignToNorth();
void fastMoveToNextPosition();
float readUltrasonic();

// =============================================
// BLE Callback Classes
// =============================================

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

class MotorCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        if (pCharacteristic == nullptr) return;
        
        uint8_t* data = pCharacteristic->getData();
        if (data == nullptr) return;
        
        size_t length = pCharacteristic->getLength();
        if (length < 2) return;  // Need exactly 2 bytes (left and right)
        
        // Parse motor commands with safety checks
        leftMotorPower = static_cast<int8_t>(data[0]);
        rightMotorPower = static_cast<int8_t>(data[1]);
        
        // Constrain values to -100 to 100 range
        leftMotorPower = constrain(leftMotorPower, -100, 100);
        rightMotorPower = constrain(rightMotorPower, -100, 100);
        
        controlMotors(leftMotorPower, rightMotorPower);
    }
};

// =============================================
// Motor Control Functions
// =============================================

void controlMotors(int leftPower, int rightPower) {
    // Apply 17.6% reduction to left motor (multiply by 0.824)
    int adjustedLeftPower = static_cast<int>(leftPower * 0.824f);

    // Left motor control (slower by 17.6%)
    digitalWrite(MOTOR_L_A, adjustedLeftPower > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_L_B, adjustedLeftPower < 0 ? HIGH : LOW);

    // Right motor control (unchanged)
    digitalWrite(MOTOR_R_A, rightPower > 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_B, rightPower < 0 ? HIGH : LOW);
}

// =============================================
// Main Setup Function
// =============================================

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize I2C and Compass
    Wire.begin(SDA_PIN, SCL_PIN);
    
    if (!compass.init()) {
        Serial.println("ERROR: LSM303DLHC compass not detected!");
        while (1);
    }
    compass.enableDefault();

    // Initialize GPIO Pins
    pinMode(MOTOR_L_A, OUTPUT);
    pinMode(MOTOR_L_B, OUTPUT);
    pinMode(MOTOR_R_A, OUTPUT);
    pinMode(MOTOR_R_B, OUTPUT);
    digitalWrite(MOTOR_L_A, LOW);
    digitalWrite(MOTOR_L_B, LOW);
    digitalWrite(MOTOR_R_A, LOW);
    digitalWrite(MOTOR_R_B, LOW);
    
    pinMode(STEPPER_IN1, OUTPUT);
    pinMode(STEPPER_IN2, OUTPUT);
    pinMode(STEPPER_IN3, OUTPUT);
    pinMode(STEPPER_IN4, OUTPUT);
    
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    // Configure Stepper Motor
    stepper.setSpeed(MAX_SPEED_RPM);

    // Initialize BLE
    BLEDevice::init("RadarCar");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE Service and Characteristics
    BLEService *pService = pServer->createService(SERVICE_UUID_CONTROL);
    
    pMotorCharacteristic = pService->createCharacteristic(
        CHAR_UUID_MOTOR,
        BLECharacteristic::PROPERTY_WRITE
    );
    pMotorCharacteristic->setCallbacks(new MotorCharacteristicCallbacks());
    
    pDistanceCharacteristic = pService->createCharacteristic(
        CHAR_UUID_DISTANCE,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pDistanceCharacteristic->addDescriptor(new BLE2902());
    
    pService->start();
    
    // Start BLE Advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID_CONTROL);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE Initialized - Waiting for connection...");
}

// =============================================
// Main Loop Function
// =============================================

void loop() {
    unsigned long currentTime = millis();

    if (!isAlignedToNorth) {
        fastAlignToNorth();
    } else {
        if (currentTime - lastMeasurementTime > 20) {
            lastMeasurementTime = currentTime;
            float distance = readUltrasonic();

            // Send distance data via BLE
            if (deviceConnected) {
                String data = String(distance) + "," + String(currentPosition);
                pDistanceCharacteristic->setValue(data.c_str());
                pDistanceCharacteristic->notify();
            }

            fastMoveToNextPosition();
        }
    }

    // Handle BLE Connection State
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);  // Give the stack time to get ready
        pServer->startAdvertising();
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}

// =============================================
// Navigation Functions
// =============================================

void fastAlignToNorth() {
    compass.read();
    float heading = compass.heading();

    // Check if within ±2.5° of target (175° = North in misaligned system)
    if (heading >= 172.5 && heading <= 177.5) {
        if (!isAlignedToNorth) {
            // First time achieving alignment
            isAlignedToNorth = true;
            currentPosition = 0;
            Serial.println("Aligned to North (155° raw). Ready for scan.");
            digitalWrite(BUZZER_PIN, HIGH);  // Start beep
            buzzerStartTime = millis();      // Record start time
            shouldBeep = true;               // Flag that we're beeping
        }
    } else {
        // Not aligned - keep searching
        stepper.step(-5);
        delay(5);
        isAlignedToNorth = false;
    }

    // Handle buzzer timeout - turn off after 200ms
    if (shouldBeep && (millis() - buzzerStartTime >= 200)) {
        digitalWrite(BUZZER_PIN, LOW);
        shouldBeep = false;
    }
}

void fastMoveToNextPosition() {
    stepper.step(STEPS_PER_POSITION);
    delay(5);
    currentPosition = (currentPosition + 1) % POSITION_COUNT;
}

// =============================================
// Sensor Functions
// =============================================

float readUltrasonic() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, MAX_DISTANCE_CM * 29);
    return (duration == 0) ? MAX_DISTANCE_CM : duration * 0.01715;
}