#include <SVKCheetah.h>
#include <SVKTunerApp.h>
#include <SoftwareSerial.h>

// Comment out to disable debug prints
#define SVKTUNER_DEBUG

// Bluetooth module connections
#define BT_RX 3  // Connect to Bluetooth TX
#define BT_TX 2  // Connect to Bluetooth RX

SoftwareSerial bluetoothSerial(BT_RX, BT_TX);
SVKTunerApp tuner(bluetoothSerial);

IRSensorsCheetah irSensors;

const uint8_t sensorCount = 15;
const uint8_t muxPins[5] = { 2, 4, 7, A2, A3};

uint16_t sensorValues[sensorCount];

// Motor Pins
const uint8_t PWMA = 3;
const uint8_t PWMB = 11;
const uint8_t DIRA = 13;
const uint8_t DIRB = A1;


void start() {
    analogWrite(PWMA, 100);
    analogWrite(PWMB, 100);
}

void stop() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

void setup() {
    Serial.begin(9600);
    bluetoothSerial.begin(9600);

    #ifdef SVKTUNER_DEBUG
    while (!Serial);  // Wait for Serial monitor on debug builds
    DEBUG_PRINTLN(F("\n\n=== Bluetooth Debug Monitor ==="));
    DEBUG_PRINTLN(F("System initialized"));
    DEBUG_PRINTLN(F("Waiting for !START! or !STOP!..."));
    #else
    Serial.println(F("Waiting for signals..."));
    #endif

    pinMode(DIRA, OUTPUT);
    pinMode(DIRB, OUTPUT);

    // Set motor directions
    digitalWrite(DIRA, LOW); // Set left motor direction
    digitalWrite(DIRB, LOW); // Set right motor direction

}

void loop() {
    // First check if we're receiving any data at all
    #ifdef SVKTUNER_DEBUG
    if (bluetoothSerial.available()) {
        DEBUG_PRINTLN(F("[BT] Data detected in buffer..."));
    }
    #endif

    // Process only start and stop commands
    tuner.processStartStopCommands();

    if(tuner.getRobotState() == RUNNING) {
        Serial.println("Robot is running");
        start();
    }
    else if(tuner.getRobotState() == STOPPED) {
        Serial.println("Robot is stopped");
        stop();
    }
    else {
        Serial.println("Unknown State");
    }

    #ifdef SVKTUNER_DEBUG
    // Additional debug for connection health
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 2000) {  // Every 2 seconds
        lastDebug = millis();
        DEBUG_PRINTLN(F("[STATUS] System active..."));
    }
    #endif

    delay(10);  // Short delay for stability
}