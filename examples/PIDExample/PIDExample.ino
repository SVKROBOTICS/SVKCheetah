#include <SVKCheetah.h>

#define USE_SVKTUNER

#ifdef USE_SVKTUNER
  #include <SVKTunerApp.h>
  #include <SoftwareSerial.h>
  #define SVKTUNER_DEBUG
  #define BT_RX 5
  #define BT_TX 6
  SoftwareSerial bluetoothSerial(BT_RX, BT_TX);
  SVKTunerApp tuner(bluetoothSerial);
  bool robotRunning = false;
#endif

#define MAX_INTEGRAL 700
#define MAX_SPEED 100


IRSensorsCheetah irSensors;


const uint8_t sensorCount = 15;
const uint8_t muxPins[5] = { 2, 4, 7, A2, A3};

uint16_t sensorValues[sensorCount];


// PID constants
float Kp = 1.45;      // Proportional constant
float Ki = 0.000;    // Integral constant
float Kd = 2.85;     // Derivative constant

// Motor Pins
const uint8_t PWMA = 3;
const uint8_t PWMB = 11;
const uint8_t DIRA = 13;
const uint8_t DIRB = A1;

// PID variables

float lastError = 0;
float integral = 0;


// Motor Speed variables
const int baseSpeed = 45;
int leftSpeed = 0;
int rightSpeed = 0;


void setup()
{
    irSensors.setMultiplexerPins(muxPins);

    delay(500);

    // Runs calibration method 100 times in order for the robot to correctly calibrate on black line values
    for(uint16_t i = 0; i < 100; i++)
    {
        irSensors.calibrate();
    }

    Serial.begin(9600);

    // Prints minimum and maximum values read by sensors
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(irSensors._calibration.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(irSensors._calibration.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();

    // Sets amount of times each sensor is read in a single loop
    irSensors.setSamplesPerSensor(1);

    #ifdef USE_SVKTUNER
      bluetoothSerial.begin(9600);
      #ifdef SVKTUNER_DEBUG
        while (!Serial);
        Serial.println(F("=== Bluetooth Debug Monitor ==="));
      #endif
    #endif

    // Set motor directions
    pinMode(DIRA, OUTPUT);
    pinMode(DIRB, OUTPUT);
    digitalWrite(DIRA, LOW); // Set left motor direction
    digitalWrite(DIRB, HIGH); // Set right motor direction

    // delay to set robot into starting position
    delay(1000);

}


void loop() {

  // At Start of every loop checks if Bluetooth Start Stop signal was received
  #ifdef USE_SVKTUNER
    #ifdef SVKTUNER_DEBUG
    if (bluetoothSerial.available()) {
        Serial.println(F("[BT] Data detected in buffer..."));
    }
    #endif

    // Process only start and stop commands
    tuner.processStartStopCommands();

    if(tuner.getRobotState() == RUNNING) {
      robotRunning = true;
    }
    else if(tuner.getRobotState() == STOPPED) {
      robotRunning = false;
    }
  #endif

  #ifdef USE_SVKTUNER
  if(robotRunning) {
  #endif
    // read calibrated sensors values and get position of black line from 0 to 14000 (15 sensors)
    float position = irSensors.readLineBlack(sensorValues);
    float error = 7000 - position; // Assuming the line is at the middle (7000)

    integral += error;
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float derivative = error - lastError;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    // Adjust motor speeds based on PID output
    leftSpeed = baseSpeed + output;
    rightSpeed = baseSpeed - output;

    // Ensure motor speeds don't exceed maximum speed limit
    leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

    // Control the motors
    analogWrite(PWMA, leftSpeed); // Left motor speed control
    analogWrite(PWMB, rightSpeed); // Right motor speed control

    // Add a small delay to allow motors to adjust
    delayMicroseconds(100);

  #ifdef USE_SVKTUNER
  }
  else {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  }
  #endif
}
