/*  PID Example with SVK Tuner mobile app
*   PID code for SVK Cheetah robot, that uses the SVK Tuner app for start/stop and editing PID parameters
*/

#include <SVKCheetah.h>
#include <SVKTunerApp.h>
#include <SoftwareSerial.h>

// Uncomment for Debug monitor mode
#define SVKTUNER_DEBUG

#define BT_RX 5
#define BT_TX 6
#define MAX_INTEGRAL 1100
#define MAX_SPEED 110

SoftwareSerial bluetoothSerial(BT_RX, BT_TX);
SVKTunerApp tuner(bluetoothSerial);

IRSensorsCheetah irSensors;

const uint8_t sensorCount = 15;
const uint8_t muxPins[5] = { 2, 4, 7, A2, A3};

uint16_t sensorValues[sensorCount];

// PID constants
float Kp = 0.048;
float Ki = 0.0000;
float Kd = 1;

// Motor Pins
const uint8_t PWMA = 3;
const uint8_t PWMB = 11;
const uint8_t DIRA = 13;
const uint8_t DIRB = A1;

// PID variables
float lastError = 0;
float integral = 0;

// Motor Speed variables
int baseSpeed = 45;
int leftSpeed = 0;
int rightSpeed = 0;

bool robotRunning = false;

void runPIDloop() {
  float position = irSensors.readLineBlack(sensorValues);
  float error = 7000 - position;

  float derivative = error - lastError;
  lastError = error;

  integral += error;
  integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);


  float output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, -90, 90);

  if (position <= 1000) {
      // Hard right turn
      leftSpeed = MAX_SPEED;
      rightSpeed = 0;
  } else if (position >= 13000) {
      // Hard left turn
      leftSpeed = 0;
      rightSpeed = MAX_SPEED;
  } else {
      // Normal PID
      leftSpeed = baseSpeed + output;
      rightSpeed = baseSpeed - output;
  }

  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  analogWrite(PWMA, leftSpeed);
  analogWrite(PWMB, rightSpeed);

  delayMicroseconds(1200);
}

void calibrateRobot() {
    // Runs calibration method 100 times in order for the robot to correctly calibrate on black line values
    for(uint16_t i = 0; i < 100; i++) irSensors.calibrate();

    #ifdef SVKTUNER_DEBUG
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
    #endif
}

void stopRobot() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void setup()
{
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  #ifdef SVKTUNER_DEBUG
    while (!Serial);
    Serial.println(F("=== Bluetooth Debug Monitor ==="));
  #endif

  // Set motor directions
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);
  digitalWrite(DIRA, LOW); // Set left motor direction
  digitalWrite(DIRB, LOW); // Set right motor direction
  
  irSensors.setMultiplexerPins(muxPins);
  // Sets amount of times each sensor is read in a single loop
  irSensors.setSamplesPerSensor(1);

  delay(1000);

  #ifdef SVKTUNER_DEBUG
    Serial.println("Waiting for Bluetooth signal...");
  #endif

  calibrateRobot();
}

void loop() {
  tuner.setRobotMode(tuner.processNextMessage());
  
  if(tuner.getRobotMode() == RUN_MODE)  {
    tuner.processStartStopCommands(); 
    if(tuner.getRobotState() == RUNNING) robotRunning = true;
    else if(tuner.getRobotState() == STOPPED) robotRunning = false;

    if(robotRunning)  {
      runPIDloop();
    } else {
      stopRobot();
    }
  }
  else if(tuner.getRobotMode() == TUNE_MODE)  {
    tuner.processBluetoothData();
    #ifdef SVKTUNER_DEBUG
      tuner.logAllParameters();
    #endif
  }
}
