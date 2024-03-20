#include <SVKCheetah.h>

#define MAX_INTEGRAL 700
#define MAX_SPEED 100


// We create the class object
IRSensorsCheetah irSensors;

// Set pin amount and pins
const uint8_t sensorCount = 15;
const uint8_t muxPins[5] = { 2, 4, 7, A2, A3};

// create array for storing IR values
uint16_t sensorValues[sensorCount];


// PID constants
float Kp = 0.485;      // Proportional constant
// float Ki = 0.001;    // Integral constant
float Kd = 4.25;     // Derivative constant

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

    pinMode(DIRA, OUTPUT);
    pinMode(DIRB, OUTPUT);

    // Sets samples taken per sensor
    irSensors.setSamplesPerSensor(1);

    // Runs calibration for 100 times in order to give enough time for
    // correct calibration to complete
    for(uint16_t i = 0; i < 200; i++)
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

    // 2.5 second delay after calibration in order for user to place robot in starting position
    delay(2500);

}


void loop() {

  // read calibrated sensors values and get position of black line from 0 to 14000 (15 sensors)
  float position = irSensors.readLineBlack(sensorValues);
  float error = 7000 - position; // Assuming the line is at the middle (7000)

//   integral += error;
//   integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

  float derivative = error - lastError;
  lastError = error;

  float output = Kp * error + Kd * derivative;

  // Adjust motor speeds based on PID output
  leftSpeed = baseSpeed + output;
  rightSpeed = baseSpeed - output;

  // Ensure motor speeds don't exceed maximum speed limit
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);


  // Control the motors
  analogWrite(PWMA, leftSpeed); // Left motor speed control
  analogWrite(PWMB, rightSpeed); // Right motor speed control

  // Set motor directions
  digitalWrite(DIRA, leftSpeed > 0 ? LOW : HIGH); // Set left motor direction
  digitalWrite(DIRB, rightSpeed > 0 ? LOW : HIGH); // Set right motor direction


  // Add a small delay to allow motors to adjust
  delayMicroseconds(100);

}
