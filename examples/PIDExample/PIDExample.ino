#include <SVKCheetah.h>

#define MAX_INTEGRAL 1400


IRSensorsCheetah cheetah;


const uint8_t sensorCount = 16;
const uint8_t muxPins1[4] = { 7, 4, 2, A7};
const uint8_t muxPins2[4] = { 7, 4, 2, A7};
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
uint16_t sensorValues[sensorCount];


// PID constants
double Kp = 0.1;      // Proportional constant
double Ki = 0.001;    // Integral constant
double Kd = 0.05;     // Derivative constant

// Motor Pins
const uint8_t PWMA = 3;
const uint8_t PMWB = 11;
const uint8_t DIRA = 13;
const uint8_t DIRB = A1;

// PID variables

double lastError = 0;
double integral = 0;


// Motor Speed variables
const int baseSpeed = 100;
int leftSpeed = 0;
int rightSpeed = 0;


void setup()
{
    cheetah.setMultiplexerPins(muxPins1, muxPins2);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(DIRA, OUTPUT);
    pinMode(DIRB, OUTPUT);


    cheetah.setCalibrationMode(true);

        // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensors
    // * 10 reads per calibrate() call = ~32 ms per calibrate() call.
    // Call calibrate() 300 times to make calibration take about 10 seconds.
    for(uint16_t i = 0; i < 200; i++)
    {
        cheetah.calibrate();
    }

    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(9600);

    // Prints minimum and maximum values read by sensors
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(cheetah._calibration.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(cheetah._calibration.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);

}


void loop() {
  // read calibrated sensors values and get position of black line from 0 to 7000 (8 sensors)
  double position = cheetah.readLineBlack(sensorValues);
  double error = 3500 - position; // Assuming the line is at the middle (3500)

  integral += error;
  integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

  double derivative = error - lastError;
  lastError = error;

  double output = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speeds based on PID output
  leftSpeed = baseSpeed + output;
  rightSpeed = baseSpeed - output;


    // Control the motors
    analogWrite(PWMA, leftSpeed); // Left motor speed control
    analogWrite(PMWB, rightSpeed); // Right motor speed control

    // Set motor directions
    digitalWrite(DIRA, leftSpeed > 0 ? HIGH : LOW); // Set left motor direction
    digitalWrite(DIRB, rightSpeed > 0 ? HIGH : LOW); // Set right motor direction

    // Add a small delay to allow motors to adjust
    delay(10);
}
