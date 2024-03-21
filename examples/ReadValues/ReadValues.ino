#include <SVKCheetah.h>

/***
 * This is an example code for reading Calibrated Values from the SVKLine Cheetah created by
 * SVKRobotics. This robot controls and reads the IR Sensors using 2 8-1 Multiplexers, that we use
 * 3 Signal Digital Pins to control what sensor is to be read, and then 2 Multiplexer Outputs to
 * read the analog value of each sensor, half from Multiplexer 1 and half from Multiplexer 2.
 * 
 * 
 * In the setup the Arduino LED will turn on while the calibration is happening, and after the
 * robot has calibrated on the black line, the Arduino LED will turn off.
 * 
 * Inside the Main loop the program will read the values of the black line while using the calibrated
 * values that the robot previously saved. The sensors will print to the serial monitor the values it's
 * currently seeing, which will be a number from 0 (maximum reflectance) to 1000(minimum reflectance) for
 * each sensor. That means 0 for white background and 1000 for the center of the black line (without taking into account any noise created).
 * 
 * The whole sensor array now reads from 0 to 14000,where 0 means line is under sensor 0, 1000 is line is under sensor 1, 2000 is line under sensor 2, etc...
 * 
*/


IRSensorsCheetah irSensors;


const uint8_t sensorCount = 15;
uint16_t sensorValues[sensorCount];


void setup()
{
    irSensors.setMultiplexerPins((const uint8_t[]) {2, 4, 7, A2, A3});

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Turns calibration on
    irSensors.setCalibrationMode(true);

    // Runs calibration method 200 times in order for the robot to correctly calibrate on black line values
    for(uint16_t i = 0; i < 200; i++)
    {
        irSensors.calibrate();
    }

    digitalWrite(LED_BUILTIN, LOW);

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
    delay(1000);
}


void loop()
{
    // read calibrated sensors values and get position of black line from 0 to 7000 (8 sensors)
    uint16_t position = irSensors.readLineBlack(sensorValues);


    // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // position
    Serial.println("Sensor array values: \n");
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
        Serial.println();
    }
    Serial.println("Line position: ");
    Serial.println(position);
    
    delay(1000);
}
