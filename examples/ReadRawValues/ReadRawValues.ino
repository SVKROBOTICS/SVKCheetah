#include <SVKCheetah.h>

/***
 * This is an example code for reading Raw Values from the SVKLine Cheetah created by
 * SVKRobotics. This robot controls and reads the IR Sensors using 2 8-1 Multiplexers, that we use
 * 3 Signal Digital Pins to control what sensor is to be read, and then 2 Multiplexer Outputs to
 * read the analog value of each sensor, half from Multiplexer 1 and half from Multiplexer 2.
 * 
 * 
 * Inside the Main loop the program will read the values of the black line while using the raw
 * values. The sensors will print to the serial monitor the values it's
 * currently seeing, which will be a number from 0 (maximum reflectance) to 1000(minimum reflectance) for
 * each sensor. That means 0 for white background and 1000 for the center of the black line.
 * 
 * 
*/


IRSensorsCheetah irSensors;


const uint8_t sensorCount = 15;
uint16_t sensorValues[sensorCount];


void setup()
{
    irSensors.setMultiplexerPins((const uint8_t[]) {2, 4, 7, A2, A3});

    Serial.begin(9600);
}


void loop()
{
    irSensors.read(sensorValues);

    // print the sensor values as numbers from 0 to 1023, where 0 means maximum
    // reflectance and 1023 means minimum reflectance
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();

    delay(500);
}