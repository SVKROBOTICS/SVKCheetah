/// Header file for reading IR Follow Line Sensors using multiplexer in SVK Robotics Line Follow Robot
#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <Servo.h>

/// @brief Class for reading IR values using Multiplexer
class IRSensorsCheetah {
    public:
        /// @brief Class Constructor
        IRSensorsCheetah() = default;

        /// @brief Destructor
        ~IRSensorsCheetah();

        /*** @brief Sets Multiplexers 3 Digital Signal pins and Multiplexers 2 Analog Output pin.
            IMPORTANT: PINS SHOULD BE INPUTTED IN THIS SPECIFIC ORDER
            S0 -> S1 -> S2-> muxOutput
            FOR EXAMPLE:   _muxPins = {5(Signal0),6(Signal1),7(Signal2),A0(OUTPUT)}
            @param pins Multiplexer pins that should be inserted with correct order (SEE ABOVE)
        ***/
        void setMultiplexerPins(const uint8_t *pins);


        /// @brief Sets number of analog readings to average per analog sensor
        /// @param samples Number of samples
        void setSamplesPerSensor(uint8_t samples);

        /// @brief Returns number of analog samples per sensor
        /// @return Number of samples
        uint8_t getSamplesPerSensor() { return _samplesPerSensor; };

        /**
         * @brief Sets if user wants the robot to calibrate before running or not
         * 
         * @param calibrationMode Bool variable for: 
         *                       - true for calibration on
         *                       - false for calibration off
         */
        void setCalibrationMode(bool calibrationMode) { _calibrateOn = calibrationMode; };

        /// @brief Gets calibration mode (on or off)
        /// @return Bool true or false
        bool getCalibrationMode() { return _calibrateOn; }

        /// @brief Stores sensor calibration data
        struct CalibrationData
        {
            /// @brief checks whether array pointers have been allocated and initialized
            bool initialized = false;
            /// @brief Minimum reading read during calibration
            uint16_t *minimum = nullptr;
            /// @brief Maximum reading read during calibration
            uint16_t *maximum = nullptr;
        };

        /// @brief Calibration data
        CalibrationData _calibration;

        /// @brief Calibrates Robot to black line values
        void calibrate();

        /// @brief Resets Robot Calibration values (if there are any values)
        void resetCalibration();

        /// @brief Reads IR sensor array raw analog values
        /// @param _sensorValues values of ir sensors
        void read(uint16_t *sensorValues);

        /// @brief Reads IR sensors and gives calibrated values from 0 to 1000
        /// @param sensorValues Calibrated sensor values
        void readCalibrated(uint16_t *sensorValues);

        /// @brief Reads if robot is seeing the black line
        /// @param _sensorValues IR Sensor values
        /// @return returns number representing black line
        uint16_t readLineBlack(uint16_t* sensorValues);

    private:
        // Amount of IR sensors
        const uint8_t _sensorAmount = 15;

        // Multiplexer signal pins and output pin
        uint8_t *_muxPins = nullptr;

        // Array of sensor values
        uint16_t *_sensorValues = nullptr;

        // checks if calibration is on and should the robot calibrate or not
        bool _calibrateOn = true;
        // Sets the amount of samples that should be taken for each sensor. Higher number reduces noise but slows down the process
        uint8_t _samplesPerSensor = 4;
        // checks if turbine should be activated or not
        bool _turbineOn = true;
        // Sets max value readLine should see
        uint16_t _maxValue = 1023;
        // Last known line position
        uint16_t _lastPosition = 0;

        /// Selects channel of multiplexer in order to read sensor values
        void selectChannel(uint8_t channel);

        /// @brief Handles actually calibrating robot
        void calibratePrivate(CalibrationData &calibration);

        /// Reads sensors
        void readPrivate(uint16_t* _sensorValues);

        /// Sees if robot is seeing a line
        uint16_t readLinesPrivate(uint16_t* _sensorValues);
};