#ifndef ARDUINO_STEPPER_DRIVER_HPP
#define ARDUINO_STEPPER_DRIVER_HPP

#include <string>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cmath>

class ArduinoStepperDriver
{
public:
    ArduinoStepperDriver(std::string device_name)
        : device_name_(device_name) {}

    int init()
    {
        std::cout << "Initializing Arduino Stepper Driver..." << std::endl;

        try
        {
            serial_port_.Open(device_name_);
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

            std::cout << "Serial port opened successfully!" << std::endl;
            return 0;
        }
        catch (...)
        {
            std::cout << "Failed to open serial port!" << std::endl;
            return -1;
        }
    }

    void activate()
    {
        std::cout << "Stepper driver activated." << std::endl;
    }

    void deactivate()
    {
        std::cout << "Stepper driver deactivated." << std::endl;
    }

    /*
        Convert radians → steps
        Send step targets for 3 motors
    */
    void setTargetPositions(double radian1, double radian2, double radian3)
    {
        if (!serial_port_.IsOpen())
            return;

        const int steps_per_rev = 3200; // 200 steps * 16 microstep
        const double two_pi = 2.0 * M_PI;

        int steps1 = static_cast<int>((radian1 / two_pi) * steps_per_rev);
        int steps2 = static_cast<int>((radian2 / two_pi) * steps_per_rev);
        int steps3 = static_cast<int>((radian3 / two_pi) * steps_per_rev);

        std::string msg =
            std::to_string(steps1) + " " +
            std::to_string(steps2) + " " +
            std::to_string(steps3) + "\n";

        try
        {
            serial_port_.Write(msg);
        }
        catch (...)
        {
            std::cout << "Failed to write to serial!" << std::endl;
        }
    }

    double getPosition(int motor_id)
    {
        // Future: implement encoder feedback
        return 0.0;
    }

    ~ArduinoStepperDriver()
    {
        if (serial_port_.IsOpen())
            serial_port_.Close();
    }

private:
    std::string device_name_;
    LibSerial::SerialPort serial_port_;
};

#endif