#ifndef ARDUINO_SERVO_DRIVER_HPP
#define ARDUINO_SERVO_DRIVER_HPP

#include <string>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cmath>

class ArduinoServoDriver {
public:
    ArduinoServoDriver(std::string device_name)
        : device_name_(device_name) {}

    int init() {
        std::cout << "Initializing Arduino Servo Driver..." << std::endl;

        try {
            serial_port_.Open(device_name_);
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            std::cout << "Serial port opened successfully!" << std::endl;
            return 0;
        } catch (...) {
            std::cout << "Failed to open serial port!" << std::endl;
            return -1;
        }
    }

    void activate() {
        std::cout << "Servo driver activated." << std::endl;
    }

    // Deactivate
    void deactivate() {
        std::cout << "Servo driver deactivated." << std::endl;
    }

    // Set target position for 3 servos
void setTargetPositions(double radian1, double radian2, double radian3) {
    if (!serial_port_.IsOpen()) return;

    int deg1 = static_cast<int>(radian1 * (180.0 / 3.14159)+90);
    int deg2 = static_cast<int>(radian2 * (180.0 / 3.14159)+90);
    int deg3 = static_cast<int>(radian3 * (180.0 / 3.14159)+90);

// clamp using if conditions
    if (deg1 < 0) {
        deg1 = 0;
    } else if (deg1 > 180) {
        deg1 = 180;
    }

    if (deg2 < 0) {
        deg2 = 0;
    } else if (deg2 > 180) {
        deg2 = 180;
    }

    if (deg3 < 0) {
        deg3 = 0;
    } else if (deg3 > 180) {
        deg3 = 180;
    }

    // message format
    // std::string msg = std::to_string(deg1) + " " +
    //                   std::to_string(deg2) + " " +
    //                   std::to_string(deg3) + "\n";

    const int steps_per_rev = 3200; // 200 steps * 16 microstep
    const double two_pi = 2.0 * M_PI;

    int steps1 = static_cast<int>((radian1 / two_pi) * steps_per_rev);
    int steps2 = static_cast<int>((radian2 / two_pi) * steps_per_rev);
    int steps3 = static_cast<int>((radian3 / two_pi) * steps_per_rev);
    std::cout << "Sending steps: "
          << steps1 << " "
          << steps2 << " "
          << steps3 << std::endl;
//     std::string msg =
//         std::to_string(1800) + " " +
//         std::to_string(1400) + " " +
//         std::to_string(1300) + "\n";

//     serial_port_.Write(msg);
// }

    std::string msg =
    std::to_string(steps1) +"\n";

serial_port_.Write(msg);
}


    double getPosition(int servo_id) {
      //feedback 
        return 0.0;
    }

    ~ArduinoServoDriver() {
        if (serial_port_.IsOpen())
            serial_port_.Close();
    }

private:
    std::string device_name_;
    LibSerial::SerialPort serial_port_;
};

#endif
