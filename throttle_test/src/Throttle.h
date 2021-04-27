#ifndef THROTTLE_H
#define THROTTLE_H
#include "Arduino.h"
#include <ODriveArduino.h>
#include "AckermannGeometry.h"

class Throttle{
  private:
    HardwareSerial& _odrive_port;
    const unsigned int _odrive_serial_baudrate;
    const uint8_t _left_wheel;
    const uint8_t _right_wheel;
	ODriveArduino _odrive;
  
  public:
    Throttle(HardwareSerial& odrive_port=Serial2, unsigned int _odrive_serial_baudrate=115200, uint8_t left_wheel=0, uint8_t right_wheel=1);
    void rotateAckermannVelocity(AckermannGeometry ackermann);
};

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

#endif
