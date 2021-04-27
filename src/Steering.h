#ifndef STEERING_H
#define STEERING_H

#include "Arduino.h"
#include "AckermannGeometry.h"
#include <Dynamixel2Arduino.h>

class Steering {
  private:
    HardwareSerial& _dxl_port;
    const unsigned int _dxl_serial_baudrate;
    const uint8_t _dxl_dir_pin;
    const float   _dxl_protocol_version;
    const uint8_t _left_dxl_id;
    const uint8_t _right_dxl_id;
    Dynamixel2Arduino _dxl;

  public:
    Steering(HardwareSerial& dxl_port = Serial1, unsigned int dxl_serial_baudrate = 115200, int dxl_dir_pin = 2, float dxl_protocol_version = 2.0, uint8_t left_dxl_id = 2, uint8_t right_dxl_id = 1);
    void rotateAckermannAngle(AckermannGeometry ackermann);
    void rotateDegree(uint8_t dxl_id, float angle);
    void rotateRaw(uint8_t dxl_id, float raw);
};

#endif
