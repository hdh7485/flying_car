#include "Steering.h"

Steering::Steering(HardwareSerial& dxl_port, unsigned int dxl_serial_baudrate, int dxl_dir_pin, float dxl_protocol_version, uint8_t left_dxl_id, uint8_t right_dxl_id)
  :_dxl_port(dxl_port), _dxl_serial_baudrate(dxl_serial_baudrate), _dxl_dir_pin(dxl_dir_pin), _dxl_protocol_version(dxl_protocol_version), _left_dxl_id(left_dxl_id), _right_dxl_id(right_dxl_id), 
  _dxl(_dxl_port, _dxl_dir_pin){
//  Dynamixel2Arduino dxl(_dxl_port, _dxl_dir_pin);
  _dxl.begin(_dxl_serial_baudrate);
  _dxl.setPortProtocolVersion(_dxl_protocol_version);
  _dxl.ping(_left_dxl_id);
  _dxl.ping(_right_dxl_id);

  // Turn off torque when configuring items in EEPROM area
  _dxl.torqueOff(_left_dxl_id);
  _dxl.torqueOff(_right_dxl_id);
  _dxl.setOperatingMode(_left_dxl_id, OP_POSITION);
  _dxl.setOperatingMode(_right_dxl_id, OP_POSITION);
  _dxl.torqueOn(_left_dxl_id);
  _dxl.torqueOn(_right_dxl_id);
}

void Steering::rotateAckermannAngle(AckermannGeometry ackermann) {
  _dxl.setGoalPosition(_left_dxl_id, ackermann.left_steer_degree, UNIT_DEGREE);
  _dxl.setGoalPosition(_right_dxl_id, ackermann.right_steer_degree, UNIT_DEGREE);
}

void Steering::rotateDegree(uint8_t dxl_id, float angle) {
  _dxl.setGoalPosition(dxl_id, angle, UNIT_DEGREE);
}

void Steering::rotateRaw(uint8_t dxl_id, float raw) {
  _dxl.setGoalPosition(dxl_id, raw, UNIT_RAW);
}
