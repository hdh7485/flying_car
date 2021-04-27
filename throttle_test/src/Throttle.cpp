#include "Throttle.h"

Throttle::Throttle(HardwareSerial& odrive_port, unsigned int odrive_serial_baudrate, uint8_t left_wheel, uint8_t right_wheel): _odrive_port(odrive_port), _odrive_serial_baudrate(odrive_serial_baudrate), _left_wheel(left_wheel), _right_wheel(right_wheel), _odrive(_odrive_port){
  _odrive_port.begin(_odrive_serial_baudrate);
  for (int axis = 0; axis < 2; ++axis) {
    _odrive_port << "w axis" << axis << ".controller.config.vel_limit " << 1000.0f << '\n';
    _odrive_port << "w axis" << axis << ".motor.config.current_lim " << 50.0f << '\n';
  }
  int requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  _odrive.run_state(0, requested_state, false); // don't wait
  _odrive.run_state(1, requested_state, false); // don't wait
} 

void Throttle::rotateAckermannVelocity(AckermannGeometry ackermann){
  _odrive.SetVelocity(0, ackermann.left_rear_rpm);
  _odrive.SetVelocity(1, -ackermann.right_rear_rpm);
}

