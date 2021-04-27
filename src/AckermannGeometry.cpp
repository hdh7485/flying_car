#include "Arduino.h"
#include "AckermannGeometry.h"

void AckermannGeometry::calculate(double steering_angle, double rpm) { //steering_angle: radian, speed: m/s
  double R = WHEEL_VERTICAL_DISTANCE / tan(steering_angle * M_PI / 180.0);
  if (steering_angle < 0.3 && steering_angle > -0.3) {
    left_steer_degree = 0.0;
    right_steer_degree = 0.0;
    left_rear_rpm = rpm;
    right_rear_rpm = rpm;
  }
  else {
    left_rear_rpm = rpm * (R + WHEEL_REAR_WIDTH / 2) / R;
    right_rear_rpm = rpm * (R - WHEEL_REAR_WIDTH / 2) / R;

    left_steer_degree = atan2(WHEEL_VERTICAL_DISTANCE, (R - WHEEL_FRONT_WIDTH / 2)) * 180.0 / M_PI ;
    right_steer_degree = atan2(WHEEL_VERTICAL_DISTANCE, (R + WHEEL_FRONT_WIDTH / 2)) * 180.0 / M_PI;
    if (left_steer_degree > 90) {
      left_steer_degree -= 180;
    }
    if (right_steer_degree > 90) {
      right_steer_degree -= 180;
    }
  }
}
