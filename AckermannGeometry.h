#ifndef ACKERMANN_GEOMETRY_H
#define ACKERMANN_GEOMETRY_H
#include "Arduino.h"

class AckermannGeometry {
  private:
    const double  WHEEL_FRONT_WIDTH = 0.48; //m
    const double  WHEEL_REAR_WIDTH = 0.53; //m
    const double  WHEEL_VERTICAL_DISTANCE = 1.44; //m
    const double  WHEEL_RADIUS = 0.127; //m

  public:
    double left_steer_degree;
    double right_steer_degree;
    double left_rear_rpm;
    double right_rear_rpm;

    void calculate(double steering_angle, double rpm);
};

#endif
