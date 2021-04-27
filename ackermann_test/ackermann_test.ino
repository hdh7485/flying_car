#include <math.h>
#include <SBUS.h>
#include <ODriveArduino.h>
#include <ros.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/Twist.h"
#include "src/AckermannGeometry.h"
#include "src/Steering.h"
#include "src/Throttle.h"

#define SBUS_SERIAL   Serial1
#define STEERING_BIAS 1.5
#define THROTTLE_BIAS -6.5

Steering steering(Serial2, 115200, 23, 2.0, 2, 1);
Throttle throttle(Serial3, 115200, 0, 1);
SBUS x8r(SBUS_SERIAL);
AckermannGeometry ackermann_geometry;

float channels[16];
bool failSafe;
bool lostFrame;

float target_steering_degree;
double target_wheel_rpm;

int requested_state;

void setup() {
  x8r.begin();
}

void loop() {
  if (x8r.readCal(&channels[0], &failSafe, &lostFrame)) {
    target_steering_degree = *(channels + 0) * 40.0 + STEERING_BIAS;
    if (target_steering_degree < 2.0 && target_steering_degree > -2.0) target_steering_degree = 0.0;
    Serial.println(target_steering_degree);
    
    target_wheel_rpm = (*(channels + 1) * -150) - THROTTLE_BIAS;
    if (target_wheel_rpm < 2.0 && target_wheel_rpm > -2.0) target_wheel_rpm = 0.0;
    Serial.println(target_wheel_rpm);
    
    ackermann_geometry.calculate(target_steering_degree, target_wheel_rpm);
    throttle.rotateAckermannVelocity(ackermann_geometry);
    steering.rotateAckermannAngle(ackermann_geometry);
    
    delay(5);
  }
}
