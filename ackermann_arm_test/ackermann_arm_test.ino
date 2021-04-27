#include <math.h>
#include <SBUS.h>
#include <ODriveArduino.h>
#include <ros.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/Twist.h"
#include "src/AckermannGeometry.h"
#include "src/Steering.h"
#include "src/Throttle.h"
#include "src/RotorArm.h"

#define SBUS_SERIAL   Serial1
#define STEERING_BIAS 1.5
#define THROTTLE_BIAS -6.5

AckermannGeometry ackermann_geometry;
Steering steering(Serial2, 115200, 23, 2.0, 2, 1);
Throttle throttle(Serial3, 115200, 0, 1);
RotorArm arm_FR(11, 12, 2,  14, 24);
RotorArm arm_RR(26, 25, 5,  14, 27);
RotorArm arm_FL(18, 19, 6,  14, 28);
RotorArm arm_RL(15, 16, 29, 14, 17);

SBUS x8r(SBUS_SERIAL);

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

    Serial.println(*(channels+11));
    Serial.println(arm_FR.isUnfoldSWPushed());
    Serial.println(arm_RR.isUnfoldSWPushed());
    if (*(channels+11) > 0.5) {     // Folding Tx
      arm_FR.fold();
      arm_RR.fold();
      arm_FL.fold();
      arm_RL.fold();
    }
    else if (*(channels+11) < -0.5) { //Unfolding Tx
      arm_FR.unfold();
      arm_RR.unfold();
      arm_FL.unfold();
      arm_RL.unfold();
    }
    else {
      arm_FR.stopMotor();
      arm_RR.stopMotor();
      arm_FL.stopMotor();
      arm_RL.stopMotor();
    }

    delay(5);
  }
}
