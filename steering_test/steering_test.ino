#include <math.h>
#include <SBUS.h>
#include <ODriveArduino.h>
#include <ros.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/Twist.h"
#include "src/AckermannGeometry.h"
#include "src/Steering.h"

ros::NodeHandle nh;

double ack_steer =  0.0;
double ack_throt =  0.0;

void ackCb(const ackermann_msgs::AckermannDriveStamped& ack_msg) {
  ack_steer = ack_msg.drive.steering_angle;
  ack_throt = ack_msg.drive.speed / 3.6;
}

ros::Subscriber<ackermann_msgs::AckermannDriveStamped> sub("/Ackermann/command/joy", &ackCb );
//ros::Subscriber<geometry_msgs::Twist> sub("ackermann_msgs", &ackCb );

//#define DEBUG_SERIAL  Serial
#define SBUS_SERIAL   Serial1

#define STEERING_BIAS 1.5
#define THROTTLE_BIAS -4.3

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

Steering steering(Serial2, 115200, 23, 2.0, 2, 1);
SBUS x8r(SBUS_SERIAL);
AckermannGeometry ackermann_geometry;

float channels[16];
bool failSafe;
bool lostFrame;

float target_steering_degree;
double target_wheel_rpm;

int requested_state;

void setup() {
  //  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
  //  DEBUG_SERIAL.println("Setting parameters...");

  nh.initNode();
  nh.subscribe(sub);
  x8r.begin();
}

void loop() {
  if (x8r.readCal(&channels[0], &failSafe, &lostFrame)) {
    target_steering_degree = *(channels + 0) * 40.0 + STEERING_BIAS;
    if (target_steering_degree < 2.0 && target_steering_degree > -2.0) target_steering_degree = 0.0;
    Serial.println(target_steering_degree);

    target_wheel_rpm = (*(channels + 1) * 150) + THROTTLE_BIAS;
    if (target_wheel_rpm < 2.0 && target_wheel_rpm > -2.0) target_wheel_rpm = 0.0;

    ackermann_geometry.calculate(target_steering_degree, target_wheel_rpm);
    steering.rotateAckermannAngle(ackermann_geometry);
    
    delay(5);
  }
}
