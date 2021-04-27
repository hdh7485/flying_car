#include <math.h>
#include <SBUS.h>
#include <ODriveArduino.h>
#include <ros.h>
#include "ros_lib/ackermann_msgs/AckermannDriveStamped.h"
#include "ros_lib/geometry_msgs/Twist.h"
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
#define SBUS_SERIAL   Serial2
#define ODRIVE_SERIAL Serial3 

//#define DEBUG_SERIAL_BAUDRATE 115200
#define ODRIVE_SERIAL_BAUDRATE 115200

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

ODriveArduino odrive(ODRIVE_SERIAL);
Steering steering(Serial1, 115200, 2, 2.0, 2, 1);
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
  
  ODRIVE_SERIAL.begin(ODRIVE_SERIAL_BAUDRATE);
  for (int axis = 0; axis < 2; ++axis) {
    ODRIVE_SERIAL << "w axis" << axis << ".controller.config.vel_limit " << 1000.0f << '\n';
    ODRIVE_SERIAL << "w axis" << axis << ".motor.config.current_lim " << 50.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }
  //  DEBUG_SERIAL.println("ODriveArduino");

  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  //  DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
  //  DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
  odrive.run_state(0, requested_state, false); // don't wait
  odrive.run_state(1, requested_state, false); // don't wait

  x8r.begin();
}

void loop() {
  if (x8r.readCal(&channels[0], &failSafe, &lostFrame)) {
    nh.spinOnce();
    if (*(channels + 5) > 0) {
      target_steering_degree = ack_steer;
      target_wheel_rpm = ack_throt;
      if (target_wheel_rpm < 2.0 && target_wheel_rpm > -2.0) target_wheel_rpm = 0.0;

      ackermann_geometry.calculate(target_steering_degree, target_wheel_rpm);
      odrive.SetVelocity(0, target_wheel_rpm);
      odrive.SetVelocity(1, -target_wheel_rpm);
      steering.rotateAckermannAngle(ackermann_geometry);

      //      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      //      DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      //      DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      //      odrive.run_state(0, requested_state, false);
      //      odrive.run_state(1, requested_state, true);
      //
      //      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      //      DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      //      DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      //      odrive.run_state(0, requested_state, false);
      //      odrive.run_state(1, requested_state, true);

      //requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      //      DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      //      DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      //      odrive.run_state(0, requested_state, false); // don't wait
      //      odrive.run_state(1, requested_state, false); // don't wait
      //      delay(100);
    }
    else {
      target_steering_degree = *(channels + 0) * -40.0 + STEERING_BIAS;
      if (target_steering_degree < 2.0 && target_steering_degree > -2.0) target_steering_degree = 0.0;

      target_wheel_rpm = (*(channels + 1) * 150) + THROTTLE_BIAS;
      if (target_wheel_rpm < 2.0 && target_wheel_rpm > -2.0) target_wheel_rpm = 0.0;

      ackermann_geometry.calculate(target_steering_degree, target_wheel_rpm);

      //      DEBUG_SERIAL << "target Angle: " << target_steering_degree << '\n';
      //      DEBUG_SERIAL << "Left Angle  : " << ackermann_geometry.left_steer_degree << '\n';
      //      DEBUG_SERIAL << "Right Angle : " << ackermann_geometry.right_steer_degree << "\n\n";
      //
      //      DEBUG_SERIAL << "target RPM: " << target_wheel_rpm << '\n';
      //      DEBUG_SERIAL << "Left RPM  : " << ackermann_geometry.left_rear_rpm << '\n';
      //      DEBUG_SERIAL << "Right RPM : " << ackermann_geometry.right_rear_rpm << "\n\n";

      odrive.SetVelocity(0, target_wheel_rpm);
      odrive.SetVelocity(1, -target_wheel_rpm);
      steering.rotateAckermannAngle(ackermann_geometry);
    }
  }
  delay(10);
}
