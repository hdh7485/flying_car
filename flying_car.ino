#include <Dynamixel2Arduino.h>
#include <math.h>
#include <SBUS.h>
#include <ODriveArduino.h>

#define DEBUG_SERIAL  Serial
#define DXL_SERIAL    Serial1
#define SBUS_SERIAL   Serial2
#define ODRIVE_SERIAL Serial3

#define DEBUG_SERIAL_BAUDRATE 115200
#define DXL_SERIAL_BAUDRATE 115200
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

const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

const uint8_t LEFT_DXL_ID = 2;
const uint8_t RIGHT_DXL_ID = 1;
const float   DXL_PROTOCOL_VERSION = 2.0;

class AckermannGeometry {
  private:
    const double  WHEEL_FRONT_WIDTH = 1.0; //m
    const double  WHEEL_REAR_WIDTH = 1.0; //m
    const double  WHEEL_VERTICAL_DISTANCE = 2.0; //m
    const double  WHEEL_RADIUS = 0.127; //m

  public:
    double left_steer_degree;
    double right_steer_degree;
    double left_rear_rpm;
    double right_rear_rpm;

    void calculate(double steering_angle, double speed) { //steering_angle: radian, speed: m/s
      if (steering_angle < 0.3 && steering_angle > -0.3) {
        left_steer_degree = 0.0;
        right_steer_degree = 0.0;
      }
      else {
        double R = WHEEL_VERTICAL_DISTANCE / tan(steering_angle * M_PI / 180.0);
        left_steer_degree = atan2(WHEEL_VERTICAL_DISTANCE, (R - WHEEL_FRONT_WIDTH / 2)) * 180.0 / M_PI ;
        right_steer_degree = atan2(WHEEL_VERTICAL_DISTANCE, (R + WHEEL_FRONT_WIDTH / 2)) * 180.0 / M_PI;
        if (left_steer_degree > 90){
          left_steer_degree -= 180;
        }
        if (right_steer_degree > 90){
          right_steer_degree -= 180;
        }
      }
      //self.left_rear_rpm =
      //self.right_rear_rpm =
    }

};

ODriveArduino odrive(ODRIVE_SERIAL);
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
SBUS x8r(SBUS_SERIAL);
AckermannGeometry ackermann_geometry;

float channels[16];
bool failSafe;
bool lostFrame;

float target_steering_degree;
double target_wheel_rpm;
bool motor1_calibration_finish;
int requested_state;

void setup() {
  ODRIVE_SERIAL.begin(ODRIVE_SERIAL_BAUDRATE);

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
  DEBUG_SERIAL.println("ODriveArduino");
  DEBUG_SERIAL.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    ODRIVE_SERIAL << "w axis" << axis << ".controller.config.vel_limit " << 1000.0f << '\n';
    ODRIVE_SERIAL << "w axis" << axis << ".motor.config.current_lim " << 50.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
  DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
  odrive.run_state(0, requested_state, false); // don't wait
  odrive.run_state(1, requested_state, false); // don't wait

  dxl.begin(DXL_SERIAL_BAUDRATE);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(LEFT_DXL_ID);
  dxl.ping(RIGHT_DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(LEFT_DXL_ID);
  dxl.torqueOff(RIGHT_DXL_ID);
  dxl.setOperatingMode(LEFT_DXL_ID, OP_POSITION);
  dxl.setOperatingMode(RIGHT_DXL_ID, OP_POSITION);
  dxl.torqueOn(LEFT_DXL_ID);
  dxl.torqueOn(RIGHT_DXL_ID);

  x8r.begin();
}

void loop() {
  if (x8r.readCal(&channels[0], &failSafe, &lostFrame)) {
//    target_steering_degree = *(channels + 0) * 30.0 * M_PI / 180;
    target_steering_degree = *(channels + 0) * -30.0;
    ackermann_geometry.calculate(target_steering_degree, 1);
    target_wheel_rpm = (*(channels + 1) * 150) - 4.3;
  }
  if (motor1_calibration_finish) {
//    DEBUG_SERIAL.println("Executing test move");
    odrive.SetVelocity(0, target_wheel_rpm);
    odrive.SetVelocity(1, -target_wheel_rpm);
    dxl.setGoalPosition(LEFT_DXL_ID, target_steering_degree, UNIT_DEGREE);
    dxl.setGoalPosition(RIGHT_DXL_ID, target_steering_degree, UNIT_DEGREE);
    DEBUG_SERIAL.println(target_steering_degree);
//    DEBUG_SERIAL.println("Executing test move");
  }
//  dxl.setGoalPosition(LEFT_DXL_ID, target_steering_degree, UNIT_DEGREE);
//  dxl.setGoalPosition(RIGHT_DXL_ID, target_steering_degree, UNIT_DEGREE);

  if (DEBUG_SERIAL.available()) {
    char c = DEBUG_SERIAL.read();

    // Run calibration sequence
    if (c == '0' || c == '1') {
      int requested_state;
      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      odrive.run_state(0, requested_state, false);
      odrive.run_state(1, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      odrive.run_state(0, requested_state, false);
      odrive.run_state(1, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      DEBUG_SERIAL << "Axis" << '0' << ": Requesting state " << requested_state << '\n';
      DEBUG_SERIAL << "Axis" << '1' << ": Requesting state " << requested_state << '\n';
      odrive.run_state(0, requested_state, false); // don't wait
      odrive.run_state(1, requested_state, false); // don't wait
      delay(100);
    }
    else {
      target_steering_degree = *(channels + 0) * -30.0 + STEERING_BIAS;
      ackermann_geometry.calculate(target_steering_degree, 1);
      DEBUG_SERIAL << "target Angle: " << target_steering_degree << '\n';
      DEBUG_SERIAL << "Left Angle  : " << ackermann_geometry.left_steer_degree << '\n';
      DEBUG_SERIAL << "Right Angle : " << ackermann_geometry.right_steer_degree << '\n';
      target_wheel_rpm = (*(channels + 1) * 150) + THROTTLE_BIAS;
      odrive.SetVelocity(0, target_wheel_rpm);
      odrive.SetVelocity(1, -target_wheel_rpm);
      dxl.setGoalPosition(LEFT_DXL_ID, ackermann_geometry.left_steer_degree, UNIT_DEGREE);
      dxl.setGoalPosition(RIGHT_DXL_ID, ackermann_geometry.right_steer_degree, UNIT_DEGREE);
    }
  }
  delay(10);
}
