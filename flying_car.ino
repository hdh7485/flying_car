#include <Dynamixel2Arduino.h>
#include <math.h>
#include <SBUS.h>

#define DEBUG_SERIAL  Serial
#define DXL_SERIAL    Serial1
#define SBUS_SERIAL   Serial2

#define DEBUG_SERIAL_BAUDRATE 115200
#define DXL_SERIAL_BAUDRATE 115200

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
    double left_steer_angle;
    double right_steer_angle;
    double left_rear_rpm;
    double right_rear_rpm;

    void calculate(double steering_angle, double speed) { //steering_angle: radian, speed: m/s
      double R = tan(steering_angle) / WHEEL_VERTICAL_DISTANCE;
      left_steer_angle = atan2(WHEEL_VERTICAL_DISTANCE, (R - WHEEL_FRONT_WIDTH / 2));
      right_steer_angle = atan2(WHEEL_VERTICAL_DISTANCE, (R + WHEEL_FRONT_WIDTH / 2));
      //self.left_rear_rpm =
      //self.right_rear_rpm =
    }

};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
SBUS x8r(SBUS_SERIAL);
AckermannGeometry ackermann_geometry;

float channels[16];
bool failSafe;
bool lostFrame;
float target_steering_degree;

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);

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
    target_steering_degree = *(channels + 0) * 30.0 * M_PI / 180;
    ackermann_geometry.calculate(target_steering_degree, 1.0);
//    DEBUG_SERIAL.println(ackermann_geometry.left_steer_angle);
    DEBUG_SERIAL.println(target_steering_degree*30);
    dxl.setGoalPosition(LEFT_DXL_ID, target_steering_degree*30, UNIT_DEGREE);
    dxl.setGoalPosition(RIGHT_DXL_ID, target_steering_degree*30, UNIT_DEGREE);

    // Print present position in degree value
    //    DEBUG_SERIAL.print("Present Position(degree) : ");
    //    DEBUG_SERIAL.println(dxl.getPresentPosition(LEFT_DXL_ID, UNIT_DEGREE));
    //    DEBUG_SERIAL.println(dxl.getPresentPosition(RIGHT_DXL_ID, UNIT_DEGREE));
  }
}
