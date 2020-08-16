#include <Dynamixel2Arduino.h>
#include "SBUS.h"

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

const uint8_t LEFT_DXL_ID = 1;
const uint8_t RIGHT_DXL_ID = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
SBUS x8r(Serial5);

SBUS x8r(Serial5);
uint16_t channels[16];
bool failSafe;
bool lostFrame;

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(115200);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(LEFT_DXL_ID);
  dxl.ping(RIGHT_DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(LEFT_DXL_ID);
  dxl.torqueOff(RIGHT_DXL_ID);
  dxl.setOperatingMode(LEFT_DXL_ID, OP_VELOCITY);
  dxl.setOperatingMode(RIGHT_DXL_ID, OP_VELOCITY);
  dxl.torqueOn(LEFT_DXL_ID);
  dxl.torqueOn(RIGHT_DXL_ID);

  x8r.begin();
}

void loop() {
  dxl.setGoalVelocity(LEFT_DXL_ID, 200);
  dxl.setGoalVelocity(RIGHT_DXL_ID, 200);
  delay(1000);
  // Print present velocity
  DEBUG_SERIAL.print("Present Velocity(raw) : ");
  DEBUG_SERIAL.println(dxl.getPresentVelocity(LEFT_DXL_ID));
  DEBUG_SERIAL.println(dxl.getPresentVelocity(RIGHTT_DXL_ID));
  delay(1000);

  // Set Goal Velocity using RPM
  dxl.setGoalVelocity(LEFT_DXL_ID, 25.8, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_DXL_ID, 25.8, UNIT_RPM);
  delay(1000);
  DEBUG_SERIAL.print("Present Velocity(rpm) : ");
  DEBUG_SERIAL.println(dxl.getPresentVelocity(LEFT_DXL_ID, UNIT_RPM));
  DEBUG_SERIAL.println(dxl.getPresentVelocity(RIGHT_DXL_ID, UNIT_RPM));
  delay(1000);

  // Set Goal Velocity using percentage (-100.0 [%] ~ 100.0 [%])
  dxl.setGoalVelocity(LEFT_DXL_ID, -10.2, UNIT_PERCENT);
  dxl.setGoalVelocity(RIGHT_DXL_ID, -10.2, UNIT_PERCENT);
  delay(1000);
  DEBUG_SERIAL.print("Present Velocity(ratio) : ");
  DEBUG_SERIAL.println(dxl.getPresentVelocity(LEFT_DXL_ID, UNIT_PERCENT));
  DEBUG_SERIAL.println(dxl.getPresentVelocity(RIGHT_DXL_ID, UNIT_PERCENT));
  delay(1000);
}
