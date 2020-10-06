#include <SBUS.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

#define DEBUG_SERIAL  Serial
#define SBUS_SERIAL   Serial2
#define ODRIVE_SERIAL Serial3

#define DEBUG_SERIAL_BAUDRATE 115200
#define ODRIVE_SERIAL_BAUDRATE 115200

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

// Serial to the ODrive
//SoftwareSerial odrive_serial(8, 9); //RX (ODrive TX), TX (ODrive RX)
// Note: you must also connect GND on ODrive to GND on Arduino!

// ODrive object
ODriveArduino odrive(ODRIVE_SERIAL);
SBUS x8r(SBUS_SERIAL);

float channels[16];
bool failSafe;
bool lostFrame;
double target_wheel_rpm;
bool motor1_calibration_finish;

void setup() {
  motor1_calibration_finish = false;
  x8r.begin();
  // ODrive uses 115200 baud
  ODRIVE_SERIAL.begin(ODRIVE_SERIAL_BAUDRATE);

  // Serial to PC
//  DEBUG_SERIAL.b/egin(DEBUG_SERIAL_BAUDRATE);
//  while (!DEBUG_SERIAL) ; // wait for A/rduino Serial Monitor to open

//  DEBUG_SERIAL.println("ODriveArduino");
//  DEBUG_SERIAL.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    ODRIVE_SERIAL << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    ODRIVE_SERIAL << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

//  DEBUG_SERIAL.println("Ready!");
//  DEBUG_SERIAL.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
//  DEBUG_SERIAL.println("Send the character 's' to stop");
//  DEBUG_SERIAL.println("Send the character 'g' to go");
//  DEBUG_SERIAL.println("Send the character 'b' to read bus voltage");
//  DEBUG_SERIAL.println("Send the character 'p' to read motor positions in a 10s loop");

  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(0, requested_state, false); // don't wait
}

void loop() {
  if (x8r.readCal(&channels[0], &failSafe, &lostFrame)) {
    target_wheel_rpm = (*(channels + 1) * 150) - 4.3;
//    DEBUG_SERIAL << target_wheel_rpm/ << '\n';
    odrive.SetVelocity(0, target_wheel_rpm);
  }
  //  if (motor1_calibration_finish) {
  //    DEBUG_SERIAL.println("Executing test move");
  //    odrive.SetVelocity(0, target_wheel_rpm);
  //    delay(100);
  //  }
  //  if (DEBUG_SERIAL.available()) {
  //    char c = DEBUG_SERIAL.read();
  //
  //    // Run calibration sequence
  //    if (c == '0' || c == '1') {
  //      int motornum = c - '0';
  //      int requested_state;
  //
  //      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  //      DEBUG_SERIAL << "Axis" << c << ": Requesting state " << requested_state << '\n';
  //      odrive.run_state(motornum, requested_state, true);
  //
  //      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  //      DEBUG_SERIAL << "  Axis" << c << ": Requesting state " << requested_state << '\n';
  //      odrive.run_state(motornum, requested_state, true);
  //
  //      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  //      DEBUG_SERIAL << "Axis" << c << ": Requesting state " << requested_state << '\n';
  //      odrive.run_state(motornum, requested_state, false); // don't wait
  //      delay(100);
  //      motor1_calibration_finish = true;
  //    }
  //
  //    // Read bus voltage
  //    if (c == 'b') {
  //      ODRIVE_SERIAL << "r vbus_voltage\n";
  //      DEBUG_SERIAL << "Vbus voltage: " << odrive.readFloat() << '\n';
  //    }
  //
  //    // print motor positions in a 10s loop
  //    if (c == 'p') {
  //      static const unsigned long duration = 10000;
  //      unsigned long start = millis();
  //      while (millis() - start < duration) {
  //        for (int motor = 0; motor < 2; ++motor) {
  //          ODRIVE_SERIAL << "r axis" << motor << ".encoder.pos_estimate\n";
  //          DEBUG_SERIAL << odrive.readFloat() << '\t';
  //        }
  //        DEBUG_SERIAL << '\n';
  //      }
  //    }
  //  }
  delay(2);
}
