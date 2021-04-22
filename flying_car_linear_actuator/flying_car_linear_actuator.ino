#define LED 13
#define FOLD_SW 14
#define UNFOLD_SW 15
#define EN1 9
#define EN2 10
#define PWM1 11

#include "SBUS.h"
SBUS futaba(Serial1);
uint16_t channels[16];
bool failSafe;
bool lostFrame;

int Motor_speed = 100;

class RotorArm {
  private:
    int _pin_en1;
    int _pin_en2;
    int _pin_pwm;
    int _pin_fold_sw;
    int _pin_unfold_sw;
    int _status;
  public:
    RotorArm(int pin_en1 = 9, int pin_en2 = 10, int pin_pwm = 11
             , int pin_fold_sw = 14, int pin_unfold_sw = 15)
      : _pin_en1(pin_en1), _pin_en2(pin_en2), _pin_pwm(pin_pwm)
      , _pin_fold_sw(pin_fold_sw), _pin_unfold_sw(pin_unfold_sw)
    {
      pinMode(_pin_en1, OUTPUT);
      pinMode(_pin_en2, OUTPUT);
      pinMode(_pin_pwm, OUTPUT);
      pinMode(_pin_fold_sw, INPUT_PULLUP);
      pinMode(_pin_unfold_sw, INPUT_PULLUP);
    }
    void fold(int pwm = 1024) {
      if (isFoldSWPushed()) {  // STOP
        stopMotor();
      }
      else {                       // FOLDING
        digitalWrite(_pin_en1, LOW);
        digitalWrite(_pin_en2, HIGH);
        analogWrite(_pin_pwm, pwm);
      }
    }

    void unfold(int pwm = 1024) {
      if (isUnfoldSWPushed()) {  // STOP
        stopMotor();
      }
      else {                       // UNFOLDING
        digitalWrite(_pin_en1, HIGH);
        digitalWrite(_pin_en2, LOW);
        analogWrite(_pin_pwm, pwm);
      }
    }

    void stopMotor(int type = 0) {
      if (type) {
        digitalWrite(_pin_en1, LOW);
        digitalWrite(_pin_en2, HIGH);
        analogWrite(_pin_pwm, 0);
      }
      else {
        digitalWrite(_pin_en1, LOW);
        digitalWrite(_pin_en2, LOW);
        analogWrite(_pin_pwm, 1024);
      }
    }
    bool isFoldSWPushed() {
      return !digitalRead(_pin_fold_sw);
    }
    bool isUnfoldSWPushed() {
      return !digitalRead(_pin_unfold_sw);
    }
};

RotorArm arm_FR(11, 12, 2,  14, 24);
RotorArm arm_RR(26, 25, 5,  14, 27);
RotorArm arm_FL(18, 19, 6,  14, 28);
RotorArm arm_RL(15, 16, 29, 14, 17);

void setup()
{
  //pinMode(ENA,OUTPUT);
  //pinMode(ENB,OUTPUT);
//  pinMode(LED, OUTPUT);
//  pinMode(EN1, OUTPUT);
//  pinMode(EN2, OUTPUT);
//  pinMode(PWM1, OUTPUT);
//  pinMode(FOLD_SW, INPUT_PULLUP);
//  pinMode(UNFOLD_SW, INPUT_PULLUP);
  //pinMode(EN3,OUTPUT);
  //pinMode(EN4,OUTPUT);
  futaba.begin();
}
void loop()
{
  if (futaba.read(channels, &failSafe, &lostFrame)) {
    Serial.println(channels[11]);
    Serial.println(arm_FR.isUnfoldSWPushed());
    Serial.println(arm_RR.isUnfoldSWPushed());
    if (channels[11] > 1500) {     // Folding Tx
      arm_FR.fold();
      arm_RR.fold();
      arm_FL.fold();
      arm_RL.fold();
      //      if (digitalRead(FOLD_SW)) {  // FOLDING
      //        digitalWrite(LED, HIGH);   // set the LED on
      //        digitalWrite(EN1, HIGH);
      //        digitalWrite(EN2, LOW);
      //        analogWrite(PWM1, 1024);
      //      }
      //      else {                       // STOP
      //        digitalWrite(LED, LOW);    // set the LED on
      //        digitalWrite(EN1, LOW);
      //        digitalWrite(EN2, LOW);
      //        analogWrite(PWM1, 0);
      //      }
    }
    else if (channels[11] < 1000) { //Unfolding Tx
      arm_FR.unfold();
      arm_RR.unfold();
      arm_FL.unfold();
      arm_RL.unfold();
      //      if (digitalRead(UNFOLD_SW)) {  // UNFOLDING
      //        digitalWrite(LED, HIGH);   // set the LED on
      //        digitalWrite(EN1, LOW);
      //        digitalWrite(EN2, HIGH);
      //        analogWrite(PWM1, 1024);
      //      }
      //      else {                       // STOP
      //        digitalWrite(LED, LOW);    // set the LED on
      //        digitalWrite(EN1, LOW);
      //        digitalWrite(EN2, LOW);
      //        analogWrite(PWM1, 0);
      //      }
    }
    else {
      arm_FR.stopMotor();
      arm_RR.stopMotor();
      arm_FL.stopMotor();
      arm_RL.stopMotor();
      //      digitalWrite(EN1, HIGH);
      //      digitalWrite(EN2, HIGH);
      //      analogWrite(PWM1, 0);
      //      delay(100);
    }
  }
}
