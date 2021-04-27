#include "Arduino.h"
#include "RotorArm.h"

RotorArm::RotorArm(int pin_en1, int pin_en2, int pin_pwm
                   , int pin_fold_sw, int pin_unfold_sw)
  : _pin_en1(pin_en1), _pin_en2(pin_en2), _pin_pwm(pin_pwm)
  , _pin_fold_sw(pin_fold_sw), _pin_unfold_sw(pin_unfold_sw)
{
  pinMode(_pin_en1, OUTPUT);
  pinMode(_pin_en2, OUTPUT);
  pinMode(_pin_pwm, OUTPUT);
  pinMode(_pin_fold_sw, INPUT_PULLUP);
  pinMode(_pin_unfold_sw, INPUT_PULLUP);
}
void RotorArm::fold(int pwm) {
  if (isFoldSWPushed()) {  // STOP
    stopMotor();
  }
  else {                       // FOLDING
    digitalWrite(_pin_en1, LOW);
    digitalWrite(_pin_en2, HIGH);
    analogWrite(_pin_pwm, pwm);
  }
}

void RotorArm::unfold(int pwm) {
  if (isUnfoldSWPushed()) {  // STOP
    stopMotor();
  }
  else {                       // UNFOLDING
    digitalWrite(_pin_en1, HIGH);
    digitalWrite(_pin_en2, LOW);
    analogWrite(_pin_pwm, pwm);
  }
}

void RotorArm::stopMotor(int type) {
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
bool RotorArm::isFoldSWPushed() {
  return !digitalRead(_pin_fold_sw);
}
bool RotorArm::isUnfoldSWPushed() {
  return !digitalRead(_pin_unfold_sw);
}
