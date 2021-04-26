#ifndef ROTOR_ARM_H
#define ROTOR_ARM_H
#include "Arduino.h"

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
             , int pin_fold_sw = 14, int pin_unfold_sw = 15);
    void fold(int pwm = 1024);
    void unfold(int pwm = 1024);
    void stopMotor(int type = 0);
    bool isFoldSWPushed();
    bool isUnfoldSWPushed();
};
#endif
