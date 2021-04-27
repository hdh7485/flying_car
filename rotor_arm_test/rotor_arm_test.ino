#define LED 13
#define FOLD_SW 14
#define UNFOLD_SW 15
#define EN1 9
#define EN2 10
#define PWM1 11

#include "SBUS.h"
#include "src/RotorArm.h"

SBUS futaba(Serial1);
uint16_t channels[16];
bool failSafe;
bool lostFrame;

int Motor_speed = 100;


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
