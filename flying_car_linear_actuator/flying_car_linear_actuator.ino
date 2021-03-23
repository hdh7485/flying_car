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
void setup()
{
  //pinMode(ENA,OUTPUT);
  //pinMode(ENB,OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(FOLD_SW, INPUT_PULLUP);
  pinMode(UNFOLD_SW, INPUT_PULLUP);
  //pinMode(EN3,OUTPUT);
  //pinMode(EN4,OUTPUT);

  futaba.begin();
}
void loop()
{
  if (futaba.read(channels, &failSafe, &lostFrame)) {
    Serial.println(channels[11]);
    Serial.println(digitalRead(FOLD_SW));
    Serial.println(digitalRead(UNFOLD_SW));
    if (channels[11] > 1500) {     // Folding Tx
      if (digitalRead(FOLD_SW)) {  // FOLDING
        digitalWrite(LED, HIGH);   // set the LED on
        digitalWrite(EN1, HIGH);
        digitalWrite(EN2, LOW);
        analogWrite(PWM1, 1024);
      }
      else {                       // STOP
        digitalWrite(LED, LOW);    // set the LED on
        digitalWrite(EN1, LOW);
        digitalWrite(EN2, LOW);
        analogWrite(PWM1, 0);
      }
    }
    else if (channels[11] < 1000) { //Unfolding Tx
      if (digitalRead(UNFOLD_SW)) {  // UNFOLDING
        digitalWrite(LED, HIGH);   // set the LED on
        digitalWrite(EN1, LOW);
        digitalWrite(EN2, HIGH);
        analogWrite(PWM1, 1024);
      }
      else {                       // STOP
        digitalWrite(LED, LOW);    // set the LED on
        digitalWrite(EN1, LOW);
        digitalWrite(EN2, LOW);
        analogWrite(PWM1, 0);
      }
    }
    else {
      digitalWrite(EN1, HIGH);
      digitalWrite(EN2, HIGH);
      analogWrite(PWM1, 0);
      delay(100);
    }
  }
}
