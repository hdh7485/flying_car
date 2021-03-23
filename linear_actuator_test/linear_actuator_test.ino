#define LED 13
//#define ENA 10
//#define ENB 5
#define EN1 9
#define EN2 10
//#define EN3 7
//#define EN4 6
int Motor_speed = 100;
void setup()
{
  //pinMode(ENA,OUTPUT);
  //pinMode(ENB,OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  //pinMode(EN3,OUTPUT);
  //pinMode(EN4,OUTPUT);
}
void loop()
{
  digitalWrite(LED, HIGH);   // set the LED on

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, LOW);
  delay(2000);
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(100);
  //  digitalWrite(EN1, HIGH);
  //  digitalWrite(EN2, HIGH);
  //  delay(100);

  digitalWrite(LED, LOW);   // set the LED on

  digitalWrite(EN1, LOW);
  digitalWrite(EN2, HIGH);
  delay(2000);
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  delay(100);
  //  digitalWrite(EN1, HIGH);
  //  digitalWrite(EN2, HIGH);
  //  delay(100);
}
