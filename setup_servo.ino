#include <Servo.h>

Servo servo1;
Servo servo2;

int servoPos = 0;

void setup()
{
  servo1.attach(3);
  servo2.attach(5);
}
void loop() 
{
  for (servoPos = 0; servoPos < 180; servoPos++)
  {
    servo1.write(servoPos);
    servo2.write(servoPos);
    delay(100);
  }
  
  for (servoPos = 180; servoPos >0; servoPos--)
  {
    servo1.write(servoPos);
    servo2.write(servoPos);
    delay(100);
  }
  
}
