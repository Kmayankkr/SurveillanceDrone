
#include <Servo.h>
 
Servo esc,esc1,esc2,esc3;

int t ; 

void setup()
{
  esc.attach(9);
  esc1.attach(6);
  esc2.attach(10);
  esc3.attach(11);
  Serial.begin(9600) ;
  t = 0 ;
}
 
void loop()
{

    if(Serial.available())
    { 
       while(Serial.available())
       {
          t = 10*t + Serial.read() - 48 ;
          delay(10) ;
       }

       Serial.println(t) ;

       esc.writeMicroseconds(t);
       esc1.writeMicroseconds(t);
       esc2.writeMicroseconds(t);
       esc3.writeMicroseconds(t);       

       t = 0 ; 
    }
    
}
