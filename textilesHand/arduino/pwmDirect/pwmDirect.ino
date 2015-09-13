#include <Servo.h>
 
Servo servoL;  // create servo object to control a servo
               // a maximum of eight servo objects can be created
Servo servoR;  // create servo object to control a servo
               // a maximum of eight servo objects can be created
 
void setup()
{
  servoL.attach(9);  // attaches the servo on pin 9 to the servo object
  servoR.attach(10);  // attaches the servo on pin 9 to the servo object

  // Open
  //servoL.write(10);
  //servoR.write(170);
 
  // Close
  //servoL.write(105);
  //servoR.write(75);

  // Extra Close
  servoL.write(125);
  servoR.write(55);

}
 
 
void loop()
{
} 
