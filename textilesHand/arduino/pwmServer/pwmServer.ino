#include <Servo.h>
 
Servo servoL;  // create servo object to control a servo
               // a maximum of eight servo objects can be created
Servo servoR;  // create servo object to control a servo
               // a maximum of eight servo objects can be created
 
void setup()
{
  servoL.attach(9);  // attaches the servo on pin 9 to the servo object
  servoR.attach(10);  // attaches the servo on pin 9 to the servo object

  Serial.begin(9600);
}
 
char incomingByte;   // for incoming serial data
 
void loop()
{

    if (Serial.available() > 0) {  // wait to receive data...

        // read the incoming byte:
        incomingByte = Serial.read();
        // say what you got:
        Serial.print("Got 0x");
        Serial.println(incomingByte, HEX);

        if(incomingByte==97) {
            Serial.println("a open");
            // Open
            servoL.write(10);
            servoR.write(170);
        } else if (incomingByte==98) {
            Serial.println("b close");
            // Extra Close
            servoL.write(125);
            servoR.write(55);
        }
    }

} 
