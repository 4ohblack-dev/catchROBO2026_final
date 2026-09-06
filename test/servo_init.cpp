#include<Arduino.h>
#include<ESP32Servo.h>

const int servopin = 26;
Servo servo;

void setup(){
    servo.attach(servopin);
    delay(100);

    servo.write(90);
}
void loop(){

}