#include<Arduino.h>
#include<Wire.h>
#include<cmath>


#define SDA2_pin 25
#define SCL2_pin 32
#define offset_Length 100.0
#define offset_Theta 90.0

#define I2C_1 Wire
TwoWire I2C_2 = TwoWire(1);

const int theta_pin = 18;
const int theta_pwm = 19;
const int theta_ch = 0;

const int length_pin = 26;
const int length_pwm = 27;
const int length_ch =1;

const int height_pin = 13;//z方向は360サーボ
const int hand_pin = 14;

class MotorDrive{
public:
  int dirpin;
  int motorpwm;
  int pwmch;

  MotorDrive(int pin1,int pin2,int ch){
    dirpin=pin1;
    motorpwm=pin2;
    pwmch=ch;
  }

  void setup() {
    pinMode(dirpin, OUTPUT);
    ledcSetup(pwmch, 12800, 8);
    ledcAttachPin(motorpwm, pwmch);
  }

  void drive(int val){
    val = constrain(val,-255,255);
    if(val<0){
      digitalWrite(dirpin,HIGH);
      ledcWrite(pwmch,-val);
    }
    else if(val>0){
      digitalWrite(dirpin,LOW);
      ledcWrite(pwmch,val);
    }
    else{
      digitalWrite(dirpin,LOW);
      ledcWrite(pwmch,0);
    }
  }
};

MotorDrive theta_M{theta_pin,theta_pwm,theta_ch};
MotorDrive length_M{length_pin,length_pwm,length_ch};

struct __attribute__((packed)) DeltaData{
  //controller input
  float leftX,leftY,leftRO,rightX,rightY,rightRO;
  int Left,Right;
  int Cross,Circle,Triangle,Rectanlge;

  int State;//bool
};

struct InputState{
  bool x1;
  bool x2;
  bool y1;
  bool y2;
  bool z1;
  bool z2;
};

struct CurrentState{
  double current_theta;
  double current_lengh_angle;
  double current_length;
};

void setup(){
  Serial.begin(115200);
  I2C_1.begin(21,22,400000);
  I2C_2.begin(SDA2_pin,SCL2_pin,400000);
  theta_M.setup();
  length_M.setup();
}
void loop(){

}