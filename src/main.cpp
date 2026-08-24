#include<Arduino.h>
#include<Wire.h>
#include<cmath>
#include<ESP32Servo.h>

const int theta_pin = 18;
const int theta_pwm = 19;
const int theta_ch = 0;

const int length_pin = 26;
const int length_pwm = 27;
const int length_ch =1;

const int height_pin = 13;//z方向は360サーボ
const int hand_pin = 14;
const int hand_rotate_pin = 25;

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
Servo height_M, hand_servo,rotate_hand_servo;

struct __attribute__((packed)) DeltaData{
  float leftX,leftY,leftRO,rightX,rightY,rightRO;
  int Left,Right;
  int Cross,Circle,Triangle,Rectanlge;
  //float deltaX,deltaY,Angle;
  int State;
};

struct InputState{
  int theta;
  int R;
  int Z;
  int RotateHand;
  int GrabHand;
};

const uint8_t HEADER = 0xAA;
const size_t DATA_SIZE = sizeof(DeltaData);
const size_t PACKET_SIZE = 1 + DATA_SIZE +1;

uint8_t calculateCRC(const uint8_t *data,size_t len){
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07; // 多項式 0x07
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void sendPacket(const DeltaData& data) {
  uint8_t buffer[PACKET_SIZE];
  buffer[0] = HEADER; 

  memcpy(&buffer[1], &data, DATA_SIZE); 
  buffer[PACKET_SIZE - 1] = calculateCRC(&buffer[1], DATA_SIZE); 
  Serial.write(buffer, PACKET_SIZE);
  Serial.flush();
}

bool receivePacket(DeltaData& data){
  static uint8_t buffer[PACKET_SIZE];
  static size_t index = 0;

  while(Serial.available() > 0){
    uint8_t received = Serial.read();
    if(index == 0){
      if(received == HEADER){
        buffer[index++] = received;
      }
      continue;
    }
    buffer[index++] = received;

    if(index >= PACKET_SIZE){
      uint8_t calculatedCRC = calculateCRC(&buffer[1], DATA_SIZE);
      uint8_t receivedCRC = buffer[PACKET_SIZE - 1];

      if(calculatedCRC == receivedCRC){

        memcpy(&data,&buffer[1],DATA_SIZE);
        index = 0;
        return true;
      }
      index = 0;
    }
  }
  return false;
}

void setup(){
  Serial.begin(115200);
  Serial.setTimeout(10);
  theta_M.setup();
  length_M.setup();
  height_M.attach(height_pin);
  hand_servo.attach(hand_pin);
  rotate_hand_servo.attach(hand_rotate_pin);

  theta_M.drive(0);
  length_M.drive(0);
  height_M.write(90);
  hand_servo.write(90);
  rotate_hand_servo.write(90);
  delay(100);
}

void loop(){

}