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

int handAngle = 90;
int rotateHandAngle = 90;

const int HAND_STEP = 1;
const int ROTATE_HAND_STEP = 1;

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

struct __attribute__((packed)) DeltaData {
  float leftX;
  float leftY;
  float rightX;
  float rightY;

  uint8_t L_button;
  uint8_t R_button;

  uint8_t button_left;
  uint8_t button_right;
  uint8_t button_up;
  uint8_t button_down;

  float deltaX;
  float deltaY;
  float angle;

  uint8_t state;
};

struct InputState{
  int theta;
  float R;
  float Z;
  int RotateHand;
  int GrabHand;
  bool state;
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

InputState getInput(DeltaData& data){
  InputState input = {0, 0, 0, 0, 0, false};  
  if(receivePacket(data)){
    //theta
    if(data.L_button && !data.R_button){
      input.theta = 1;
    } else if(data.R_button && !data.L_button){
      input.theta = -1;
    } else{
      input.theta = 0;
    }
    //r
    input.R = data.leftY;
    //z
    input.Z = data.rightY;
    //grab
    if(data.button_up && !data.button_down){
      input.GrabHand = 1;
    } else if(data.button_down && !data.button_up){
      input.GrabHand = -1;
    } else{
      input.GrabHand = 0;
    }
    //rotate hand
    if(data.button_left && !data.button_right){
      input.RotateHand = 1;
    } else if(data.button_right && !data.button_left){
      input.RotateHand = -1;
    } else{
      input.RotateHand = 0;
    }
    //state
    input.state = data.state;
  }
  return input;
}

void thetaDrive(InputState input){
  if(input.theta == 1){
    theta_M.drive(25);
  } else if(input.theta == -1){
    theta_M.drive(-25);
  } else{
    theta_M.drive(0);
  }
}

void lengthDrive(InputState input){//pwmは要変更
  const float DEAD_ZONE = 0.2;
  if(input.R > DEAD_ZONE){
    length_M.drive(25);
  } else if(input.R < -DEAD_ZONE){
    length_M.drive(-25);
  } else{
    length_M.drive(0);
  }
}

void Z_Drive(InputState input){
  int z_value = (int)(90 + 60*input.Z);
  if(input.Z == 0){
    height_M.write(90);
  } else if(z_value > 95 || z_value < 85){
    height_M.write(z_value);
  }
}

void handDrive(InputState input){
  if(input.GrabHand != 0){
    handAngle += input.GrabHand*HAND_STEP;
    handAngle = constrain(handAngle,0,180);
    hand_servo.write(handAngle);
  }

  if(input.RotateHand != 0){
    rotateHandAngle += input.RotateHand*ROTATE_HAND_STEP;
    rotateHandAngle = constrain(rotateHandAngle,0,180);
    rotate_hand_servo.write(rotateHandAngle);
  }
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

/*
void loop(){

  DeltaData data;

  InputState input = getInput(data);

  thetaDrive(input);
  lengthDrive(input);
  Z_Drive(input);
  handDrive(input);
}
*/