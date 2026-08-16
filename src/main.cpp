#include<Arduino.h>
#include<Wire.h>
#include<cmath>
#include<Adafruit_AS5600.h>


#define SDA2_pin 25
#define SCL2_pin 32
#define offset_Length 100.0
#define offset_Theta 90.0
#define pinion_circle 9.6*PI //ピニオンの円周 
#define theta_parcent 10.0//thetaのサイズ比

#define I2C_1 Wire
TwoWire I2C_2 = TwoWire(1);
Adafruit_AS5600 theta_as5600,length_as5600;
Adafruit_AS5600* as5600[] = { &theta_as5600, &length_as5600 };
const int theta_as = 0;
const int length_as = 1;

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

//send data with serial
const uint8_t HEADER = 0xAA;
const size_t DATA_SIZE = sizeof(DeltaData);
const size_t PACKET_SIZE = 1 + DATA_SIZE +1;

struct __attribute__((packed)) DeltaData{
  //controller input
  float leftX,leftY,leftRO,rightX,rightY,rightRO;
  int Left,Right;
  int Cross,Circle,Triangle,Rectanlge;

  int State;//bool
};

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

//data struct
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
  double current_length_angle;
  double current_length;
  double current_X;
  double current_Y;
};

int loopcount = 0;
double lastRawAngle = 0.0;
bool isfirstread = true;

CurrentState getCurrentState(){
  CurrentState state;
  double thetaRaw = as5600[theta_as]->getRawAngle();
  double current_theta = thetaRaw*360.0/4096.0;

  double lengthRaw = as5600[length_as]->getRawAngle();

  if(isfirstread){
    lastRawAngle = lengthRaw;
    isfirstread = false;
  } else {
    double diff = lengthRaw - lastRawAngle;
    if(diff < -2048)loopcount++;
    else if(diff > 2048)loopcount--;
    lastRawAngle = lengthRaw;
  }

  double totalRaw = loopcount * 4096.0 + lengthRaw;
  float totalDegree = totalRaw*360.0/4096.0;

  state.current_theta = current_theta;
  state.current_length = offset_Length + totalDegree*pinion_circle/360.0;
  state.current_length_angle = totalDegree;
  state.current_X=state.current_length*std::cos(state.current_theta*PI/180.0);
  state.current_Y=state.current_length*std::sin(state.current_theta*PI/180.0);

  return state;
}

struct OutputState{
  double target_theta;
  double target_length_angle;
  double target_length;
  double target_X;
  double target_Y;
};

OutputState setTarget(double x, double y){
  OutputState output;
  output.target_X = x;
  output.target_Y = y;
  output.target_length = std::sqrt(output.target_X*output.target_X + output.target_Y*output.target_Y);
  output.target_theta = std::atan2(output.target_Y,output.target_X)*180.0/PI;
  output.target_length_angle = (output.target_length - offset_Length)/pinion_circle*360.0;
  return output;
}


void setup(){
  Serial.begin(115200);
  I2C_1.begin(21,22,400000);
  I2C_2.begin(SDA2_pin,SCL2_pin,400000);
  theta_M.setup();
  length_M.setup();

  theta_M.drive(0);
  length_M.drive(0);
}
void loop(){

}