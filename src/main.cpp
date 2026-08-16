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

const double THETA_TOLERANCE = 2.0;
const double LENGTH_TOLERANCE = 2.0;

const double THETA_KP = 2.0;
const double LENGTH_KP = 2.0;

const int MAX_MOTOR_PWM = 100;

//send data with serial
struct __attribute__((packed)) DeltaData{
  //controller input
  float leftX,leftY,leftRO,rightX,rightY,rightRO;
  int Left,Right;
  int Cross,Circle,Triangle,Rectanlge;

  int State;//bool
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

//data struct
struct InputState{
  bool x1;
  bool x2;
  bool y1;
  bool y2;
  bool z1;
  bool z2;
};

struct AngleState{
  double lastRawAngle;
  long loopcount;
  bool initialized;
};
AngleState theta_angle = {0.0,0,false};
AngleState length_angle = {0.0,0,false};

struct CurrentState{
  double current_theta;
  double current_length_angle;
  double current_length;
  double current_X;
  double current_Y;
};
struct OutputState{
  double target_theta;
  double target_length_angle;
  double target_length;
  double target_X;
  double target_Y;
};

InputState getControllerInput(const DeltaData& data){
  InputState input;

  input.x1 = (data.Left != 0);
  input.x2 = (data.Right != 0);

  input.y1 = (data.Triangle != 0);
  input.y2 = (data.Cross != 0);

  input.z1 = (data.Circle != 0);
  input.z2 = (data.Rectanlge != 0);

  return input;
}

double getContinuousAngle(Adafruit_AS5600* sensor,AngleState& state){
   double raw = sensor->getRawAngle();

  if(!state.initialized){
    state.lastRawAngle = raw;
    state.initialized = true;
  }
  else{
    double diff = raw - state.lastRawAngle;

    // 4095 → 0 の境界を通過
    if(diff < -2048){
      state.loopcount++;
    }
    // 0 → 4095 の境界を通過
    else if(diff > 2048){
      state.loopcount--;
    }

    state.lastRawAngle = raw;
  }

  double totalRaw =
      state.loopcount * 4096.0 + raw;

  return totalRaw * 360.0 / 4096.0;
}


CurrentState getCurrentState(){
  CurrentState state;
  double current_theta = getContinuousAngle(as5600[theta_as],theta_angle);
  double current_length_angle = getContinuousAngle(as5600[length_as],length_angle);

  state.current_theta = current_theta;
  state.current_length_angle = current_length_angle;
  state.current_length = offset_Length + current_length_angle*pinion_circle/360.0;
  state.current_X=state.current_length*std::cos(state.current_theta*PI/180.0);
  state.current_Y=state.current_length*std::sin(state.current_theta*PI/180.0);

  return state;
}

OutputState setTarget(double x, double y){
  OutputState output;
  output.target_X = x;
  output.target_Y = y;
  output.target_length = std::sqrt(output.target_X*output.target_X + output.target_Y*output.target_Y);
  output.target_theta = std::atan2(output.target_Y,output.target_X)*180.0/PI;
  output.target_length_angle = (output.target_length - offset_Length)/pinion_circle*360.0;
  return output;
}

OutputState updateTarget(const DeltaData& data, OutputState output){
  InputState input = getControllerInput(data);
  double x = output.target_X;
  double y = output.target_Y;
  if(input.x2) x += 20;
  if(input.x1) x -= 20;
  if(input.y1) y += 20;
  if(input.y2) y -= 20;

  return setTarget(x,y);
}

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

double angleDifference(double target, double current){
  double diff = target - current;

  while(diff > 180.0){
    diff -= 360.0;
  }

  while(diff < -180.0){
    diff += 360.0;
  }

  return diff;
}

void controlMotor(const CurrentState& state, const OutputState& target){
  double theta_error = angleDifference(target.target_theta,state.current_theta);
  int theta_pwm = 0;
  if(std::abs(theta_error) > THETA_TOLERANCE){
    theta_pwm = (int)(THETA_KP * theta_error);
    theta_pwm = constrain(theta_pwm,-MAX_MOTOR_PWM,MAX_MOTOR_PWM);
  }
  theta_M.drive(theta_pwm);

  double length_error = target.target_length_angle - state.current_length_angle;
  int length_pwm;
  if(std::abs(length_error) > LENGTH_TOLERANCE){
    length_pwm = (int)(LENGTH_KP*length_error);
    length_pwm = constrain(length_pwm,-MAX_MOTOR_PWM,MAX_MOTOR_PWM);
  }
  length_M.drive(length_pwm);
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