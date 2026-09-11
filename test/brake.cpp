#include <Arduino.h>
#include <cmath>
#include <ESP32Servo.h>

const int theta_pin = 18;
const int theta_pwm = 19;
const int theta_ch = 6;

const int length_pin = 32;
const int length_pwm = 33;
const int length_ch = 7;

const int height_pin = 13;
const int hand_pin = 14;
const int hand_rotate_pin = 26;

int handAngle = 90;
int rotateHandAngle = 90;

const int SERVO_MIN_ANGLE = 10;
const int SERVO_MAX_ANGLE = 170;

const int HAND_STEP = 2;
const int ROTATE_HAND_STEP = 1;
const int AUTO_ROTATE_STEP = 1;
const int AUTO_ROTATE_TOLERANCE = 3;

const unsigned long SERVO_UPDATE_INTERVAL = 20;

unsigned long lastHandUpdate = 0;
unsigned long lastRotateHandUpdate = 0;

const unsigned long COMM_TIMEOUT = 1000;

const int leftAnglepin = 16;
const int rightAnglepin = 5;
const int statepin = 17;

class MotorDrive {

public:
    int dirpin;
    int motorpwm;
    int pwmch;
    MotorDrive(int pin1,int pin2,int ch) {
        dirpin = pin1;
        motorpwm = pin2;
        pwmch = ch;
    }

    void setup() {
        pinMode(dirpin,OUTPUT);
        ledcSetup(pwmch,12800,8);
        ledcAttachPin(motorpwm,pwmch);
        digitalWrite(dirpin,LOW);
        ledcWrite(pwmch,0);
    }


    void drive(int val) {
        val = constrain(val,-255,255);
        if (val < 0) {
            digitalWrite(dirpin,HIGH);
            ledcWrite(pwmch,-val);
        } else if (val > 0) {
            digitalWrite(dirpin,LOW);
            ledcWrite(pwmch,val);
        } else {
            digitalWrite(dirpin,LOW);
            ledcWrite(pwmch,0);
        }
    }
};

MotorDrive theta_M{theta_pin,theta_pwm,theta_ch};
MotorDrive length_M{length_pin,length_pwm,length_ch};

Servo height_M,hand_servo,rotate_hand_servo;

struct __attribute__((packed)) DeltaData {

    float leftX;
    float leftY;
    float rightY;

    uint8_t button_left;
    uint8_t button_right;
    uint8_t button_up;
    uint8_t button_down;

    uint8_t button_circle;
    uint8_t button_cross;
    uint8_t button_triangle;
    uint8_t button_rectangle;

    uint8_t state;
    float angle;
};

struct InputState {

    float theta;

    float R;
    float Z;

    int RotateHand;
    int GrabHand;

    int releaseObject;
    int getObject;
    int liftup;
    int adjustHand;

    int objectState;
    float objectAngle;
    float detectedAngle;
};

const uint8_t HEADER = 0xAA;
const size_t DATA_SIZE = sizeof(DeltaData);
const size_t PACKET_SIZE = 1 + DATA_SIZE + 1;

unsigned long lastPacketTime = 0;

uint8_t calculateCRC(const uint8_t *data,size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0;i < len;i++) {
        crc ^= data[i];
        for (uint8_t j = 0;j < 8;j++) {
            if (crc & 0x80) {
                crc =(crc << 1)^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool receivePacket(DeltaData& data) {
    static uint8_t buffer[PACKET_SIZE];
    static size_t index = 0;
    while (Serial.available() > 0) {
        uint8_t received = Serial.read();
        if (index == 0) {
            if (received == HEADER) {
                buffer[index++] = received;
            }
            continue;
        }

        buffer[index++] = received;

        if (index >= PACKET_SIZE) {
            uint8_t calculatedCRC = calculateCRC(&buffer[1],DATA_SIZE);
            uint8_t receivedCRC = buffer[PACKET_SIZE - 1];

            if (calculatedCRC == receivedCRC) {
                memcpy(&data,&buffer[1],DATA_SIZE);
                index = 0;
                return true;
            }
            index = 0;
        }
    }
    return false;
}

InputState getInput(DeltaData& data) {

    InputState input = {};
    input.theta = data.leftX;
    input.R = data.leftY;
    input.Z = data.rightY;
    input.objectState = data.state;
    input.detectedAngle = data.angle;

    if ( data.button_up && !data.button_down && !data.button_left && !data.button_right && !data.button_circle && !data.button_cross && !data.button_triangle && !data.button_rectangle) {
        input.GrabHand = 1;
    } else if (data.button_down && !data.button_up && !data.button_left && !data.button_right && !data.button_circle && !data.button_cross && !data.button_triangle && !data.button_rectangle) {
        input.GrabHand = -1;
    } else {
        input.GrabHand = 0;
    }

    if (data.button_left && !data.button_right && !data.button_up && !data.button_down && !data.button_circle && !data.button_cross && !data.button_triangle && !data.button_rectangle) {
        input.RotateHand = 1;
    } else if (data.button_right && !data.button_left && !data.button_up && !data.button_down && !data.button_circle && !data.button_cross && !data.button_triangle && !data.button_rectangle) {
        input.RotateHand = -1;
    } else {
        input.RotateHand = 0;
    }

    if(data.button_circle && !data.button_left && !data.button_right && !data.button_up && !data.button_down && !data.button_cross && !data.button_triangle && !data.button_rectangle){
        input.releaseObject = 1;
    } else{
        input.releaseObject = 0;
    }
    if(data.button_cross && !data.button_circle && !data.button_left && !data.button_right && !data.button_up && !data.button_down && !data.button_triangle && !data.button_rectangle){
        input.getObject = 1;
    } else{
        input.getObject = 0;
    }

    if(data.button_triangle && !data.button_circle && !data.button_left && !data.button_right && !data.button_up && !data.button_down && !data.button_cross && !data.button_rectangle){
        input.liftup = 1;
    } else{
        input.liftup = 0;
    }

    if(data.button_rectangle && !data.button_triangle && !data.button_circle && !data.button_left && !data.button_right && !data.button_up && !data.button_down && !data.button_cross){
        input.adjustHand = 1;
    } else{
        input.adjustHand = 0;
    }

    if(data.angle<85.0){
        input.objectAngle = -1;
    } else if(data.angle>95.0){
        input.objectAngle = 1;
    } else{
        input.objectAngle = 0;
    }
    return input;
}

const float DEAD_ZONE = 0.2;
const unsigned long THETA_BRAKE_TIME = 80;
const int THETA_DRIVE_PWM = 25;
const int THETA_BRAKE_PWM = 25;

bool thetaWasMoving = false;
bool thetaBraking = false;

int lastThetaDirection = 0;

unsigned long thetaBrakeStartTime = 0;

void thetaDrive(InputState input) {

    unsigned long now = millis();

    if (input.theta > DEAD_ZONE) {
        theta_M.drive(THETA_DRIVE_PWM);
        thetaWasMoving = true;
        thetaBraking = false;
        lastThetaDirection = 1;
        return;
    }
    if (input.theta < -DEAD_ZONE) {
        theta_M.drive(-THETA_DRIVE_PWM);
        thetaWasMoving = true;
        thetaBraking = false;
        lastThetaDirection = -1;
        return;
    }
    if (thetaWasMoving && !thetaBraking) {
        thetaBraking = true;
        thetaBrakeStartTime = now;
        thetaWasMoving = false;
    }
    if (thetaBraking) {
        if (now - thetaBrakeStartTime < THETA_BRAKE_TIME) {
            theta_M.drive(-lastThetaDirection * THETA_BRAKE_PWM);
        } else {
            theta_M.drive(0);
            thetaBraking = false;
        }
        return;
    }
    theta_M.drive(0);
}

void lengthDrive(InputState input) {
    if (input.R > DEAD_ZONE) {
        length_M.drive(30);
    } else if (input.R < -DEAD_ZONE) {
        length_M.drive(-30);
    } else {
        length_M.drive(0);
    }
}

const unsigned long liftDuration = 2000;
const int lift_value = 170;
unsigned long liftStartTime = 0;
bool previousLiftButton = false;
bool lifting = false;

void Z_Drive(InputState input) {
    unsigned long now = millis();

    bool liftPressed = input.liftup;
    if(liftPressed && !previousLiftButton && !lifting){
        lifting = true;
        liftStartTime = now;
    }
    previousLiftButton = liftPressed;

    if(lifting){
        if((now - liftStartTime) < liftDuration){
            height_M.write(lift_value);
            return;
        }
        lifting = false;
        height_M.write(95);
        return;
    }


    const float HAND_DEAD_ZONE = 0.08;
    if (fabs(input.Z) < HAND_DEAD_ZONE) {
        height_M.write(95);
        return;
    }
    int z_value = 0;
    
    if(input.Z < 0){
        z_value =(int)(90 + 60 * input.Z);
    } else {
        z_value = (int)(90 + 80*input.Z);
    }
    z_value =constrain(z_value,0,180);
    height_M.write(z_value);
}

void handDrive(InputState input) {
    unsigned long now = millis();

    if (input.getObject) {
        handAngle = 10;
        hand_servo.write(handAngle);
    } else if (input.releaseObject) {
        handAngle = 120;
        hand_servo.write(handAngle);
    } else if (input.GrabHand != 0 && (now - lastHandUpdate) >= SERVO_UPDATE_INTERVAL) {
        int newAngle = handAngle + input.GrabHand * HAND_STEP;
        newAngle = constrain(newAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        if (newAngle != handAngle) {
            handAngle = newAngle;
            hand_servo.write(handAngle);
        }
        lastHandUpdate = now;
    }

    if (input.adjustHand) {
        int objectAngle = map((int)input.detectedAngle,0, 180,SERVO_MIN_ANGLE,SERVO_MAX_ANGLE);
        objectAngle = constrain(objectAngle,SERVO_MIN_ANGLE,SERVO_MAX_ANGLE);
        int error = objectAngle - rotateHandAngle;
        if ((now - lastRotateHandUpdate) >= SERVO_UPDATE_INTERVAL) {
            if (error > AUTO_ROTATE_TOLERANCE) {
                rotateHandAngle += AUTO_ROTATE_STEP;
            } else if (error < -AUTO_ROTATE_TOLERANCE) {
                rotateHandAngle -= AUTO_ROTATE_STEP;
            }
            rotateHandAngle = constrain(rotateHandAngle,SERVO_MIN_ANGLE,SERVO_MAX_ANGLE);
            rotate_hand_servo.write(rotateHandAngle);
            lastRotateHandUpdate = now;
        }
    } else if (input.RotateHand != 0 && (now - lastRotateHandUpdate) >= SERVO_UPDATE_INTERVAL) {
        int newAngle = rotateHandAngle + input.RotateHand * ROTATE_HAND_STEP;
        newAngle = constrain(newAngle,SERVO_MIN_ANGLE,SERVO_MAX_ANGLE);
        if (newAngle != rotateHandAngle) {
            rotateHandAngle = newAngle;
            rotate_hand_servo.write(rotateHandAngle);
        }
        lastRotateHandUpdate = now;
    }
}

void stopMotors(){
    theta_M.drive(0);   
    length_M.drive(0);
}


void expressState(InputState input){
    if(input.objectState){
        digitalWrite(statepin, HIGH);
    } else{
        digitalWrite(statepin,LOW);
    }
    if(input.objectAngle==1){
        digitalWrite(leftAnglepin, LOW);
        digitalWrite(rightAnglepin, HIGH);
    } else if(input.objectAngle==-1){
        digitalWrite(leftAnglepin, HIGH);
        digitalWrite(rightAnglepin, LOW);
    } else{
        digitalWrite(leftAnglepin, LOW);
        digitalWrite(rightAnglepin, LOW);
    }
}

void setup()
{

    Serial.begin(115200);
    Serial.setTimeout(10);

    theta_M.setup();
    length_M.setup();

    height_M.attach(height_pin);
    hand_servo.attach(hand_pin);
    rotate_hand_servo.attach(hand_rotate_pin);
    pinMode(statepin, OUTPUT);
    pinMode(rightAnglepin, OUTPUT);
    pinMode(leftAnglepin, OUTPUT);

    digitalWrite(statepin, LOW);
    digitalWrite(rightAnglepin, LOW);
    digitalWrite(leftAnglepin, LOW);

    stopMotors();
    height_M.write(90);
    handAngle = 90;
    rotateHandAngle = 90;

    hand_servo.write(handAngle);
    rotate_hand_servo.write(rotateHandAngle);

    lastPacketTime = millis();
    delay(100);
}

void loop(){
    DeltaData data;
    bool received = false;

    while (receivePacket(data)) {
        received = true;
        lastPacketTime = millis();
    }

    if (received) {
        InputState input = getInput(data);

        thetaDrive(input);
        lengthDrive(input);
        Z_Drive(input);
        handDrive(input);
        expressState(input);
    }

    if ((millis() - lastPacketTime) > COMM_TIMEOUT) {
        stopMotors();
        digitalWrite(statepin, LOW);
        digitalWrite(leftAnglepin, LOW);
        digitalWrite(rightAnglepin, LOW);
    }
}