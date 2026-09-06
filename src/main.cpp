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

const unsigned long SERVO_UPDATE_INTERVAL = 20;

unsigned long lastHandUpdate = 0;
unsigned long lastRotateHandUpdate = 0;

const unsigned long BUTTON_DEBOUNCE_TIME = 30;

int stableGrabInput = 0;
int previousGrabRawInput = 0;
unsigned long grabInputChangeTime = 0;

int stableRotateInput = 0;
int previousRotateRawInput = 0;
unsigned long rotateInputChangeTime = 0;

const unsigned long COMM_TIMEOUT = 1000;

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

    InputState input = {0,0,0,0,0};
    input.theta = data.leftX;
    input.R = data.leftY;
    input.Z = data.rightY;

    if ( data.button_up && !data.button_down && !data.button_left && !data.button_right && !data.button_circle && !data.button_cross && !data.button_triangle) {
        input.GrabHand = 1;
    } else if (data.button_down && !data.button_up && !data.button_left && !data.button_right && !data.button_circle && !data.button_cross && !data.button_triangle) {
        input.GrabHand = -1;
    } else {
        input.GrabHand = 0;
    }

    if (data.button_left && !data.button_right && !data.button_up && !data.button_down && !data.button_circle && !data.button_cross && !data.button_triangle) {
        input.RotateHand = 1;
    } else if (data.button_right && !data.button_left && !data.button_up && !data.button_down && !data.button_circle && !data.button_cross && !data.button_triangle) {
        input.RotateHand = -1;
    } else {
        input.RotateHand = 0;
    }

    if(data.button_circle && !data.button_left && !data.button_right && !data.button_up && !data.button_down && !data.button_cross && !data.button_triangle){
        input.releaseObject = 1;
    } else{
        input.releaseObject = 0;
    }
    if(data.button_cross && !data.button_circle && !data.button_left && !data.button_right && !data.button_up && !data.button_down && !data.button_triangle){
        input.getObject = 1;
    } else{
        input.getObject = 0;
    }

    if(data.button_triangle && !data.button_circle && !data.button_left && !data.button_right && !data.button_up && !data.button_down && !data.button_cross){
        input.liftup = 1;
    } else{
        input.liftup = 0;
    }
    return input;
}

const float DEAD_ZONE = 0.2;

void thetaDrive(InputState input) {
    if (input.theta > DEAD_ZONE) {
        theta_M.drive(25);
    } else if (input.theta < -DEAD_ZONE) {
        theta_M.drive(-25);
    } else {
        theta_M.drive(0);
    }
}

void lengthDrive(InputState input) {
    if (input.R > DEAD_ZONE) {
        length_M.drive(25);
    } else if (input.R < -DEAD_ZONE) {
        length_M.drive(-25);
    } else {
        length_M.drive(0);
    }
}

const unsigned long liftDuration = 2000;
const int lift_value = 150;
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
    if (abs(input.Z) < HAND_DEAD_ZONE) {
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

int debounceGrabInput(int rawInput) {
    unsigned long now = millis();

    if (rawInput != previousGrabRawInput) {
        previousGrabRawInput = rawInput;
        grabInputChangeTime = now;
    }
    if ((now - grabInputChangeTime) >= BUTTON_DEBOUNCE_TIME) {
        stableGrabInput = previousGrabRawInput;
    }
    return stableGrabInput;
}

int debounceRotateInput(int rawInput) {
    unsigned long now = millis();
    if (rawInput != previousRotateRawInput) {
        previousRotateRawInput = rawInput;
        rotateInputChangeTime = now;
    }

    if ((now - rotateInputChangeTime)>= BUTTON_DEBOUNCE_TIME) {
        stableRotateInput = previousRotateRawInput;
    }
    return stableRotateInput;
}

void handDrive(InputState input) {

    unsigned long now = millis();
    if(input.getObject){
        const int getAngle = 10;
        if(handAngle != getAngle){
            handAngle = getAngle;
            hand_servo.write(handAngle);
        }
    } else if(input.releaseObject){
        const int releaseAngle = 120;
        if(handAngle != releaseAngle){
            handAngle = releaseAngle;
            hand_servo.write(handAngle);
        }
    } else{
        int grabInput = debounceGrabInput(input.GrabHand);
        if (grabInput != 0 && (now - lastHandUpdate) >= SERVO_UPDATE_INTERVAL) {
            int newAngle = handAngle + grabInput * HAND_STEP;
            newAngle = constrain(newAngle,SERVO_MIN_ANGLE,SERVO_MAX_ANGLE );
            if (newAngle != handAngle) {
                handAngle = newAngle;
                hand_servo.write(handAngle);
            }
            lastHandUpdate = now;
        }
    }
    int rotateInput = debounceRotateInput(input.RotateHand);
    if (rotateInput != 0 && (now - lastRotateHandUpdate) >=SERVO_UPDATE_INTERVAL) {
        int newAngle = rotateHandAngle + rotateInput * ROTATE_HAND_STEP;
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

void stopServos()
{
    stableGrabInput = 0;
    previousGrabRawInput = 0;

    stableRotateInput = 0;
    previousRotateRawInput = 0;
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

    if (receivePacket(data)) {
        lastPacketTime = millis();

        InputState input = getInput(data);

        thetaDrive(input);
        lengthDrive(input);
        Z_Drive(input);
        handDrive(input);
    }

    if ((millis() - lastPacketTime) > COMM_TIMEOUT) {
        stopMotors();
        stopServos();
    }
}