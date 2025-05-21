#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <ESP32Servo.h>
#include <math.h>

MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float pitchFiltered = 0.0;
float pitchBias = 0.0;
float alpha = 0.98;

volatile bool mpuInterrupt = false;
#define MPU_INT_PIN 19

Servo myServo;
#define SERVO_PIN 13

float setpoint = 0.0;
float kp = 20.0;
float ki = 30.0;
float kd = 1.0;
float integral = 0, lastError = 0;
unsigned long lastTime = 0;

String inputString = "";
bool stringComplete = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  myServo.attach(SERVO_PIN, 500, 2500);
  myServo.write(90);

  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT);
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
    dmpReady = true;
  }

  lastTime = millis();
}

void loop() {
  handleSerialInput();
  if (!dmpReady || !mpuInterrupt) return;

  mpuInterrupt = false;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    float pitch = -asin(2.0 * (q.x * q.z - q.w * q.y)) * 180.0 / M_PI;
    pitch += pitchBias;

    pitchFiltered = alpha * pitchFiltered + (1 - alpha) * pitch;

    float error = setpoint - pitchFiltered;
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    integral += error * dt;
    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = -(kp * error + ki * integral + kd * derivative);
    int servoAngle = constrain(90 + output, 0, 180);
    myServo.write(servoAngle);

    Serial.print("Pitch: ");
    Serial.print(pitchFiltered, 2);
    Serial.print(" | Servo: ");
    Serial.print(servoAngle);
    Serial.print(" | PID: kp=");
    Serial.print(kp);
    Serial.print(" ki=");
    Serial.print(ki);
    Serial.print(" kd=");
    Serial.print(kd);
    Serial.print(" | Bias=");
    Serial.print(pitchBias);
    Serial.print(" | Alpha=");
    Serial.println(alpha, 2);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void handleSerialInput() {
  if (stringComplete) {
    inputString.trim();

    if (inputString.startsWith("kp=")) {
      kp = inputString.substring(3).toFloat();
      Serial.print("Updated kp: ");
      Serial.println(kp);
    } else if (inputString.startsWith("ki=")) {
      ki = inputString.substring(3).toFloat();
      Serial.print("Updated ki: ");
      Serial.println(ki);
    } else if (inputString.startsWith("kd=")) {
      kd = inputString.substring(3).toFloat();
      Serial.print("Updated kd: ");
      Serial.println(kd);
    } else if (inputString.startsWith("bias=")) {
      pitchBias = inputString.substring(5).toFloat();
      Serial.print("Updated pitch bias: ");
      Serial.println(pitchBias);
    } else if (inputString.startsWith("setpoint=")) {
      setpoint = inputString.substring(9).toFloat();
      Serial.print("Updated setpoint: ");
      Serial.println(setpoint);
    } else if (inputString.startsWith("alpha=")) {
      alpha = inputString.substring(6).toFloat();
      Serial.print("Updated filter alpha: ");
      Serial.println(alpha, 2);
    } else {
      Serial.println("Unknown command. Use kp=, ki=, kd=, bias=, setpoint=, alpha=");
    }

    inputString = "";
    stringComplete = false;
  }
}
