#define LIGHT 12
#define FIELD 11

#include <Arduino.h>
#include <PID_v1.h>

double r1 = 46.03;
double r2 = 9.89;

struct {
  uint8_t kp = 6;
  uint8_t ki = 150;
  uint8_t kd = 0;
  uint8_t sampleTime = 100;
  double lowerLimit = 255 * 0.20; // 20% dc
  double upperLimit = 255 * 0.85; // 85% dc
} pidParameter;

double afterStartVoltage = 13.2;
double chargeVoltage = 14.4;
double maxVoltage = 14.7;
double batteryLightVoltage = 12.8;
uint32_t startupDelay = 25000;
uint32_t fullChargeDelay = 85000;

uint8_t outputPWM = 0;
double targetVoltage, inputVoltage, targetPWM;

PID myPID(&inputVoltage, &targetPWM, &targetVoltage, pidParameter.kp, pidParameter.ki, pidParameter.kd, P_ON_E, DIRECT);
unsigned long startTime;

void initPID() {
  targetVoltage = 0.0;
  myPID.SetOutputLimits(pidParameter.lowerLimit, pidParameter.upperLimit);
  myPID.SetSampleTime(pidParameter.sampleTime);
  myPID.SetMode(MANUAL);
}

float calculateInputVoltage(int analogRaw) {
  float analogVoltage = (analogRaw * 5.0) / 1024.0;
  return analogVoltage / (r2/(r1+r2));
}

void setup() {
  pinMode(FIELD, OUTPUT);
  pinMode(LIGHT, OUTPUT);
  digitalWrite(FIELD, LOW);
  digitalWrite(LIGHT, LOW);
  startTime = millis();
  initPID();
}

void loop() {
  inputVoltage = calculateInputVoltage(analogRead(A0));

  if(inputVoltage < batteryLightVoltage) {
    digitalWrite(LIGHT, HIGH);
  } else {
    digitalWrite(LIGHT, LOW);
  }

  if(millis() - startTime > fullChargeDelay) {
    targetVoltage = chargeVoltage;
    myPID.SetMode(AUTOMATIC);
  } else if (millis() - startTime > startupDelay) {
    targetVoltage = afterStartVoltage;
    myPID.SetMode(AUTOMATIC);
  } else {
    targetVoltage = 0.0;
    myPID.SetMode(MANUAL);
  }

  myPID.Compute();

  if(inputVoltage > maxVoltage) {
    outputPWM = int(pidParameter.lowerLimit);
  } else {
    outputPWM = int(targetPWM);
  }
  analogWrite(FIELD, outputPWM);

  delay(10);

}