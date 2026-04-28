#include <Wire.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11);

// ───────────── 핀 설정 ─────────────
#define MotorSpeedPin 5
#define CWPin         3
#define CCWPin        4
#define Cds1Pin       A1
#define Cds2Pin       A3

// ───────────── MPU 설정 ─────────────
const uint8_t MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float ax, ay, az, gx, gy, gz;
float ax_f, ay_f, az_f;

// ───────────── 상태 변수 ─────────────
int motorSpeedVal = 0;
int MotorDir = 0;
int Cds1Val = 0, Cds2Val = 0;

// ───────────── 필터 변수 ─────────────
float dt = 0.01f;
float alpha_acc = 0.9f;
float alpha_comp = 0.98f;
float pitch_f = 0.0f;

// ───────────── 조도 기준값 ─────────────
int Bright1lf = 310;   // CdS1 기준값
int Bright2lf = 360;   // CdS2 기준값
const int MIN_PWM = 180;
const int MAX_PWM = 220;

// ───────────── 함수 정의 ─────────────
inline void motorPWM(uint8_t pwm) {
  if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
  analogWrite(MotorSpeedPin, pwm);
  motorSpeedVal = pwm;
}
inline void motorCW()  { digitalWrite(CWPin, HIGH); digitalWrite(CCWPin, LOW);  MotorDir = +1; }
inline void motorCCW() { digitalWrite(CWPin, LOW);  digitalWrite(CCWPin, HIGH); MotorDir = -1; }
inline void motorStop(){ motorPWM(0); MotorDir = 0; }

bool mpuBegin() {
  Wire.begin(); delay(50);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); if (Wire.endTransmission(true)!=0) return false;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); if (Wire.endTransmission(true)!=0) return false;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); if (Wire.endTransmission(true)!=0) return false;
  return true;
}

void mpuReadRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return;
  AcX = (Wire.read()<<8)|Wire.read();
  AcY = (Wire.read()<<8)|Wire.read();
  AcZ = (Wire.read()<<8)|Wire.read();
  (void)((Wire.read()<<8)|Wire.read());
  GyX = (Wire.read()<<8)|Wire.read();
  GyY = (Wire.read()<<8)|Wire.read();
  GyZ = (Wire.read()<<8)|Wire.read();

  ax = (float)AcX / 16384.0f;
  ay = (float)AcY / 16384.0f;
  az = (float)AcZ / 16384.0f;
  gx = (float)GyX / 131.0f;
  gy = (float)GyY / 131.0f;
  gz = (float)GyZ / 131.0f;
}

void updateComplementaryFilter() {
  ax_f = alpha_acc * ax_f + (1.0f - alpha_acc) * ax;
  ay_f = alpha_acc * ay_f + (1.0f - alpha_acc) * ay;
  az_f = alpha_acc * az_f + (1.0f - alpha_acc) * az;

  float pitch_acc = atan2(ay_f, sqrt(ax_f * ax_f + az_f * az_f)) * 180.0f / PI;
  pitch_f = alpha_comp * (pitch_f + gy * dt) + (1.0f - alpha_comp) * pitch_acc;
}

// ───────────── 태양 추적 제어 ─────────────
void LP_MODE() {
  Cds1Val = analogRead(Cds1Pin);
  Cds2Val = analogRead(Cds2Pin);

  // ① 초기상태: 두 센서 다 어두우면 탐색 시작
  if ((motorSpeedVal == 0) && (Cds1Val > Bright1lf && Cds2Val > Bright2lf)) {
    motorCCW();
    motorPWM(200);
  }

  // ② 모터 제어 조건
  if (Cds1Val > Bright1lf && Cds2Val <= Bright2lf) {
    motorCW();
  } 
  else if (Cds1Val <= Bright1lf && Cds2Val > Bright2lf) {
    motorCCW();
  } 
  else {
    motorStop(); // 둘 다 충분히 밝으면 정지
  }

  // ③ PWM 크기 조절
  if (Cds1Val <= Bright1lf || Cds2Val <= Bright2lf) motorSpeedVal = 100;
  else motorSpeedVal = 200;

  motorPWM(motorSpeedVal);
}

// ───────────── 전송 함수 ─────────────
void sendCSV(Stream &out) {
  //조도 센서 값만 뽑는 실험
  out.print(Cds1Val); out.print(',');
  out.print(Cds2Val); out.print(',');
  out.print(MotorDir); out.print(',');
  
  //두번 실험 
  //out.print(); out.print(',');
  //out.print(pitch_f, 2);
  //out.println();
}
void sendTelemetry() {
  sendCSV(Serial);
  sendCSV(BTSerial);
}

// ───────────── Setup & Loop ─────────────
void setup() {
  pinMode(MotorSpeedPin, OUTPUT);
  pinMode(CWPin, OUTPUT);
  pinMode(CCWPin, OUTPUT);
  digitalWrite(CWPin, LOW);
  digitalWrite(CCWPin, LOW);
  motorPWM(0);

  Serial.begin(9600);
  BTSerial.begin(9600);

  if (!mpuBegin()) {
    Serial.println("MPU FAIL");
    BTSerial.println("MPU FAIL");
  } else {
    Serial.println("System Start!");
  }
}

void loop() {
  mpuReadRaw();
  updateComplementaryFilter();

  LP_MODE();        // 태양추적 로직 실행
  sendTelemetry();  // 센서값 및 상태 전송

  delay(50);
}
