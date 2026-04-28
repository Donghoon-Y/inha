#include <Wire.h>
#include <SoftwareSerial.h>

/* ===== 핀 ===== */
const int CWPin       = 3;
const int CCWPin      = 4;
const int MotorPWMPin = 5;

/* ===== Bluetooth (HC-05 등) ===== */
SoftwareSerial BT(10, 11);
bool LOG_TO_BT = false;   // 디버깅 때는 false로 두세요

/* ===== MPU6050 ===== */
const uint8_t MPU_ADDR = 0x68;
const float   GYRO_LSB_PER_DPS = 131.0f; // ±250 dps
int16_t AcXr, AcYr, AcZr, GyXr, GyYr, GyZr, Tmpr;
float   gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

/* ===== 제어 축/부호 설정 (런타임 변경 가능) ===== */
char CTRL_AXIS = 'X';     // 'X','Y','Z' 중 선택
int  AXIS_SIGN = +1;      // +1 또는 -1

/* ===== PID/FeedForward 설정 ===== */
float w_ref = 20.0f;                     // 목표 각속도 [dps]
float Kp = 0.05f, Ki = 0.0001f, Kd = 0.05f; //0.0125에서 줄임
float Kff = 1.2f;                        // 피드포워드 (저속 토크 보강)

const int PWM_MAX = 255;                 // 상한 올려 토크 여유 확보
const int PWM_MIN = 0;
int   PWM_MIN_RUN = 70;                  // 데드존 (모터/드라이버별로 다름, 현장 튠)

/* ===== 내부 상태 ===== */
float err_i = 0.0f;
float prev_meas = 0.0f;
float dmeas_f = 0.0f;
const float d_alpha = 0.25f;             // D용 1차 LPF
unsigned long t_prev_ms = 0;
static unsigned long kick_until_ms = 0;  // 스타트킥 타이머

/* ===== 유틸 출력 ===== */
template<typename T> inline void outP(const T& v){ Serial.print(v); if(LOG_TO_BT) BT.print(v); }
template<typename T> inline void outPL(const T& v){ Serial.println(v); if(LOG_TO_BT) BT.println(v); }
inline void outPF(float v,int d){ Serial.print(v,d); if(LOG_TO_BT) BT.print(v,d); }

/* ===== 모터 제어 ===== */
inline void motor_setPWM(int u){
  u = constrain(u, PWM_MIN, PWM_MAX);
  analogWrite(MotorPWMPin, u);
}
inline void motor_CW(){  digitalWrite(CWPin,HIGH); digitalWrite(CCWPin,LOW); }
inline void motor_CCW(){ digitalWrite(CWPin,LOW);  digitalWrite(CCWPin,HIGH); }
inline void motor_stop(){ digitalWrite(CWPin,LOW); digitalWrite(CCWPin,LOW); motor_setPWM(0); err_i = 0.0f; }

/* ===== MPU 레지스터 쓰기 ===== */
void mpu_write(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission(true);
}

/* ===== MPU 초기화 ===== */
void mpu_init(){
  Wire.begin();
  delay(50);
  mpu_write(0x6B, 0x01);  // PWR_MGMT_1: PLL with X gyro ref
  mpu_write(0x1A, 0x03);  // CONFIG: DLPF_CFG=3 (~44Hz)
  mpu_write(0x1B, 0x00);  // GYRO_CONFIG: ±250dps
  mpu_write(0x1C, 0x00);  // ACCEL_CONFIG: ±2g
  delay(50);
}

/* ===== 자이로 바이어스 캘리브레이션 ===== */
void mpu_calibrate_gyro(uint16_t N=500){
  long sumX=0, sumY=0, sumZ=0;
  for(uint16_t i=0;i<N;i++){
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);
    int16_t ax = (Wire.read()<<8)|Wire.read();
    int16_t ay = (Wire.read()<<8)|Wire.read();
    int16_t az = (Wire.read()<<8)|Wire.read();
    int16_t tp = (Wire.read()<<8)|Wire.read();
    int16_t gx = (Wire.read()<<8)|Wire.read();
    int16_t gy = (Wire.read()<<8)|Wire.read();
    int16_t gz = (Wire.read()<<8)|Wire.read();
    sumX += gx; sumY += gy; sumZ += gz;
    delay(2);
  }
  gyroBiasX = (float)sumX / (float)N;
  gyroBiasY = (float)sumY / (float)N;
  gyroBiasZ = (float)sumZ / (float)N;

  outPL("Gyro bias calibrated:");
  outP("bx="); outPF(gyroBiasX,1);
  outP(", by="); outPF(gyroBiasY,1);
  outP(", bz="); outPF(gyroBiasZ,1); outPL("");
}

/* ===== 14바이트 RAW 읽기 & 축 재배치 (필요하면 조정) ===== */
void mpu_readRaw(){
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t tmpAcX = (Wire.read()<<8)|Wire.read();
  int16_t tmpAcY = (Wire.read()<<8)|Wire.read();
  int16_t tmpAcZ = (Wire.read()<<8)|Wire.read();
  Tmpr = (Wire.read()<<8)|Wire.read();
  int16_t tmpGyX = (Wire.read()<<8)|Wire.read();
  int16_t tmpGyY = (Wire.read()<<8)|Wire.read();
  int16_t tmpGyZ = (Wire.read()<<8)|Wire.read();

  // 보드 장착 방향에 맞춰 필요 시 바꾸세요
  AcXr = tmpAcZ;  AcYr = tmpAcY;  AcZr = tmpAcX;
  GyXr = tmpGyZ;  GyYr = tmpGyY;  GyZr = tmpGyX;
}

/* ===== 현재 측정 각속도(dps) 계산 (축/부호 적용) ===== */
float get_w_meas_dps(){
  float gx = ((float)GyXr - gyroBiasX) / GYRO_LSB_PER_DPS;
  float gy = ((float)GyYr - gyroBiasY) / GYRO_LSB_PER_DPS;
  float gz = ((float)GyZr - gyroBiasZ) / GYRO_LSB_PER_DPS;

  float w = 0.0f;
  switch(CTRL_AXIS){
    case 'X': w = gx; break;
    case 'Y': w = gy; break;
    case 'Z': w = gz; break;
    default:  w = gx; break;
  }
  return AXIS_SIGN * w;
}

/* ===== PID 스텝 ===== */
void pid_step(float dt){
  mpu_readRaw();

  float w_meas = get_w_meas_dps();     // 선택축/부호 반영
  float err = w_ref - w_meas;

  // 적분 + 안티윈드업
  err_i += err * dt;
  float i_lim = (Ki > 1e-6f) ? (PWM_MAX / Ki) : 0.0f;
  if(i_lim > 0.0f) err_i = constrain(err_i, -i_lim, i_lim);

  // 미분(측정값 미분) + 저역통과
  float dmeas = (w_meas - prev_meas) / dt;
  dmeas_f = d_alpha * dmeas + (1.0f - d_alpha) * dmeas_f;

  // PID + 피드포워드
  float u = Kp*err + Ki*err_i - Kd*dmeas_f + Kff*w_ref;

  // PWM 변환 + 데드존 보정
  float mag = fabs(u);
  int pwm = 0;

  if (fabs(w_ref) >= 1.0f) {
    if (mag < 1.0f) pwm = PWM_MIN_RUN; else pwm = (int)mag;
    pwm = max(pwm, PWM_MIN_RUN);
    pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  } else pwm = 0;

  // 스타트 킥 (정지 상태에서 잠깐 강하게)
  if (fabs(w_ref) >= 5.0f && fabs(prev_meas) < 2.0f) {
    if (millis() < kick_until_ms) {
      pwm = max(pwm, PWM_MIN_RUN + 25);
    } else if (pwm >= PWM_MIN_RUN) {
      kick_until_ms = millis() + 200; // 0.2초 킥
    }
  }

  // 모터 구동 (u의 부호 = 원하는 방향)
  if(fabs(w_ref) < 1.0f){
    motor_stop();
  } else {
    if(u >= 0){ motor_CW();  motor_setPWM(pwm); }
    else       { motor_CCW(); motor_setPWM(pwm); }
  }

  prev_meas = w_meas;

  // 디버그
  outP("axis="); outP(CTRL_AXIS);
  outP(", sign="); outP(AXIS_SIGN>0?"+":"-");
  outP(", w_ref="); outPF(w_ref,1);
  outP(", w_meas="); outPF(w_meas,1);
  outP(", err="); outPF(err,1);
  outP(", u="); outPF(u,1);
  outP(", pwm="); outP(pwm);
  outP(", dir="); outPL(u>=0?"CW":"CCW");
}

/* ===== 명령 파서 =====
   S<val>  : w_ref 설정 (예: S20)
   P/I/D   : PID 게인
   F<val>  : Kff
   B0/1    : BT 로그 끄기/켜기
   X/Y/Z   : 제어축 선택
   R       : 부호 반전 (AXIS_SIGN *= -1)
   G       : 현재 3축 자이로(dps) 한 줄 출력
   Q       : 현재 설정 조회
*/
void parse_stream(Stream &S){
  static char buf[24]; static uint8_t i=0;
  while(S.available()){
    char c=S.read();
    if(c=='\n'||c=='\r'){
      buf[i]='\0';
      if(i>0){
        switch(buf[0]){
          case 'S': w_ref=atof(buf+1); outP("SET w_ref="); outPL(w_ref); break;
          case 'P': Kp=atof(buf+1); outP("SET Kp="); outPL(Kp); break;
          case 'I': Ki=atof(buf+1); outP("SET Ki="); outPL(Ki); break;
          case 'D': Kd=atof(buf+1); outP("SET Kd="); outPL(Kd); break;
          case 'F': Kff=atof(buf+1); outP("SET Kff="); outPL(Kff); break;
          case 'B': LOG_TO_BT = (atoi(buf+1)!=0); outP("LOG_TO_BT="); outPL(LOG_TO_BT); break;
          case 'X': CTRL_AXIS='X'; outPL("Axis=X"); break;
          case 'Y': CTRL_AXIS='Y'; outPL("Axis=Y"); break;
          case 'Z': CTRL_AXIS='Z'; outPL("Axis=Z"); break;
          case 'R': AXIS_SIGN*=-1; outP("Axis sign flipped -> "); outPL(AXIS_SIGN>0?"+":"-"); break;
          case 'G': {
            float gx = ((float)GyXr - gyroBiasX) / GYRO_LSB_PER_DPS;
            float gy = ((float)GyYr - gyroBiasY) / GYRO_LSB_PER_DPS;
            float gz = ((float)GyZr - gyroBiasZ) / GYRO_LSB_PER_DPS;
            outP("Gy[dps] X="); outPF(gx,1);
            outP(", Y="); outPF(gy,1);
            outP(", Z="); outPF(gz,1); outPL("");
          } break;
          case 'Q':
            outP("Axis="); outP(CTRL_AXIS);
            outP(", Sign="); outP(AXIS_SIGN>0?"+":"-");
            outP(", Kp="); outPF(Kp,3);
            outP(", Ki="); outPF(Ki,3);
            outP(", Kd="); outPF(Kd,3);
            outP(", Kff="); outPF(Kff,2);
            outP(", w_ref="); outPF(w_ref,1);
            outP(", PWM_MIN_RUN="); outP(PWM_MIN_RUN);
            outP(", PWM_MAX="); outPL(PWM_MAX);
            break;
          default: outP("ERR cmd: "); outPL(buf);
        }
      }
      i=0;
    } else { if(i<sizeof(buf)-1) buf[i++]=c; }
  }
}

/* ===== setup / loop ===== */
void setup(){
  pinMode(CWPin, OUTPUT); pinMode(CCWPin, OUTPUT); pinMode(MotorPWMPin, OUTPUT);
  motor_stop();

  Serial.begin(9600);
  BT.begin(9600);

  mpu_init();
  outPL("Calibrating gyro... Keep still");
  mpu_calibrate_gyro(500);
  outPL("PID-Gyro ready. Cmd: S<dps>, P/I/D/F<k>, X/Y/Z, R, G, Q, B0/1");
  t_prev_ms = millis();
}

void loop(){
  parse_stream(Serial); parse_stream(BT);

  unsigned long now = millis();
  float dt = (now - t_prev_ms)/1000.0f;
  if(dt >= 0.01f){       // 10ms 이상 됐을 때만 갱신
    t_prev_ms = now;
    pid_step(dt);
  }
}
