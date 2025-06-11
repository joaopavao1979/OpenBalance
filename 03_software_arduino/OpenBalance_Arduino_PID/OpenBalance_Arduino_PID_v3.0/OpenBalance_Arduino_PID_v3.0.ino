#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =====================
// OpenBalance Arduino – Versão Atualizada com Calibração Física
// PID onboard, recebe erro da dashboard
// =====================

// === PARÂMETROS DE CALIBRAÇÃO (preencha após montagem) ===
const int ANG_EQUIL_X  =  88;   // ângulo de equilíbrio X
const int ANG_MIN_X    =  60;   // ângulo mínimo físico X
const int ANG_MAX_X    = 102;   // ângulo máximo físico X

const int ANG_EQUIL_Y  =  75;   // ângulo de equilíbrio Y
const int ANG_MIN_Y    =  60;   // ângulo mínimo físico Y
const int ANG_MAX_Y    = 100;   // ângulo máximo físico Y

// Pulsos servo (μs) limite genérico
const int PULSE_MIN_US = 1000;
const int PULSE_MAX_US = 2000;

// Canais PCA9685
const uint8_t CHANNEL_X = 0;
const uint8_t CHANNEL_Y = 1;

// Frequência do PWM
const int PWM_FREQ = 100;  // Hz (100 Hz recomendado para servos digitais)

// Ganhos PID (ajuste conforme necessário)
const float Kp = 1.2;
const float Ki = 0.02;
const float Kd = 0.05;

// Limites anti-windup
const float SUM_MAX = 300.0;

// Derivativo - filtro alpha
const float D_ALPHA = 0.8;

// ===== Variáveis derivadas de calibração =====
int pulseMinX, pulseCenterX, pulseMaxX;
int pulseMinY, pulseCenterY, pulseMaxY;
int PWM_MIN, PWM_MAX;

// ===== Estado do PID =====
float errX = 0, errY = 0;
float prevErrX = 0, prevErrY = 0;
float sumErrX  = 0, sumErrY  = 0;
float dErrXf   = 0, dErrYf   = 0;

bool motorsEnabled = false;
unsigned long lastTime = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  // === Calcula pulsos a partir dos ângulos calibrados ===
  pulseMinX    = map(ANG_MIN_X,   ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);
  pulseCenterX = map(ANG_EQUIL_X, ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxX    = map(ANG_MAX_X,   ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);

  pulseMinY    = map(ANG_MIN_Y,   ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);
  pulseCenterY = map(ANG_EQUIL_Y, ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxY    = map(ANG_MAX_Y,   ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);

  // === Calcula limites PCA9685 (0–4095) com período dinâmico ===
  int periodUs = 1000000 / PWM_FREQ;  // µs por ciclo
  PWM_MIN = map(PULSE_MIN_US, 0, periodUs, 0, 4095);
  PWM_MAX = map(PULSE_MAX_US, 0, periodUs, 0, 4095);

  // Centra servos
  setServoPulse(CHANNEL_X, pulseCenterX);
  setServoPulse(CHANNEL_Y, pulseCenterY);

  lastTime = millis();
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() < 1) return;

  if (line.startsWith("M1")) {
    motorsEnabled = true;
    Serial.println("MOTORS_ON");
  }
  else if (line.startsWith("M0")) {
    motorsEnabled = false;
    resetPID();
    Serial.println("MOTORS_OFF");
  }
  else if (line.startsWith("C,")) {
    // opcional: calibração via dashboard
  }
  else if (line.startsWith("E,")) {
    // Formato: E,errX,errY
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1 + 1);
    if (i1 > 0 && i2 > i1) {
      errX = constrain(line.substring(i1+1, i2).toFloat(),
                       -(pulseCenterX - pulseMinX),
                        (pulseMaxX   - pulseCenterX));
      errY = constrain(line.substring(i2+1).toFloat(),
                       -(pulseCenterY - pulseMinY),
                        (pulseMaxY   - pulseCenterY));
      if (motorsEnabled) applyPID();
    }
  }
}

void applyPID() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0 || dt > 0.5) return;  // descarta ciclos anormais

  // Integral + anti-windup
  sumErrX += errX * dt;
  sumErrY += errY * dt;
  sumErrX = constrain(sumErrX, -SUM_MAX, SUM_MAX);
  sumErrY = constrain(sumErrY, -SUM_MAX, SUM_MAX);

  // Derivativo bruto
  float rawDX = (errX - prevErrX) / dt;
  float rawDY = (errY - prevErrY) / dt;
  // Filtro exponencial do derivativo
  dErrXf = D_ALPHA * dErrXf + (1 - D_ALPHA) * rawDX;
  dErrYf = D_ALPHA * dErrYf + (1 - D_ALPHA) * rawDY;

  // PID
  float outX = Kp * errX + Ki * sumErrX + Kd * dErrXf;
  float outY = Kp * errY + Ki * sumErrY + Kd * dErrYf;

  prevErrX = errX;
  prevErrY = errY;

  // Constrói pulso final e envia ao servo
  int pulseX = constrain(pulseCenterX + int(outX), pulseMinX, pulseMaxX);
  int pulseY = constrain(pulseCenterY + int(outY), pulseMinY, pulseMaxY);

  setServoPulse(CHANNEL_X, pulseX);
  setServoPulse(CHANNEL_Y, pulseY);

  // Debug
  Serial.print("PID_ON SX:"); Serial.print(pulseX);
  Serial.print(" SY:"); Serial.println(pulseY);
}

void resetPID() {
  sumErrX  = 0;
  sumErrY  = 0;
  prevErrX = 0;
  prevErrY = 0;
  dErrXf   = 0;
  dErrYf   = 0;
}

void setServoPulse(uint8_t channel, int microseconds) {
  int periodUs = 1000000 / PWM_FREQ;
  int pwmVal = map(microseconds, 0, periodUs, 0, 4095);
  pwmVal = constrain(pwmVal, PWM_MIN, PWM_MAX);
  pwm.setPWM(channel, 0, pwmVal);
}
