#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =====================
// OpenBalance Arduino – PID com Calibração Física
// Versão 2.3: Ganhos reajustados e OUT_SCALE = 1
// =====================

// === DADOS DE CALIBRAÇÃO (preencha após montagem) ===
// Ângulo de Equilíbrio X: 88
// Ângulo Mínimo       X: 60
// Ângulo Máximo       X: 102
const int ANG_EQUIL_X = 88;
const int ANG_MIN_X   = 60;
const int ANG_MAX_X   = 102;

// Ângulo de Equilíbrio Y: 75
// Ângulo Mínimo       Y: 60
// Ângulo Máximo       Y: 100
const int ANG_EQUIL_Y = 75;
const int ANG_MIN_Y   = 60;
const int ANG_MAX_Y   = 100;

// Pulsos (µs) mínimos e máximos
const int PULSE_MIN_US = 1000;
const int PULSE_MAX_US = 2000;

// Canais PCA9685
const uint8_t CHANNEL_X = 0;
const uint8_t CHANNEL_Y = 1;

// Frequência do PCA9685
const int PWM_FREQ = 100;  // 100 Hz para servos digitais

// Variáveis derivadas da calibração
int pulseCenterX, pulseMinX, pulseMaxX;
int pulseCenterY, pulseMinY, pulseMaxY;

// Resolução PCA9685 (0–4095) ajustada ao período
int PWM_MIN, PWM_MAX;

// Ganhos PID reajustados para resposta moderada
const float Kp = 1.2;
const float Ki = 0.02;
const float Kd = 0.05;
// Sem factor de escala adicional
const float OUT_SCALE = 1.0;

// Estados do PID
float errX = 0, errY = 0;
float prevErrX = 0, prevErrY = 0;
float sumErrX  = 0, sumErrY  = 0;

bool motorsEnabled = false;
unsigned long lastTime = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  // Calcular pulsos de centro/min/max a partir dos ângulos
  pulseCenterX = map(ANG_EQUIL_X, ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);
  pulseMinX    = map(ANG_MIN_X,   ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxX    = map(ANG_MAX_X,   ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);

  pulseCenterY = map(ANG_EQUIL_Y, ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);
  pulseMinY    = map(ANG_MIN_Y,   ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxY    = map(ANG_MAX_Y,   ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);

  // Período em µs de cada ciclo: 20000 µs / PWM_FREQ
  int periodUs = 20000 / PWM_FREQ;
  PWM_MIN = map(PULSE_MIN_US, 0, periodUs, 0, 4095);
  PWM_MAX = map(PULSE_MAX_US, 0, periodUs, 0, 4095);

  // Centrar servos
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
  }
  else if (line.startsWith("M0")) {
    motorsEnabled = false;
    resetPID();
  }
  else if (line.startsWith("E,")) {
    // E,errX,errY
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1 + 1);
    errX = constrain(line.substring(i1 + 1, i2).toFloat(),
                     - (pulseCenterX - pulseMinX),
                     + (pulseMaxX   - pulseCenterX));
    errY = constrain(line.substring(i2 + 1).toFloat(),
                     - (pulseCenterY - pulseMinY),
                     + (pulseMaxY   - pulseCenterY));
    if (motorsEnabled) applyPID();
  }
}

void applyPID() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0 || dt > 1.0) return;

  // Integral + anti-windup
  sumErrX += errX * dt;
  sumErrY += errY * dt;
  sumErrX = constrain(sumErrX, -300, 300);
  sumErrY = constrain(sumErrY, -300, 300);

  // Derivativo
  float dErrX = (errX - prevErrX) / dt;
  float dErrY = (errY - prevErrY) / dt;

  // PID
  float outX = Kp * errX + Ki * sumErrX + Kd * dErrX;
  float outY = Kp * errY + Ki * sumErrY + Kd * dErrY;

  // Sem escala extra
  outX *= OUT_SCALE;
  outY *= OUT_SCALE;

  prevErrX = errX;
  prevErrY = errY;

  // Calcula pulso final e aplica limites
  int pulseX = constrain(pulseCenterX + int(outX), pulseMinX, pulseMaxX);
  int pulseY = constrain(pulseCenterY + int(outY), pulseMinY, pulseMaxY);

  setServoPulse(CHANNEL_X, pulseX);
  setServoPulse(CHANNEL_Y, pulseY);

  // Debug via Serial
  Serial.print("SX:"); Serial.print(pulseX);
  Serial.print(" SY:"); Serial.println(pulseY);
}

void resetPID() {
  sumErrX   = 0;
  sumErrY   = 0;
  prevErrX  = 0;
  prevErrY  = 0;
}

void setServoPulse(uint8_t channel, int microseconds) {
  int periodUs = 20000 / PWM_FREQ;
  int pwmVal = map(microseconds, 0, periodUs, 0, 4095);
  pwmVal = constrain(pwmVal, PWM_MIN, PWM_MAX);
  pwm.setPWM(channel, 0, pwmVal);
}
