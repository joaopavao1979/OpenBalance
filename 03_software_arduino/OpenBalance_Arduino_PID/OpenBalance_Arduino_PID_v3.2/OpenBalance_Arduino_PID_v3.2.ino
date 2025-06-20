#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =====================
// OpenBalance Arduino – Versão Adaptada
// PID onboard, recebe erro da dashboard
// =====================

// === PARÂMETROS DE CALIBRAÇÃO (preencha após montagem) ===
const int ANG_EQUIL_X  =  75;
const int ANG_MIN_X    =  60;
const int ANG_MAX_X    =  100;

const int ANG_EQUIL_Y  =  70;
const int ANG_MIN_Y    =  60;
const int ANG_MAX_Y    =  90;

const int PULSE_MIN_US = 1000;
const int PULSE_MAX_US = 2000;

// Canais PCA9685
const uint8_t CHANNEL_X = 0;
const uint8_t CHANNEL_Y = 1;
const int PWM_FREQ = 50;

// Ganhos PID (ajuste conforme necessário)
const float Kp_X = 0.3, Ki_X = 0.001, Kd_X = 0.2;
const float Kp_Y = 0.15, Ki_Y = 0.02, Kd_Y = 0.2;
const float SUM_MAX = 300.0;
const float D_ALPHA = 0.8;

// ===== Variáveis derivadas de calibração =====
int pulseMinX, pulseCenterX, pulseMaxX;
int pulseMinY, pulseCenterY, pulseMaxY;
int PWM_MIN, PWM_MAX;

// ===== Estado PID e servos =====
float errX = 0, errY = 0;
float prevErrX = 0, prevErrY = 0;
float sumErrX = 0, sumErrY = 0;
float dErrXf = 0, dErrYf = 0;
bool motorsEnabled = false;
unsigned long lastTime = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  pulseMinX    = map(ANG_MIN_X,   ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);
  pulseCenterX = map(ANG_EQUIL_X, ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxX    = map(ANG_MAX_X,   ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);
  pulseMinY    = map(ANG_MIN_Y,   ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);
  pulseCenterY = map(ANG_EQUIL_Y, ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxY    = map(ANG_MAX_Y,   ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);

  int periodUs = 1000000 / PWM_FREQ;
  PWM_MIN = map(PULSE_MIN_US, 0, periodUs, 0, 4095);
  PWM_MAX = map(PULSE_MAX_US, 0, periodUs, 0, 4095);

  setServoPulse(CHANNEL_X, pulseCenterX);
  setServoPulse(CHANNEL_Y, pulseCenterY);

  lastTime = millis();
}

void loop() {
  // --- LEITURA SERIAL RÁPIDA E SEGURA ---
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() < 1) continue;

    if (line.startsWith("M1")) {
      motorsEnabled = true;
      Serial.println("MOTORS_ON");
    }
    else if (line.startsWith("M0")) {
      motorsEnabled = false;
      resetPID();
      Serial.println("MOTORS_OFF");
    }
    else if (line.startsWith("E,")) {
      // Processamento do erro enviado pela dashboard
      int i1 = line.indexOf(',');
      int i2 = line.indexOf(',', i1 + 1);
      if (i1 > 0 && i2 > i1) {
        errX = constrain(line.substring(i1+1, i2).toFloat(),
                         -(pulseCenterX - pulseMinX),
                          (pulseMaxX   - pulseCenterX));
        errY = constrain(line.substring(i2+1).toFloat(),
                         -(pulseCenterY - pulseMinY),
                          (pulseMaxY   - pulseCenterY));
        // Cálculo PID só quando recebe comando e motores ligados
        if (motorsEnabled) applyPID();
      }
    }
    // Podes adicionar mais comandos aqui
  }
}

// --- CONTROLO PID ---
void applyPID() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (dt <= 0.001 || dt > 0.5) return;

  // Eixo X
  sumErrX += errX * dt;
  sumErrX = constrain(sumErrX, -SUM_MAX, SUM_MAX);
  float rawDX = (errX - prevErrX) / dt;
  dErrXf = D_ALPHA * dErrXf + (1 - D_ALPHA) * rawDX;
  float outX = (Kp_X * errX) + (Ki_X * sumErrX) + (Kd_X * dErrXf);
  prevErrX = errX;
  int pulseX = constrain(pulseCenterX + int(outX), pulseMinX, pulseMaxX);
  setServoPulse(CHANNEL_X, pulseX);

  // Eixo Y
  sumErrY += errY * dt;
  sumErrY = constrain(sumErrY, -SUM_MAX, SUM_MAX);
  float rawDY = (errY - prevErrY) / dt;
  dErrYf = D_ALPHA * dErrYf + (1 - D_ALPHA) * rawDY;
  float outY = ((Kp_Y * errY) + (Ki_Y * sumErrY) + (Kd_Y * dErrYf));
  prevErrY = errY;
  int pulseY = constrain(pulseCenterY + int(outY), pulseMinY, pulseMaxY);
  setServoPulse(CHANNEL_Y, pulseY);

  // Debug opcional:
  // Serial.print("PID_X:"); Serial.print(pulseX);
  // Serial.print(" PID_Y:"); Serial.println(pulseY);
}

// --- RESET PID ---
void resetPID() {
  sumErrX  = 0; sumErrY  = 0;
  prevErrX = 0; prevErrY = 0;
  dErrXf   = 0; dErrYf   = 0;
}

// --- SERVO CONTROL ---
void setServoPulse(uint8_t channel, int microseconds) {
  int periodUs = 1000000 / PWM_FREQ;
  int pwmVal = map(microseconds, 0, periodUs, 0, 4095);
  pwmVal = constrain(pwmVal, PWM_MIN, PWM_MAX);
  pwm.setPWM(channel, 0, pwmVal);
}
