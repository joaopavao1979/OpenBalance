#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =====================
// OpenBalance Arduino - PCA9685 Version
// PID on-board, receives error from dashboard
// =====================

// PCA9685 driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Endereço padrão

// Servo shield channels
const int CHANNEL_X = 0;
const int CHANNEL_Y = 1;

// Pulsos (em microssegundos)
const int PULSE_CENTER = 1500;
const int PULSE_MIN = 1000;
const int PULSE_MAX = 2000;

// PWM resolution (12 bits → 4096 passos)
const int PWM_FREQ = 50; // Hz
const int PWM_MIN = map(PULSE_MIN, 0, 20000, 0, 4095); // mapeamento para 50 Hz
const int PWM_MAX = map(PULSE_MAX, 0, 20000, 0, 4095);

// PID gains
const float Kp = 1.2;
const float Ki = 0.02;
const float Kd = 0.05;

// Estado
float errX = 0, errY = 0;
float prevErrX = 0, prevErrY = 0;
float sumErrX = 0, sumErrY = 0;

int xOffset = 0;
int yOffset = 0;

bool motorsEnabled = false;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  delay(10); // aguarda estabilidade

  // Centra os servos
  setServoPulse(CHANNEL_X, PULSE_CENTER + xOffset);
  setServoPulse(CHANNEL_Y, PULSE_CENTER + yOffset);

  lastTime = millis();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() < 1) return;

    if (line.startsWith("M1")) {
      motorsEnabled = true;
    } else if (line.startsWith("M0")) {
      motorsEnabled = false;
    } else if (line.startsWith("C,")) {
      int idx1 = line.indexOf(',');
      int idx2 = line.indexOf(',', idx1 + 1);
      if (idx1 > 0 && idx2 > idx1) {
        int px = line.substring(idx1 + 1, idx2).toInt();
        int py = line.substring(idx2 + 1).toInt();
        xOffset = px - PULSE_CENTER;
        yOffset = py - PULSE_CENTER;
        Serial.print("CALIB_OK,");
        Serial.print(xOffset);
        Serial.print(",");
        Serial.println(yOffset);
      }
    } else if (line.startsWith("E,")) {
      int idx1 = line.indexOf(',');
      int idx2 = line.indexOf(',', idx1 + 1);
      if (idx1 > 0 && idx2 > idx1) {
        errX = line.substring(idx1 + 1, idx2).toFloat();
        errY = line.substring(idx2 + 1).toFloat();
        if (motorsEnabled) applyPID();
      }
    }
  }
}

void applyPID() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0) return;

  // PID
  sumErrX += errX * dt;
  sumErrY += errY * dt;

  float dErrX = (errX - prevErrX) / dt;
  float dErrY = (errY - prevErrY) / dt;

  float outX = Kp * errX + Ki * sumErrX + Kd * dErrX;
  float outY = Kp * errY + Ki * sumErrY + Kd * dErrY;

  prevErrX = errX;
  prevErrY = errY;

  int pulseX = constrain(PULSE_CENTER + xOffset + int(outX), PULSE_MIN, PULSE_MAX);
  int pulseY = constrain(PULSE_CENTER + yOffset + int(outY), PULSE_MIN, PULSE_MAX);

  setServoPulse(CHANNEL_X, pulseX);
  setServoPulse(CHANNEL_Y, pulseY);

  Serial.print("SX:"); Serial.print(pulseX);
  Serial.print(" SY:"); Serial.println(pulseY);
}

void setServoPulse(uint8_t channel, int microseconds) {
  // Convert microseconds to PCA9685 pulse length (0-4095)
  int pwmVal = map(microseconds, 0, 20000, 0, 4095);
  pwmVal = constrain(pwmVal, PWM_MIN, PWM_MAX);
  pwm.setPWM(channel, 0, pwmVal);
}
