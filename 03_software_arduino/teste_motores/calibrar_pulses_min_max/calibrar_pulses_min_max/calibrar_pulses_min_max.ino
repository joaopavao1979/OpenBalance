#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint8_t CHANNEL_X = 0;
const uint8_t CHANNEL_Y = 1;
const int PWM_FREQ = 50; // 50 Hz para servos

// Testa estes valores primeiro
const int PULSE_MIN_US = 1000;
const int PULSE_MAX_US = 2000;

// Converter de microssegundos para valor do PCA9685 (0-4095)
int pulseToPwm(int pulse_us) {
  int period_us = 1000000 / PWM_FREQ;
  return map(pulse_us, 0, period_us, 0, 4095);
}

void setServoPulse(uint8_t channel, int microseconds) {
  pwm.setPWM(channel, 0, pulseToPwm(microseconds));
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  Serial.println("Teste de servo: envia 'min', 'max', ou valor em us (ex: 1234)");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    int pulse = 0;
    if (cmd.equalsIgnoreCase("min")) {
      pulse = PULSE_MIN_US;
    } else if (cmd.equalsIgnoreCase("max")) {
      pulse = PULSE_MAX_US;
    } else {
      pulse = cmd.toInt(); // permite enviar valores à mão, ex: 1500
    }

    if (pulse >= 500 && pulse <= 2500) { // valores razoáveis para a maioria dos servos
      setServoPulse(CHANNEL_X, pulse);
      setServoPulse(CHANNEL_Y, pulse);
      Serial.print("Pulso aplicado: ");
      Serial.println(pulse);
    } else {
      Serial.println("Valor fora do intervalo seguro!");
    }
  }
}
