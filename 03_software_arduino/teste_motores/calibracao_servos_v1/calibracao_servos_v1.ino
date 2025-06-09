#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// -----------------------------------------------------------------------------
// Configuração do PCA9685 (DollaTek 16-ch PWM Servo Driver)
// -----------------------------------------------------------------------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ       50      // frequência de PWM para servos (50 Hz)
#define SERVOMIN         115     // valor mínimo de PWM (tick PCA9685) ≈  1.0 ms
#define SERVOMAX         490     // valor máximo de PWM (tick PCA9685) ≈  2.0 ms

// Canais do PCA9685 onde os servos X e Y estão conectados
#define SERVO_X_CHANNEL  0
#define SERVO_Y_CHANNEL  1

void setup() {
  // Inicializa comunicação serial
  Serial.begin(9600);
  while (!Serial) { /* aguarda porta Serial habilitar (apenas em alguns Arduinos) */ }

  // Inicializa PCA9685
  pwm.begin();
  // Se quiser ajustar o clock interno (opcional):
  // pwm.setOscillatorFrequency(27000000);  // método correto
  pwm.setPWMFreq(SERVO_FREQ);     // configura para 50 Hz

  // Posição neutra inicial (90°) para ambos os servos
  setServoAngle(SERVO_X_CHANNEL, 90);
  setServoAngle(SERVO_Y_CHANNEL, 90);

  // Mensagem de boas-vindas
  Serial.println();
  Serial.println("=== Calibracao de Servos via PCA9685 ===");
  Serial.println("Comandos: X<ang> ou Y<ang> onde <ang> de 0 a 180");
  Serial.println("Exemplos: X0   X90   X180   Y45");
  Serial.println("----------------------------------------");
}

void loop() {
  if (Serial.available()) {
    String linha = Serial.readStringUntil('\n');
    linha.trim();  // remove espaços e CR/LF

    if (linha.length() < 2) {
      Serial.println("Comando invalido. Use X<0-180> ou Y<0-180>");
      return;
    }

    char eixo = toupper(linha.charAt(0));    // 'X' ou 'Y'
    int angulo = linha.substring(1).toInt(); // converte resto para inteiro

    // Limita o ângulo ao intervalo [0, 180]
    angulo = constrain(angulo, 0, 180);

    if (eixo == 'X') {
      setServoAngle(SERVO_X_CHANNEL, angulo);
      Serial.print("Servo X -> ");
      Serial.print(angulo);
      Serial.println(" graus");
    }
    else if (eixo == 'Y') {
      setServoAngle(SERVO_Y_CHANNEL, angulo);
      Serial.print("Servo Y -> ");
      Serial.print(angulo);
      Serial.println(" graus");
    }
    else {
      Serial.println("Eixo desconhecido. Use 'X' ou 'Y'.");
    }
  }
}

/**
 * Converte ângulo [0..180] em valor de PWM no PCA9685 e envia comando.
 *
 * @param channel Canal do PCA9685 (0..15)
 * @param angle   Ângulo desejado (0..180°)
 */
void setServoAngle(uint8_t channel, uint8_t angle) {
  // Mapeia 0..180° para SERVOMIN..SERVOMAX ticks
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}
