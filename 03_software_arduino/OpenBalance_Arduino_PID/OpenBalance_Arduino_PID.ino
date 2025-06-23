/*
===============================================================================
Projeto: OpenBalance - Sistema de Equilíbrio de Bola com Controlo PID
Ficheiro: OpenBalance_Arduino_PID.ino
Descrição: Código para Arduino que controla dois servomotores via PCA9685.
           Implementa controlo PID por eixo com receção de erro via serial.
Autor: João Pavão
Data: 22/06/2025
Disciplina: Laboratório de Aplicações em Robótica e Aprendizagem (PRIA)
Instituição: Universidade dos Açores
Versão: v1.0
Licença: MIT License
===============================================================================
Notas:
- Este ficheiro integra o sistema físico de controlo da plataforma OpenBalance.
- Os parâmetros PID são configuráveis e o código está preparado para testes em tempo real.
- Comentários adicionais no código indicam sugestões de afinação prática (Kp, Ki, Kd).
===============================================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>  // Biblioteca para controlo do PCA9685 (servos PWM)

// === PARÂMETROS DE CALIBRAÇÃO (definidos com base na montagem física) ===
const int ANG_EQUIL_X  =  90;  // Ângulo neutro para eixo X (posição de equilíbrio)
const int ANG_MIN_X    =  75;  // Ângulo mínimo permitido para o servo X
const int ANG_MAX_X    = 105;  // Ângulo máximo permitido para o servo X

const int ANG_EQUIL_Y  =  77;  // Ângulo neutro para eixo Y
const int ANG_MIN_Y    =  65;  // Ângulo mínimo permitido para o servo Y
const int ANG_MAX_Y    =  95;  // Ângulo máximo permitido para o servo Y

const int PULSE_MIN_US = 1000; // Pulso PWM correspondente ao ângulo mínimo (1 ms)
const int PULSE_MAX_US = 2000; // Pulso PWM correspondente ao ângulo máximo (2 ms)

// === Canais do PCA9685 usados para cada eixo ===
const uint8_t CHANNEL_X = 0;
const uint8_t CHANNEL_Y = 1;
const int PWM_FREQ = 50; // Frequência típica para servos (50 Hz)

// ===== Parâmetros PID por eixo, obtidos pelo método ZN ======
// --------------EIXO X----------------------------------------
const float Kp_X = 0.18, Ki_X = 0.015, Kd_X = 0.07;
// Sugestão: aumenta Kp_X para resposta mais rápida. 
//Se houver oscilações, aumenta Kd_X ou reduz Kp_X.

// --------------EIXO Y----------------------------------------
const float Kp_Y = 0.14, Ki_Y = 0.015, Kd_Y = 0.05;
// Sugestão: se o eixo Y parecer mais lento, 
//testa aumentar Ki_Y para 0.02 ou subir Kp_Y ligeiramente.
// ============================================================

const float SUM_MAX = 300.0;  // Limite para o termo integral
const float D_ALPHA = 0.8;    // Fator de suavização do termo derivativo (filtro exponencial)

// === Variáveis de mapeamento PWM baseadas na calibração ===
int pulseMinX, pulseCenterX, pulseMaxX;
int pulseMinY, pulseCenterY, pulseMaxY;
int PWM_MIN, PWM_MAX;

// === Variáveis de estado PID e controlo ===
float errX = 0, errY = 0;
float prevErrX = 0, prevErrY = 0;
float sumErrX = 0, sumErrY = 0;
float dErrXf = 0, dErrYf = 0;
bool motorsEnabled = false;
unsigned long lastTime = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
// Instancia o objeto 'pwm' para controlar o módulo PCA9685 no endereço I2C 0x40 (endereço padrão).
// Este driver permite gerar até 16 sinais PWM com resolução de 12 bits, ideal para servos.
// Nota: o endereço pode ser alterado fisicamente no módulo ligando os pinos A0–A5.


void setup() {
  Serial.begin(115200);     // Inicia a comunicação serial com o PC a 115200 bps
  pwm.begin();              // Inicializa o PCA9685 com o endereço I2C definido
  pwm.setPWMFreq(PWM_FREQ); // Define a frequência de operação dos servos (tipicamente 50 Hz)
  delay(10);                // Pequena pausa para estabilização
  
  // Converte os ângulos físicos dos servos em pulsos PWM (em microssegundos)
  pulseMinX    = map(ANG_MIN_X,   ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);
  pulseCenterX = map(ANG_EQUIL_X, ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxX    = map(ANG_MAX_X,   ANG_MIN_X, ANG_MAX_X, PULSE_MIN_US, PULSE_MAX_US);

  pulseMinY    = map(ANG_MIN_Y,   ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);
  pulseCenterY = map(ANG_EQUIL_Y, ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxY    = map(ANG_MAX_Y,   ANG_MIN_Y, ANG_MAX_Y, PULSE_MIN_US, PULSE_MAX_US);

  // Converte os limites PWM (em us) para valores entre 0 e 4095 (12 bits do PCA9685)
  int periodUs = 1000000 / PWM_FREQ;                 // Calcula o período em microssegundos
  PWM_MIN = map(PULSE_MIN_US, 0, periodUs, 0, 4095);
  PWM_MAX = map(PULSE_MAX_US, 0, periodUs, 0, 4095);

  // Inicializa os servos nas posições de equilíbrio
  setServoPulse(CHANNEL_X, pulseCenterX);
  setServoPulse(CHANNEL_Y, pulseCenterY);

  lastTime = millis();   // Guarda a marca de tempo inicial para o cálculo do dt no PID
}

void loop() {
  // Lê continuamente a porta serial para receber comandos da dashboard
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');  // Lê uma linha até encontrar '\n'
    line.trim();                                 // Remove espaços em branco e quebras de linha
    if (line.length() < 1) continue;             // Ignora linhas vazias

    // Comando para ativar os motores
    if (line.startsWith("M1")) {
      motorsEnabled = true;                      // Ativa flag de controlo PID
      Serial.println("MOTORS_ON");               // Resposta opcional para dashboard
    }

    // Comando para desligar motores e reiniciar PID
    else if (line.startsWith("M0")) {
      motorsEnabled = false;                     // Desativa flag
      resetPID();                                // Reinicia todas as variáveis PID
      Serial.println("MOTORS_OFF");              // Confirmação
    }

    // Comando para envio de erros no formato "E,erroX,erroY"
    else if (line.startsWith("E,")) {
      int i1 = line.indexOf(',');
      int i2 = line.indexOf(',', i1 + 1);

      // Verifica se o formato é válido antes de extrair os valores
      if (i1 > 0 && i2 > i1) {
        // Extrai e converte os erros X e Y como floats
        errX = constrain(line.substring(i1+1, i2).toFloat(),
                         -(pulseCenterX - pulseMinX),
                          (pulseMaxX   - pulseCenterX));
        errY = constrain(line.substring(i2+1).toFloat(),
                         -(pulseCenterY - pulseMinY),
                          (pulseMaxY   - pulseCenterY));

        // Aplica PID apenas se os motores estiverem ativados
        if (motorsEnabled) applyPID();
      }
    }

    // Futuras extensões: adicionar comandos para calibração, modo automático, etc.
  }
}


void applyPID() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;                           // Tempo decorrido (em segundos)
  lastTime = now;

  if (dt <= 0.001 || dt > 0.5) return;                            // Proteção contra dt inválido (ruído ou pausas longas)

  // --- PID para eixo X ---
  sumErrX += errX * dt;                                           // Atualiza a integral
  sumErrX = constrain(sumErrX, -SUM_MAX, SUM_MAX);                // Limita a integral para evitar wind-up

  float rawDX = (errX - prevErrX) / dt;                           // Derivada bruta
  dErrXf = D_ALPHA * dErrXf + (1 - D_ALPHA) * rawDX;              // Derivada suavizada (filtro exponencial)
  float outX = (Kp_X * errX) + (Ki_X * sumErrX) + (Kd_X * dErrXf);// Saída PID

  prevErrX = errX;                                                // Atualiza erro anterior
  int pulseX = constrain(pulseCenterX + int(outX), pulseMinX, pulseMaxX); // Calcula pulso servo
  setServoPulse(CHANNEL_X, pulseX);                               // Envia comando para servo X

  // --- PID para eixo Y ---
  sumErrY += errY * dt;
  sumErrY = constrain(sumErrY, -SUM_MAX, SUM_MAX);

  float rawDY = (errY - prevErrY) / dt;
  dErrYf = D_ALPHA * dErrYf + (1 - D_ALPHA) * rawDY;
  float outY = (Kp_Y * errY) + (Ki_Y * sumErrY) + (Kd_Y * dErrYf);

  prevErrY = errY;
  int pulseY = constrain(pulseCenterY + int(outY), pulseMinY, pulseMaxY);
  setServoPulse(CHANNEL_Y, pulseY);
}


void resetPID() {
  sumErrX  = 0; sumErrY  = 0;  // Zera integrais
  prevErrX = 0; prevErrY = 0;  // Zera erros anteriores
  dErrXf   = 0; dErrYf   = 0;  // Zera derivadas filtradas
}

void setServoPulse(uint8_t channel, int microseconds) {
  int periodUs = 1000000 / PWM_FREQ;                       // Período total do PWM
  int pwmVal = map(microseconds, 0, periodUs, 0, 4095);    // Mapeia valor para resolução do PCA9685
  pwmVal = constrain(pwmVal, PWM_MIN, PWM_MAX);            // Garante que está dentro dos limites seguros
  pwm.setPWM(channel, 0, pwmVal);                          // Envia o sinal para o canal indicado
}

