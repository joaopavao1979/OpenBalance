#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =====================
// OpenBalance Arduino – Versão Atualizada com Calibração Física
// PID onboard, recebe erro da dashboard
// =====================

// === PARÂMETROS DE CALIBRAÇÃO (preencha após montagem) ===
const int ANG_EQUIL_X  =  62;   // ângulo de equilíbrio X
const int ANG_MIN_X    =  50;   // ângulo mínimo físico X
const int ANG_MAX_X    = 90;   // ângulo máximo físico X

const int ANG_EQUIL_Y  =  70;   // ângulo de equilíbrio Y
const int ANG_MIN_Y    =  55;   // ângulo mínimo físico Y
const int ANG_MAX_Y    = 95;   // ângulo máximo físico Y

// Pulsos servo (μs) limite genérico
const int PULSE_MIN_US = 1000;
const int PULSE_MAX_US = 2000;

// Canais PCA9685
const uint8_t CHANNEL_X = 0;
const uint8_t CHANNEL_Y = 1;

// Frequência do PWM em Hz
const int PWM_FREQ = 50;  // (50 Hz é o padrão, mas pode-se alterar)

// Ganhos PID (ajuste conforme necessário)
const float Kp_X = 0.3;    // Proporcional: corrige diretamente com base no erro atual para o eixo X
const float Kp_Y = 0.3;    // Proporcional: corrige diretamente com base no erro atual para o eixo Y
/*
Kp (Proporcional):
Controla a resposta direta ao erro.
Se estiver muito alto, a plataforma oscila muito rapidamente.
Se estiver muito baixo, a resposta fica lenta ou insuficiente.
*/
const float Ki_X = 0.01;   // Integral: corrige com base no histórico acumulado de erros para o eixo X
const float Ki_Y = 0.01;   // Integral: corrige com base no histórico acumulado de erros para o eixo Y
/*
Ki (Integral):
Ajuda a eliminar erros residuais persistentes (a bola não fica exatamente no centro).
Se estiver alto demais, pode causar lentidão e oscilações.
*/
const float Kd_X = 0.1;   // Derivativo: corrige com base na tendência do erro para o eixo X
const float Kd_Y = 0.1;   // Derivativo: corrige com base na tendência do erro para o eixo Y
/*
Kd (Derivativo):
Controla oscilações, antecipando a mudança do erro.
Demasiado alto pode provocar ruído, demasiado baixo pode não eliminar oscilações.
*/

// Limites anti-windup (será para evitar que o integral cresça descontroladamente)
const float SUM_MAX = 300.0;

// Derivativo - filtro alpha
const float D_ALPHA = 0.8;

// ===== Variáveis derivadas de calibração =====
int pulseMinX, pulseCenterX, pulseMaxX;
int pulseMinY, pulseCenterY, pulseMaxY;
int PWM_MIN, PWM_MAX;

// ===== Variáveis utilizadas no PID =====
// =========== Estado do PID =====
float errX = 0, errY = 0;               // Erro atual
float prevErrX = 0, prevErrY = 0;       // Erro anterior (para derivada)
float sumErrX  = 0, sumErrY  = 0;       // Somatório acumulado do erro (integral)
float dErrXf   = 0, dErrYf   = 0;       // Erro derivativo filtrado (para suavizar oscilações)

bool motorsEnabled = false;
unsigned long lastTime = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(115200);
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
  if (!Serial.available()) return;             //Verifica se há dados disponíveis via Serial, se não houver dados novos, simplesmente retorna e aguarda
  String line = Serial.readStringUntil('\n');  //Lê dados até encontrar uma quebra de linha
  line.trim();                                 //Remove espaços em branco ou caracteres invisíveis extras no início e no final da string
  if (line.length() < 1) return;               //Ignora linhas vazias

  if (line.startsWith("M1")) {
    // M1: ativa motores
    motorsEnabled = true;
    Serial.println("MOTORS_ON");
  }
  else if (line.startsWith("M0")) {
    // M0: desativa motores
    motorsEnabled = false;
    resetPID();
    Serial.println("MOTORS_OFF");
  }
  else if (line.startsWith("C,")) {
    // opcional: comandos de calibração via dashboard
  }
  else if (line.startsWith("E,")) {
    // Formato: E,errX,errY são os erros enviados pela Dashboard para cálculo PID
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1 + 1);
    if (i1 > 0 && i2 > i1) {
      errX = constrain(line.substring(i1+1, i2).toFloat(),
                       -(pulseCenterX - pulseMinX),
                        (pulseMaxX   - pulseCenterX));
      errY = constrain(line.substring(i2+1).toFloat(),
                       -(pulseCenterY - pulseMinY),
                        (pulseMaxY   - pulseCenterY));
      //Se os motores estiverem ativados (motorsEnabled), então o cálculo PID é realizado imediatamente (applyPID)
      if (motorsEnabled) applyPID();
    }
  }
}

// ===================================================================
// Função applyPID() – cálculo do controlo PID para cada eixo (X e Y)
// Utiliza ganhos PID independentes e aplica limites físicos de segurança
// ===================================================================
void applyPID() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;  // Intervalo de tempo desde o último ciclo (em segundos)
  lastTime = now;

  // Proteção contra intervalos de tempo anormais
  if (dt <= 0.001 || dt > 0.5) return;

  // ============================
  // === PID para o eixo X ===
  // ============================

  // Termo integral (soma dos erros ao longo do tempo)
  sumErrX += errX * dt;

  // Anti-windup: limita o crescimento do termo integral
  sumErrX = constrain(sumErrX, -SUM_MAX, SUM_MAX);

  // Termo derivativo (variação do erro)
  float rawDX = (errX - prevErrX) / dt;

  // Filtragem exponencial do derivativo (suaviza ruído)
  dErrXf = D_ALPHA * dErrXf + (1 - D_ALPHA) * rawDX;

  // Cálculo PID completo para o eixo X
  float outX = (Kp_X * errX) + (Ki_X * sumErrX) + (Kd_X * dErrXf);

  // Atualiza o erro anterior
  prevErrX = errX;

  // Aplica o controlo ao servo X (com limites de segurança)
  int pulseX = constrain(pulseCenterX + int(outX), pulseMinX, pulseMaxX);
  setServoPulse(CHANNEL_X, pulseX);

  // ============================
  // === PID para o eixo Y ===
  // ============================

  // Termo integral
  sumErrY += errY * dt;
  sumErrY = constrain(sumErrY, -SUM_MAX, SUM_MAX);

  // Termo derivativo
  float rawDY = (errY - prevErrY) / dt;
  dErrYf = D_ALPHA * dErrYf + (1 - D_ALPHA) * rawDY;

  // Cálculo PID completo para o eixo Y
  float outY = (Kp_Y * errY) + (Ki_Y * sumErrY) + (Kd_Y * dErrYf);

  // Atualiza o erro anterior
  prevErrY = errY;

  // Aplica o controlo ao servo Y (com limites de segurança)
  int pulseY = constrain(pulseCenterY + int(outY), pulseMinY, pulseMaxY);
  setServoPulse(CHANNEL_Y, pulseY);

  // ============================
  // Debug via Serial
  // ============================
  Serial.print("PID_X:"); Serial.print(pulseX);
  Serial.print(" PID_Y:"); Serial.println(pulseY);
}
// Serve para reiniciar todas as variáveis do controlo PID. Quando se desliga os motores (M0) e volta-se a ligar (M1)
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
