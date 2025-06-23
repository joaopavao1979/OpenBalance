#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// ================================================
// OpenBalance Arduino – Versão 2.1 (Estável)
// PID e Calibração controlados via Serial
// Adicionada lógica para comando de teste MOVE
// ================================================

// --- PROTOCOLO DE COMUNICAÇÃO (v2.1) ---
// Comandos recebidos da Dashboard (terminados em '\n'):
//   'M1'                 -> Ativa os motores e o controlo PID.
//   'M0'                 -> Desativa os motores e recentra os servos.
//   'E,errX,errY'        -> Recebe o erro (em pixels) e aplica um ciclo PID.
//   'P,kp,ki,kd,kp,ki,kd'-> Define os 6 ganhos PID (X e Y).
//   'CAL_X,min,eq,max'   -> Define os ângulos (graus) de calibração do eixo X.
//   'CAL_Y,min,eq,max'   -> Define os ângulos (graus) de calibração do eixo Y.
//   'MOVE,ch,angle'      -> (NOVO) Move um servo para um ângulo específico para teste.
//   'SAVE'               -> Guarda a calibração e os ganhos PID atuais na EEPROM.
//
// Respostas enviadas para a Dashboard:
//   'ACK: ...', 'INFO: ...', 'WARN: ...'

// --- Estrutura para guardar configurações na EEPROM ---
struct Settings {
  long version; // Usado para validar se a EEPROM tem dados válidos
  float Kp_X, Ki_X, Kd_X;
  float Kp_Y, Ki_Y, Kd_Y;
  int ANG_MIN_X, ANG_EQUIL_X, ANG_MAX_X;
  int ANG_MIN_Y, ANG_EQUIL_Y, ANG_MAX_Y;
};

const long SETTINGS_VERSION = 20240522L; // Mude este número se alterar a struct

// === PARÂMETROS DE HARDWARE ===
const int PULSE_MIN_US = 1000; // Microsegundos para o ângulo mínimo do servo
const int PULSE_MAX_US = 2000; // Microsegundos para o ângulo máximo do servo
const uint8_t CHANNEL_X = 0;
const uint8_t CHANNEL_Y = 1;
const int PWM_FREQ = 50; // Hz

// === PARÂMETROS DE CONTROLO (Valores padrão, podem ser sobrepostos pela EEPROM/Serial) ===
Settings settings = {
  SETTINGS_VERSION,
  // Ganhos PID
  0.25, 0.005, 0.1,  // Kp_X, Ki_X, Kd_X
  0.15, 0.02,  0.1,  // Kp_Y, Ki_Y, Kd_Y
  // Calibração de Ângulos
  75, 89, 105,       // ANG_MIN_X, ANG_EQUIL_X, ANG_MAX_X
  65, 79, 100        // ANG_MIN_Y, ANG_EQUIL_Y, ANG_MAX_Y
};

const float SUM_MAX = 300.0; // Limite Anti-Windup para o termo integral
const float D_ALPHA = 0.8;   // Fator de suavização para o termo derivativo

// ===== Variáveis de estado e cálculo =====
int pulseMinX, pulseCenterX, pulseMaxX;
int pulseMinY, pulseCenterY, pulseMaxY;

float errX = 0, errY = 0;
float prevErrX = 0, prevErrY = 0;
float sumErrX = 0, sumErrY = 0;
float dErrXf = 0, dErrYf = 0; // Derivada filtrada
bool motorsEnabled = false;
unsigned long lastTime = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ==================
// ===    SETUP   ===
// ==================
void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  loadSettings(); // Tenta carregar configurações da EEPROM
  
  recalculatePulseLimits(); // Calcula os pulsos com base nos ângulos (padrão ou da EEPROM)
  
  // Posiciona os servos no ponto de equilíbrio
  setServoPulse(CHANNEL_X, pulseCenterX);
  setServoPulse(CHANNEL_Y, pulseCenterY);

  lastTime = millis();
}

// =================
// ===    LOOP   ===
// =================
void loop() {
  if (Serial.available()) {
    handleSerialCommand();
  }
}

// ===================================
// === PROCESSAMENTO DOS COMANDOS ===
// ===================================
void handleSerialCommand() {
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() < 1) return;

  if (line.startsWith("M1")) {
    motorsEnabled = true;
    Serial.println("ACK: MOTORS_ON");
  } 
  else if (line.startsWith("M0")) {
    motorsEnabled = false;
    resetPID();
    setServoPulse(CHANNEL_X, pulseCenterX);
    setServoPulse(CHANNEL_Y, pulseCenterY);
    Serial.println("ACK: MOTORS_OFF");
  } 
  else if (line.startsWith("E,")) {
    if (sscanf(line.c_str(), "E,%f,%f", &errX, &errY) == 2) {
      if (motorsEnabled) applyPID();
    }
  }
  else if (line.startsWith("P,")) {
    if (sscanf(line.c_str(), "P,%f,%f,%f,%f,%f,%f", 
        &settings.Kp_X, &settings.Ki_X, &settings.Kd_X, 
        &settings.Kp_Y, &settings.Ki_Y, &settings.Kd_Y) == 6) {
      Serial.println("ACK: PID gains updated.");
    }
  }
  else if (line.startsWith("CAL_X,")) {
    if (sscanf(line.c_str(), "CAL_X,%d,%d,%d", 
        &settings.ANG_MIN_X, &settings.ANG_EQUIL_X, &settings.ANG_MAX_X) == 3) {
      recalculatePulseLimits();
      Serial.println("ACK: X-axis calibration updated.");
    }
  }
  else if (line.startsWith("CAL_Y,")) {
    if (sscanf(line.c_str(), "CAL_Y,%d,%d,%d", 
        &settings.ANG_MIN_Y, &settings.ANG_EQUIL_Y, &settings.ANG_MAX_Y) == 3) {
      recalculatePulseLimits();
      Serial.println("ACK: Y-axis calibration updated.");
    }
  }
  else if (line.startsWith("SAVE")) {
    saveSettings();
    Serial.println("ACK: Settings saved to EEPROM.");
  }
  // =======================================================
  // === NOVO BLOCO DE CÓDIGO PARA TESTE DE CALIBRAÇÃO ===
  // =======================================================
  else if (line.startsWith("MOVE,")) {
    int channel = -1, angle = -1;
    // Extrai o canal e o ângulo do comando
    if (sscanf(line.c_str(), "MOVE,%d,%d", &channel, &angle) == 2) {
      if (channel >= 0 && channel <= 1 && angle >= 0 && angle <= 180) {
        // Converte o ângulo recebido (0-180) para um pulso em microssegundos
        int pulse_us = map(angle, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
        
        // Usa a nossa função segura para mover o servo
        setServoPulse(channel, pulse_us);
        
        // Envia feedback de volta para a dashboard
        Serial.print("ACK: Moved channel ");
        Serial.print(channel);
        Serial.print(" to angle ");
        Serial.println(angle);
      }
    }
  }
}

// ===================================
// === LÓGICA DE CONTROLO E SERVOS ===
// ===================================

void applyPID() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  if (dt <= 0.001) return;

  // Eixo X
  sumErrX += settings.Ki_X * errX * dt;
  sumErrX = constrain(sumErrX, -SUM_MAX, SUM_MAX);
  float dErrX = (errX - prevErrX) / dt;
  dErrXf = D_ALPHA * dErrXf + (1 - D_ALPHA) * dErrX;
  float outX = (settings.Kp_X * errX) + sumErrX + (settings.Kd_X * dErrXf);
  prevErrX = errX;
  int pulseX = constrain(pulseCenterX + int(outX), pulseMinX, pulseMaxX);
  setServoPulse(CHANNEL_X, pulseX);

  // Eixo Y
  sumErrY += settings.Ki_Y * errY * dt;
  sumErrY = constrain(sumErrY, -SUM_MAX, SUM_MAX);
  float dErrY = (errY - prevErrY) / dt;
  dErrYf = D_ALPHA * dErrYf + (1 - D_ALPHA) * dErrY;
  float outY = (settings.Kp_Y * errY) + sumErrY + (settings.Kd_Y * dErrYf);
  prevErrY = errY;
  int pulseY = constrain(pulseCenterY + int(outY), pulseMinY, pulseMaxY);
  setServoPulse(CHANNEL_Y, pulseY);
}

void resetPID() {
  sumErrX = 0; sumErrY = 0;
  prevErrX = 0; prevErrY = 0;
  dErrXf = 0; dErrYf = 0;
}

void setServoPulse(uint8_t channel, int microseconds) {
  int periodUs = 1000000 / PWM_FREQ;
  int pwmVal = map(microseconds, 0, periodUs, 0, 4095);
  pwm.setPWM(channel, 0, pwmVal);
}

void recalculatePulseLimits() {
  pulseMinX    = map(settings.ANG_MIN_X,   0, 180, PULSE_MIN_US, PULSE_MAX_US);
  pulseCenterX = map(settings.ANG_EQUIL_X, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxX    = map(settings.ANG_MAX_X,   0, 180, PULSE_MIN_US, PULSE_MAX_US);
  
  pulseMinY    = map(settings.ANG_MIN_Y,   0, 180, PULSE_MIN_US, PULSE_MAX_US);
  pulseCenterY = map(settings.ANG_EQUIL_Y, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
  pulseMaxY    = map(settings.ANG_MAX_Y,   0, 180, PULSE_MIN_US, PULSE_MAX_US);
}

// ===================================
// === GESTÃO DA EEPROM ===
// ===================================
void saveSettings() {
  EEPROM.put(0, settings);
}

void loadSettings() {
  Settings storedSettings;
  EEPROM.get(0, storedSettings);

  if (storedSettings.version == SETTINGS_VERSION) {
    settings = storedSettings; // Carrega as configurações válidas
    Serial.println("INFO: Loaded settings from EEPROM.");
  } else {
    // Dados inválidos ou versão antiga, usa os valores padrão
    Serial.println("WARN: EEPROM data invalid, using defaults.");
  }
}