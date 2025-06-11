/*
 ================================================================================
 OpenBalance Arduino Firmware (Versão Final - PCA9685 & MG996R)
 ================================================================================

 Descrição:
   Este é o firmware final e otimizado para a plataforma OpenBalance, projetado
   para operar com um controlador de servos PCA9685 e servos digitais de
   alto torque (como o MG996R). Ele atua como um receptor de comandos robusto,
   eficiente e seguro para o OpenBalance Dashboard em Python.

 Arquitetura e Lógica:
   1.  **Inicialização:** O Arduino inicializa a comunicação Serial e o
       módulo PCA9685, configurando a frequência de PWM para 250 Hz, ideal
       para a resposta rápida dos servos digitais.
   2.  **Escuta Passiva:** O loop principal verifica continuamente a porta série
       de forma não-bloqueante, garantindo máxima responsividade.
   3.  **Análise de Comando:** Os comandos recebidos ('M1', 'M0', 'C,xxxx,yyyy')
       são analisados e executados de forma segura.
   4.  **Execução Segura:** Os comandos de movimento são restringidos a um
       intervalo seguro (MIN_US, MAX_US) antes de serem enviados ao PCA9685,
       protegendo os servos contra danos.

 Hardware:
   - Placa Arduino (Uno, Nano, etc.).
   - Módulo Controlador de Servos PCA9685.
   - 2x Servos Digitais MG996R (ou similar).
   - Fonte de alimentação externa (e.g., 6V) para os servos.

 Ligação:
   - Arduino (SDA, SCL) -> PCA9685 (SDA, SCL).
   - Fonte externa (V+, GND) -> Bloco de terminais do PCA9685.
   - GND da fonte externa -> GND do Arduino (referência comum).

 Bibliotecas Necessárias (instalar via Library Manager do Arduino IDE):
   - "Adafruit PWM Servo Driver"
   - "Adafruit BusIO"

 Autor: Equipa de Engenharia de Software (em colaboração com João Pavão)
*/

// ================================================================================
// 1. INCLUDES E DEFINIÇÕES GLOBAIS
// ================================================================================

#include <Wire.h>                      // Biblioteca para comunicação I²C
#include <Adafruit_PWMServoDriver.h>   // Biblioteca para o controlador PCA9685

// --- CONFIGURAÇÃO DOS CANAIS DOS SERVOS ---
// Defina aqui em quais canais do PCA9685 os seus servos estão conectados.
const int SERVO_X_CHANNEL = 0; // Servo do eixo X no canal 0
const int SERVO_Y_CHANNEL = 1; // Servo do eixo Y no canal 1

// --- PARÂMETROS DO PWM ---
// OTIMIZAÇÃO PARA SERVO DIGITAL (MG966R):
// Servos digitais podem operar a frequências de PWM mais altas, resultando em
// maior precisão e torque de sustentação. 250 Hz é um bom ponto de partida.
// Se notar aquecimento excessivo, pode reduzir para 100 Hz ou 50 Hz.
const int SERVO_FREQ = 50;

// --- LIMITES DE SEGURANÇA (HARD-CODED) ---
// Esta é a última linha de defesa para proteger os seus servos.
// O intervalo de 1000-2000μs é o padrão para a maioria dos servos.
const int MIN_US = 1000; // Limite mínimo seguro em microssegundos
const int MAX_US = 2000; // Limite máximo seguro em microssegundos


// ================================================================================
// 2. OBJETOS E VARIÁVEIS GLOBAIS
// ================================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Instância do controlador

String serialData = "";      // String para acumular os dados recebidos
bool motorsOn = false;       // Flag para controlar o estado dos motores


// ================================================================================
// 3. FUNÇÃO DE SETUP - Executada uma vez no arranque
// ================================================================================

void setup() {
  // Inicializa a comunicação Serial com a mesma velocidade definida no Python.
  Serial.begin(9600);
  serialData.reserve(32); // Otimização: pre-aloca memória para a string.

  // Inicializa a comunicação I²C e o controlador PCA9685.
  pwm.begin();
  
  // Define a frequência do PWM otimizada para os servos digitais.
  pwm.setPWMFreq(SERVO_FREQ);

  // Mensagem de boas-vindas para o Serial Monitor.
  Serial.println("OpenBalance Arduino Firmware Final (PCA9685 / MG996R)");
  Serial.println("Frequencia PWM: " + String(SERVO_FREQ) + " Hz");
  Serial.println("Aguardando comandos do Dashboard Python...");
}


// ================================================================================
// 4. FUNÇÃO DE LOOP - Executada continuamente
// ================================================================================

void loop() {
  handleSerial();
}


// ================================================================================
// 5. FUNÇÕES AUXILIARES
// ================================================================================

/**
 * @brief Lê e processa todos os dados disponíveis na porta série de forma não-bloqueante.
 */
void handleSerial() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();

    if (receivedChar == '\n') { // Fim do comando detectado
      parseCommand(serialData);
      serialData = ""; // Limpa a string para o próximo comando
    } else {
      serialData += receivedChar; // Constrói a string do comando
    }
  }
}

/**
 * @brief Interpreta o comando recebido e executa a ação correspondente.
 * @param cmd A string de comando completa (sem o '\\n').
 */
void parseCommand(String cmd) {
  cmd.trim(); // Remove espaços em branco

  // --- Comando para Ligar os Motores ---
  if (cmd == "M1") {
    motorsOn = true;
    // Move para uma posição central segura (1500μs) ao ligar.
    pwm.writeMicroseconds(SERVO_X_CHANNEL, 1500);
    pwm.writeMicroseconds(SERVO_Y_CHANNEL, 1500);
    Serial.println("OK: Motores LIGADOS.");
    return;
  }

  // --- Comando para Desligar os Motores ---
  if (cmd == "M0") {
    // Desliga os pulsos PWM para os canais dos servos.
    pwm.setPWM(SERVO_X_CHANNEL, 0, 0);
    pwm.setPWM(SERVO_Y_CHANNEL, 0, 0);
    motorsOn = false;
    Serial.println("OK: Motores DESLIGADOS.");
    return;
  }

  // --- Comando de Movimento (e.g., "C,1550,1480") ---
  if (cmd.startsWith("C")) {
    if (!motorsOn) {
      Serial.println("AVISO: Comando de movimento ignorado. Motores desligados. Envie 'M1' primeiro.");
      return;
    }

    // Extrai os valores de microssegundos para X e Y
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      String x_str = cmd.substring(firstComma + 1, secondComma);
      String y_str = cmd.substring(secondComma + 1);

      int pos_x = x_str.toInt();
      int pos_y = y_str.toInt();

      // **MECANISMO DE SEGURANÇA**
      pos_x = constrain(pos_x, MIN_US, MAX_US);
      pos_y = constrain(pos_y, MIN_US, MAX_US);

      // Envia os comandos finais e seguros para o PCA9685.
      pwm.writeMicroseconds(SERVO_X_CHANNEL, pos_x);
      pwm.writeMicroseconds(SERVO_Y_CHANNEL, pos_y);
    }
  }
}