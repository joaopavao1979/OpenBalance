# 📁 03_software_arduino

# 💾 Código Arduino – Projeto OpenBalance
Este diretório contém os sketches do Arduino para o controlo dos servos do sistema OpenBalance. O código comunica com o computador (Python) para ajustar a inclinação da plataforma em tempo real.

---

## 🧪 Subpastas

### 📁 `testes_motores/`
        calibracao_servos_v1
        código simples para calibrar os 2 motores servo (X e Y). Inicialmente os servo vão para a poisição de 90º e atraves do monitor serial podemos ajustar escrevendo X0 (para o angulo 0º no motor X) ou Y120 (para o angulo de 120º no motor Y).

### 📁 `controlo_pid/`
- `openbalance_pid.ino`: implementa PID no próprio Arduino (opcional).
- Pode funcionar com sensores físicos em vez de visão computacional.

### 📁 `serial_python/`
- `receiver_com_serial.ino`: código principal para receber ângulos X e Y calculados em Python via serial e aplicar nos servos.

---

## 🛠️ Hardware Esperado

- Arduino UNO
- 2 servos MG996R ou similares
- Cabo USB para comunicação serial

---

## 🔌 Comunicação com Python

- Os ângulos desejados (θ₁, θ₂) são enviados via porta serial (ex: "90;95;100")
- O Arduino interpreta os valores e usa `servo.writeMicroseconds()` ou `servo.write()`

---

## 🔧 Dependências

- Biblioteca Servo.h (incluída por padrão no Arduino IDE)
- Biblioteca Adafruit_PWMServoDriver.h (Shield usada para os motores servo)
