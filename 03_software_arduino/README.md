# 💾 Código Arduino – Projeto OpenBalance

Este diretório contém os sketches do Arduino para o controlo dos servos do sistema OpenBalance. O código comunica com o computador (Python) para ajustar a inclinação da plataforma em tempo real.

---

## 🧪 Subpastas

### 📁 `testes_motores/`
- `teste_manual_servo.ino`: código simples para testar se os servos estão a funcionar corretamente (varre de 0° a 180° com delay).

### 📁 `controlo_pid/`
- `openbalance_pid.ino`: implementa PID no próprio Arduino (opcional).
- Pode funcionar com sensores físicos em vez de visão computacional.

### 📁 `serial_python/`
- `receiver_com_serial.ino`: código principal para receber ângulos X e Y calculados em Python via serial e aplicar nos servos.

---

## 🛠️ Hardware Esperado

- Arduino UNO ou Leonardo (ATmega328 ou ATmega32u4)
- 3 servos MG996R ou similares
- Cabo USB para comunicação serial

---

## 🔌 Comunicação com Python

- Os ângulos desejados (θ₁, θ₂, θ₃) são enviados via porta serial (ex: "90;95;100")
- O Arduino interpreta os valores e usa `servo.writeMicroseconds()` ou `servo.write()`

---

## 🔧 Dependências

- Biblioteca Servo.h (incluída por padrão no Arduino IDE)
