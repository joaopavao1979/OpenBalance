# 📁 03_software_arduino

# 💾 Código Arduino – Projeto OpenBalance

Este repositório documenta os diferentes estágios de desenvolvimento do sistema físico OpenBalance, focado no controlo de uma plataforma de equilíbrio de bola com servomotores de alto torque e controlo PID.

O projeto evoluiu ao longo de várias abordagens e testes. Abaixo estão descritas as principais pastas e o papel de cada uma.

---

## 🗂️ Estrutura de Pastas

### 📁 `teste_motores/`
**Objetivo:** Calibração e teste individual dos servos antes da montagem do sistema.  
**Conteúdo:**  
- `calibracao_servos_v1.ino`: permite ajustar manualmente os ângulos dos servos X e Y via monitor série.  
- Comandos como `X80`, `Y120` permitem verificar os limites físicos de forma segura.

---

### 📁 `OpenBalance_Arduino_PID_testes/`
**Objetivo:** Primeiros esboços do controlo PID a correr diretamente no Arduino.  
**Conteúdo:**  
- Testes com controlo PID simples sem integração total com visão computacional.  
- Experiências com filtragem, mapeamento de ângulos e resposta dos motores.

---

### 📁 `OpenBalance_Arduino_Firmware/`
**Objetivo:** Tentativa experimental de mover o cálculo PID para o Python (dashboard), deixando o Arduino apenas a aplicar ângulos.  
**Estado:** Abordagem parcialmente funcional, mas abandonada por instabilidade na comunicação serial e atrasos.  
**Lições aprendidas:** O controlo PID é mais fiável se executado diretamente no microcontrolador.

---

### 📁 `OpenBalance_Arduino_PID/` ✅ **VERSÃO FINAL**
**Objetivo:** Implementação final e estável do sistema com controlo PID on-board.  
**Conteúdo:**  
- `OpenBalance_Arduino_PID.ino`: versão oficial final.  
- O Arduino recebe apenas os erros (X, Y) da dashboard Python via serial e calcula os ângulos de inclinação localmente usando PID.  
- Código comentado, modular e robusto.

---

## ⚙️ Hardware Utilizado

- Arduino UNO ou compatível  
- 2x servos DM996R (15 kg/cm)  
- Driver PCA9685 (PWM 16 canais)  
- Fonte regulável 6V/72W  
- Conexão com PC via USB  

---

## 🔌 Comunicação com Python

- A dashboard em Python envia erros `E,errX,errY` via porta serial.
- Comandos adicionais como `M1` e `M0` ligam/desligam os motores.
- O Arduino responde com mensagens de estado como `MOTORS_ON`.

---

## 🛠️ Bibliotecas Requeridas

- `Wire.h`  
- `Adafruit_PWMServoDriver.h`  

---

## 📌 Notas Finais

- Este repositório serve também fins pedagógicos, sendo documentado e modular.  
- A versão final (pasta `OpenBalance_Arduino_PID/`) deve ser usada como referência para integrações futuras com IA ou reforço (Q-Learning, etc.).