# 📋 Guia de Operação e Checklist do OpenBalance

✨ Parabéns por chegar à fase final do projeto OpenBalance! ✨

Este guia é a sua checklist definitiva para arrancar, calibrar e operar a sua plataforma de equilíbrio de forma segura e eficiente. Siga cada passo na ordem correta para garantir um arranque perfeito.

---

## Fase 1: Preparação e Verificação do Hardware 🛠️🔌

A segurança em primeiro lugar! Antes de ligar qualquer fonte de alimentação, vamos garantir que todas as ligações estão corretas.

*   [ ] **1. Desligue Tudo:** Certifique-se de que a fonte de alimentação externa de 6V e o cabo USB do Arduino estão completamente desligados.

*   [ ] **2. Verifique as Ligações Físicas:** Dê uma última olhada em todas as ligações.
    *   [   ] **🔌 USB:** O Arduino está ligado ao seu computador via cabo USB.
    *   [ ] **🔗 I²C:** Os pinos `SDA` e `SCL` do Arduino estão ligados aos pinos `SDA` e `SCL` do módulo PCA9685.
    *   [ ] **🤖 Servos:** O servo do eixo **X** está ligado ao **canal 0** do PCA9685. O servo do eixo **Y** está ligado ao **canal 1**.
    *   [ ] **⚡ Alimentação Externa:** A sua fonte de 6V está ligada aos bornes `V+` (positivo) e `GND` (negativo) do PCA9685.
    *   [ ] **🌍 Terra Comum (CRÍTICO!):** Existe um fio a ligar um pino `GND` do PCA9685 a um pino `GND` do seu Arduino. Sem isto, a comunicação I²C não funcionará corretamente.

---

## Fase 2: Calibração Física e Recolha de Dados 📝📐

Nesta fase, usamos uma ferramenta de diagnóstico (o seu script de calibração manual) para descobrir os limites físicos da sua plataforma.

*   [ ] **1. Carregar o Script de Calibração Manual:**
    *   Ligue **apenas** o cabo USB do Arduino ao computador.
    *   Abra o Arduino IDE e carregue o seu **script de calibração** (o que responde a comandos como `X90`, `Y80`, etc.).

*   [ ] **2. Abrir o Serial Monitor:**
    *   No Arduino IDE, abra o **Serial Monitor** (`Ctrl+Shift+M`).
    *   Verifique se a velocidade (baudrate) está definida para **9600**.

*   [ ] **3. Descobrir e Anotar os Ângulos ✍️:**
    *   Agora, ligue a sua fonte de alimentação externa de 6V.
    *   Envie comandos pelo Serial Monitor para encontrar os ângulos exatos. Seja paciente e encontre o valor perfeito para cada um. Anote-os aqui:

    ```
    -------------------------------------------
    VALORES DE CALIBRAÇÃO FÍSICA
    -------------------------------------------
    Preencher de cada vez que fizer a montagem	

    Ângulo de Equilíbrio X (e.g., 90): 100
    Ângulo Mínimo X        (e.g., 60): ______
    Ângulo Máximo X       (e.g., 120): ______

    Ângulo de Equilíbrio Y (e.g., 80): 80
    Ângulo Mínimo Y        (e.g., 50): 55
    Ângulo Máximo Y       (e.g., 110): ______
    -------------------------------------------
    ```
    *   Quando terminar, **desligue a fonte de alimentação externa**.

---

## Fase 3: Configuração do Sistema Final ⚙️🚀

Com os dados recolhidos, vamos carregar o software final e configurar a dashboard.

*   [ ] **1. Carregar o Firmware Final:**
    *   **Feche o Serial Monitor** no Arduino IDE (um passo crucial!).
    *   Abra o **firmware final e completo** que preparámos (para o PCA9685, com `SERVO_FREQ = 250`).
    *   Carregue este firmware para o seu Arduino.

*   [ ] **2. Executar a Dashboard Python:**
    *   No seu computador, execute a aplicação `OpenBalance Dashboard v5.0`.

*   [ ] **3. Ligar e Configurar na Dashboard:**
    *   No separador **"Operação"**, clique em **"Ligar"** para estabelecer a comunicação com o Arduino.
    *   Vá para o separador **"Assistente de Calibração (º)"**.
    *   Introduza os 6 ângulos que anotou na fase anterior nos campos correspondentes.
    *   Clique no botão **"Calcular e Aplicar para o separador Calibração (μs)"**. A magia aconteceu! ✨

*   [ ] **4. Verificar a Configuração:**
    *   Vá ao separador **"Calibração (μs)"**. Verifique se os campos foram preenchidos com valores numéricos (microssegundos).
    *   Ligue novamente a fonte de alimentação externa de 6V.
    *   Use os botões **"T"** para testar cada limite e o botão **"Testar Posição Central"** para confirmar que a plataforma se move como esperado.

---

## Fase 4: Operação e Sintonização Fina 🧠🟢

Tudo pronto para o "Go Live"!

*   [ ] **1. Preparar para o Controlo:**
    *   Vá para o separador **"Operação"**.
    *   Clique em **"Ligar Motores"**.
    *   No painel de vídeo, certifique-se de que a checkbox **"Filtro de Kalman"** está marcada (recomendado).
    *   No painel esquerdo, configure o **filtro HSV** para detetar a sua bola de forma fiável (um círculo verde deve aparecer à volta dela).

*   [ ] **2. Iniciar o Controlo Ativo:**
    *   Coloque a bola gentilmente no centro da plataforma.
    *   Clique em **"Ligar Seguimento"**. O sistema está agora a tentar ativamente equilibrar a bola.

*   [ ] **3. Sintonizar o PID:**
    *   Agora é a sua vez! Ajuste os 6 ganhos do PID (Kp, Ki, Kd para X e Y) até que a bola fique estável e o sistema responda de forma suave e precisa. Comece com valores pequenos e aumente gradualmente.

---

## Fase 5: Desligar de Forma Segura 🛑😴

Siga sempre esta ordem para desligar o sistema.

*   [ ] **1. Pare o Controlo no Software:**
    *   Na dashboard, clique em **"Parar Seguimento"**.
    *   Clique em **"Desligar Motores"**.
*   [ ] **2. Feche a Aplicação:**
    *   Feche a janela da aplicação Python. A sua última configuração será guardada automaticamente no `config.json`.
*   [ ] **3. Desligue o Hardware:**
    *   Desligue a fonte de alimentação externa de 6V.
    *   Desligue o cabo USB do Arduino.

---

## 🤔 Dicas Rápidas e Troubleshooting

*   **Servos não se movem?** Verifique a alimentação externa de 6V, a ligação de terra comum e as ligações I²C.
*   **Dashboard não liga ao Arduino?** Verifique se selecionou a porta COM correta e se o Serial Monitor do Arduino IDE está fechado.
*   **Movimento muito "nervoso" ou com "zumbido"?** Certifique-se de que o Filtro de Kalman está ativo. Se ainda assim acontecer, diminua os seus ganhos `Kd` e `Kp`.
*   **A bola "foge" sempre para um lado?** Verifique a sua calibração de equilíbrio no Assistente, ou pode ser que o seu ganho `Kp` esteja com o sinal invertido (raro, mas possível dependendo da montagem).

## 🎉 Parabéns!

Você construiu, configurou e operou com sucesso uma plataforma de equilíbrio avançada. Este é um feito de engenharia que combina mecânica, eletrónica e software. Bom trabalho! 🏆