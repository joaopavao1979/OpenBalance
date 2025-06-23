# 📁 04_Dashboard_OpenBalance

# 🖥️ Dashboard Python – Projeto OpenBalance

Este repositório contém o código da interface gráfica desenvolvida em Python para o sistema físico OpenBalance, que controla uma plataforma de equilíbrio de bola em tempo real. Utiliza visão computacional, controlo PID e comunicação serial com Arduino.

---

## 🗂️ Estrutura de Pastas

### 📁 `testes_Dashboard/`
**Objetivo:** Reunião de versões experimentais e esboços da interface.  
**Conteúdo:**  
- Primeiros testes com Tkinter, sliders HSV, deteção OpenCV, e comunicação serial.  
- Algumas versões antigas ou auxiliares podem ser limpas futuramente.

---

### 📁 `Dashboard_estável/`
**Objetivo:** Versão funcional e confiável da dashboard sem gráfico de erros.  
**Características:**  
- Envio de erros `E,errX,errY` via serial.  
- Interface modular com sliders HSV, centro clicável, e controlo de motores.  
- Estável e ideal para execução contínua sem sobrecarga gráfica.

---

### 📁 `Dashboard_OpenBalance_V1.0/` ✅ **VERSÃO FINAL**
**Objetivo:** Versão final da interface com **gráfico de erro X/Y em tempo real**.  
**Ficheiro principal:** `Dashboard_OpenBalance_V1.0.py`  
**Conteúdo:**  
- Visualização OpenCV (320x240) com sobreposição de posição da bola e centro de referência.  
- Sliders ajustáveis HSV para calibração visual.  
- Controlo serial completo com comandos `M1`, `M0`, e `E,errX,errY`.  
- Interface robusta com secção para PID, estado, nivelamento e modo de seguimento.  
- Integração com `matplotlib` para abrir janela com gráfico de erro acumulado ao clicar no botão correspondente.

---

## 📦 Bibliotecas Usadas

- `customtkinter`  
- `opencv-python` (`cv2`)  
- `pyserial`  
- `Pillow`  
- `numpy`  
- `matplotlib`

Instalação recomendada:

```bash
pip install customtkinter opencv-python pyserial Pillow numpy matplotlib
```

---

## 🔌 Comunicação com Arduino

- Envio de erros via porta serial no formato: `E,errX,errY`
- Comandos de controlo:  
  - `M1` → Ativa motores  
  - `M0` → Desativa motores  
- Sincronização total com o firmware `OpenBalance_Arduino_PID.ino`

---

## 📌 Notas Finais

- A versão **`Dashboard_OpenBalance_V1.0`** deve ser usada para demonstrações e testes finais.  
- A pasta `Dashboard_estável` mantém uma alternativa leve e funcional, sem gráfico.
- As versões em `testes_Dashboard` são experimentais e poderão ser organizadas ou removidas.