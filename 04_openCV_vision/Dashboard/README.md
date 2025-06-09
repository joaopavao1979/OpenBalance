# OpenBalance Dashboard

Interface gráfica em Python para controle de uma plataforma de equilíbrio de bola, com visão computacional, PID em tempo real e ferramentas de calibração.

---

## 🖥️ Visão Computacional HSV
- Ajuste interativo de filtros HSV via **sliders** e **presets coloridos**  
- Preview em tempo real do feed de vídeo ou da máscara segmentada  
- Cruz de referência central e definição de ponto-alvo por **clique** na imagem  

## ⚙️ Controle PID
- Sliders de **Kp**, **Ki**, **Kd** com leitura imediata  
- Botão **“Aplicar Ganhos”** para atualizar o controlador sem reiniciar  
- Envio serial das coordenadas de erro (-X,-Y) ao Arduino/PCA9685  

## 🔧 Calibração de Servos
- Suporte ao **PCA9685 16-ch PWM** + servos MG996/DM996  
- Aba **“Calibração”** com:
  - **Offset central** (µs) para nivelamento fino  
  - **Limites de pulso** (mín./máx.) e botões de teste dedicados  
- Constrain automático dos ângulos aos limites mecânicos seguros  

## 🗄️ Configuração & Persistência
- **Salvar/Carregar** ajustes HSV, PID e calibração em arquivos **JSON** ou **CSV**  
- **Status bar** para feedback de sucesso/erro  
- Layout responsivo em **modo escuro** (dark-blue) com menus nativos  

---

## 🚀 Como usar

1. **Ajuste o filtro HSV** até isolar apenas a bola no feed.  
2. **Conecte** ao Arduino/PCA9685 e **teste** posições mín./máx. na aba Calibração.  
3. **Clique** no vídeo para definir um novo ponto-alvo ou use o centro padrão.  
4. **Aplique os ganhos PID** e habilite o “Seguimento” para manter a bola estável.  
5. **Salve** sua configuração para reutilização futura.

---

## 📄 Licença

Distribuído sob a licença **MIT**. Veja [LICENSE](LICENSE) para detalhes.  
