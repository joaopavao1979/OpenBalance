# -*- coding: utf-8 -*-
"""
================================================================================
OpenBalance Dashboard (Versão 4.2 - Edição Final Didática)
================================================================================

Descrição:
    Dashboard de controlo para uma plataforma de equilíbrio de bola (Ball and
    Plate). Esta versão final representa a consolidação da arquitetura de
    controlo centralizado, onde toda a lógica de processamento de imagem e
    cálculo do PID reside no software Python. O microcontrolador (Arduino)
    atua apenas como um "atuador", recebendo comandos diretos de posição
    dos servos.

Arquitetura Principal:
    1.  **Visão Computacional (OpenCV):** Captura o vídeo, deteta a bola através
        de um filtro de cor HSV e calcula a sua posição (x, y).
    2.  **Cálculo de Erro:** O erro é a distância entre a posição atual da bola
        e o centro do ecrã (setpoint).
    3.  **Controlador PID (Python):** Dois controladores PID (um para o eixo X,
        outro para o Y) recebem o erro e calculam um sinal de correção.
    4.  **Mapeamento e Comando:** O sinal de correção do PID é mapeado para um
        intervalo de microssegundos (μs) correspondente ao movimento físico
        dos servos.
    5.  **Comunicação Série (PySerial):** O comando final (e.g., "C,1550,1480\n")
        é enviado para o Arduino, que move os servos para a posição desejada.

Melhorias Notáveis nesta Versão:
    - Lógica do PID reside 100% no Python para flexibilidade e depuração fácil.
    - Separador "Calibração" robusto com ferramentas de Nivelamento e Amplitude.
    - Botões de teste dedicados ("T") para cada limite de movimento (mín/máx),
      permitindo uma calibração física iterativa, precisa e segura.
    - Layout da grelha de botões de preset de cor corrigido para ser mais
      compacto e escalável.
    - Código extensivamente comentado para fins didáticos e de manutenção futura.

Autor:
    João Pavão (Conceito Original)
    Refatorado e Comentado por Equipa de Engenharia de Software (para fins didáticos)
"""

# -----------------------------------------------------------------------------
# 1. IMPORTAÇÕES E CONSTANTES GLOBAIS
# -----------------------------------------------------------------------------
# Importações de bibliotecas standard e de terceiros
import json                     # Para ler e escrever ficheiros de configuração (config.json)
import threading                # Para executar o processamento de vídeo numa thread separada, evitando que a UI congele
import cv2                      # OpenCV: A biblioteca principal para todo o processamento de imagem e vídeo
import numpy as np              # NumPy: Essencial para operações numéricas eficientes, especialmente com imagens (arrays)
import customtkinter as ctk     # Biblioteca para criar a interface gráfica moderna
import serial                   # PySerial: Para comunicação com a porta série (Arduino)
import serial.tools.list_ports  # Para listar as portas COM disponíveis
from PIL import Image, ImageTk  # Pillow (PIL): Para converter imagens do formato OpenCV para um formato que o Tkinter/CTk possa exibir
import tkinter as tk            # Tkinter: Usado para alguns widgets base e variáveis (StringVar, etc.)
from tkinter import filedialog, messagebox # Módulos do Tkinter para caixas de diálogo padrão

# --- Constantes de Configuração da Aplicação ---
WINDOW_WIDTH = 1500             # Largura da janela principal em pixels
WINDOW_HEIGHT = 850             # Altura da janela principal em pixels
VIDEO_WIDTH = 800               # Largura da área de vídeo em pixels
VIDEO_HEIGHT = 600              # Altura da área de vídeo em pixels
DEFAULT_BAUDRATE = 9600         # Baudrate padrão para a comunicação série (deve corresponder ao do Arduino)

# --- Constantes de Protocolo e UI ---
CMD_MOTORS_ON = "M1\n"          # Comando para ligar os motores no Arduino
CMD_MOTORS_OFF = "M0\n"         # Comando para desligar os motores no Arduino
CMD_CALIBRATION_PREFIX = "C"    # Prefixo para comandos de calibração/movimento direto (e.g., "C,1500,1500\n")

# Paleta de cores para consistência visual da UI
COLOR_SUCCESS = {"fg_color": "#2E7D32", "hover_color": "#1B5E20"}  # Verde para ações de sucesso/ligar
COLOR_DANGER = {"fg_color": "#C62828", "hover_color": "#B71C1C"}   # Vermelho para ações de perigo/desligar
COLOR_NEUTRAL = {"fg_color": "#1E88E5", "hover_color": "#1565C0"}  # Azul para ações neutras/principais
COLOR_SECONDARY = {"fg_color": "#616161", "hover_color": "#424242"}# Cinzento para ações secundárias

# Dicionário com presets de valores HSV (Hue, Saturation, Value) para deteção de cores comuns
# O espaço de cor HSV é mais robusto a variações de iluminação do que o RGB.
HSV_PRESETS = {
    "Vermelho":{"h_min":0,  "h_max":10,  "s_min":100, "s_max":255, "v_min":100, "v_max":255},
    "Laranja": {"h_min":10, "h_max":25,  "s_min":100, "s_max":255, "v_min":100, "v_max":255},
    "Verde":   {"h_min":40, "h_max":80,  "s_min":100, "s_max":255, "v_min":100, "v_max":255},
    "Azul":    {"h_min":90, "h_max":130, "s_min":100, "s_max":255, "v_min":50,  "v_max":255},
    "Cinzento":{"h_min":0,  "h_max":180, "s_min":0,   "s_max":50,  "v_min":50,  "v_max":200},
    "Branco":  {"h_min":0,  "h_max":180, "s_min":0,   "s_max":30,  "v_min":200, "v_max":255},
}

# Dicionário para estilizar os botões de preset, garantindo boa legibilidade
BUTTON_COLORS = {
    "Vermelho":{"fg":"#FF0000", "hover":"#CC0000", "text":"white"},
    "Laranja": {"fg":"#FFA500", "hover":"#CC8400", "text":"white"},
    "Verde":   {"fg":"#00B200", "hover":"#008000", "text":"white"},
    "Azul":    {"fg":"#0000FF", "hover":"#0000CC", "text":"white"},
    "Cinzento":{"fg":"#808080", "hover":"#666666", "text":"white"},
    "Branco":  {"fg":"#FFFFFF", "hover":"#DDDDDD", "text":"black"},
}

# -----------------------------------------------------------------------------
# 2. CLASSES UTILITÁRIAS
# -----------------------------------------------------------------------------

class SerialManager:
    """ Gere toda a comunicação com a porta série (Arduino). """
    def __init__(self):
        self.serial_conn = None  # Objeto da conexão série

    def list_ports(self):
        """ Retorna uma lista de todas as portas COM disponíveis no sistema. """
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        """ Tenta conectar-se a uma porta série. Fecha qualquer conexão anterior. """
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            return True  # Sucesso
        except Exception:
            self.serial_conn = None
            return False # Falha

    def disconnect(self):
        """ Fecha a conexão série se estiver aberta. """
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.serial_conn = None

    def send(self, data_str):
        """ Envia uma string de dados para o Arduino. """
        if not self.serial_conn or not self.serial_conn.is_open:
            return  # Não faz nada se não estiver conectado
        try:
            # A string precisa de ser codificada em bytes (utf-8 é o padrão) antes de ser enviada.
            self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException:
            # Se ocorrer um erro (e.g., o cabo foi desligado), desconecta-se para evitar mais erros.
            self.disconnect()

class StatusBar(ctk.CTkFrame):
    """ Uma barra de status na parte inferior da janela para exibir mensagens. """
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w")
        self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None # Armazena o ID do evento 'after' para poder cancelá-lo

    def show_message(self, message, duration_ms=4000, is_error=False):
        """ Mostra uma mensagem temporária. Se uma nova mensagem chegar, a anterior é substituída. """
        if self._job:
            self.after_cancel(self._job) # Cancela a tarefa anterior para limpar a mensagem
        text_color = COLOR_DANGER["fg_color"] if is_error else "gray70"
        self.label.configure(text=message, text_color=text_color)
        # Usa o método 'after' do Tkinter para agendar a limpeza da mensagem no futuro, sem bloquear a UI.
        self._job = self.after(duration_ms, lambda: self.label.configure(text=""))

class PIDController:
    """ Implementação de um controlador Proporcional-Integral-Derivativo (PID). """
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd # Ganhos do controlador
        self.setpoint = setpoint               # O valor desejado (neste caso, 0, o centro do ecrã)
        self._last_error = 0                   # Erro do ciclo anterior (para o termo Derivativo)
        self._integral = 0                     # Acumulação do erro (para o termo Integral)

    def update(self, process_variable):
        """ Calcula a saída do PID com base no valor atual do processo. """
        # Termo Proporcional (P): Reage ao erro atual.
        error = self.setpoint - process_variable
        
        # Termo Integral (I): Corrige o erro residual acumulado ao longo do tempo.
        self._integral += error
        
        # Termo Derivativo (D): Tenta antecipar o erro futuro, reagindo à taxa de variação do erro.
        derivative = error - self._last_error
        
        self._last_error = error
        
        # A saída final é a soma ponderada dos três termos.
        return self.Kp * error + self.Ki * self._integral + self.Kd * derivative

    def set_gains(self, Kp, Ki, Kd):
        """ Permite alterar os ganhos do PID em tempo de execução. """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self._integral = 0 # Reinicia o termo integral para evitar "saltos" na saída

# -----------------------------------------------------------------------------
# 3. CLASSES DE INTERFACE GRÁFICA (FRAMES)
# -----------------------------------------------------------------------------

class HSVSettingsFrame(ctk.CTkFrame):
    """ Frame para configurar os parâmetros de deteção de cor (HSV). """
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.video_handler = video_handler
        self.grid_columnconfigure(0, weight=1) # Faz com que a coluna se expanda com a janela
        self.filter_enabled = True

        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10, 15), sticky="n")

        self.hsv_sliders = []
        self.hsv_values = []
        labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
        
        # --- Criação dinâmica dos sliders e labels para os 6 parâmetros HSV ---
        for idx, label in enumerate(labels):
            ctk.CTkLabel(self, text=label).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(value="0") # Variável de texto para mostrar o valor do slider
            self.hsv_values.append(val_var)
            
            # O lambda é usado para passar a variável correta (val_var) para a função de callback.
            slider = ctk.CTkSlider(self, from_=0, to=255, command=lambda val, var=val_var: var.set(f"{int(val)}"))
            slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew")
            self.hsv_sliders.append(slider)
            
            ctk.CTkLabel(self, textvariable=val_var).grid(row=2 + 2*idx, column=0, padx=(270,10), sticky="e")

        self.show_mask_var = ctk.BooleanVar(value=False)
        self.checkbox_mask = ctk.CTkCheckBox(self, text="Mostrar Máscara", variable=self.show_mask_var, command=self._on_toggle_mask)
        self.checkbox_mask.grid(row=13, column=0, pady=(15,10), padx=10, sticky="w")
        
        # --- Grelha para os botões de preset ---
        preset_frame = ctk.CTkFrame(self, fg_color="transparent")
        preset_frame.grid(row=14, column=0, sticky="ew", padx=10)
        preset_frame.grid_columnconfigure((0, 1, 2), weight=1) 
        
        ctk.CTkButton(preset_frame, text="Sem Filtro", command=self._disable_filter, **COLOR_SECONDARY).grid(row=0, column=0, pady=3, padx=3, sticky="ew")
        
        # ## CORREÇÃO v4.2: Lógica de layout dos botões de cor ##
        # Este ciclo distribui os botões de cor numa grelha de 3 colunas,
        # tornando o layout compacto e organizado.
        color_items = list(BUTTON_COLORS.items())
        for i, (name, colors) in enumerate(color_items):
            row = (i + 1) // 3 # Divisão inteira para determinar a linha
            col = (i + 1) % 3  # Resto da divisão para determinar a coluna
            # O lambda cn=name captura o nome da cor no momento da criação do botão.
            ctk.CTkButton(preset_frame, text=name, fg_color=colors["fg"], hover_color=colors["hover"], text_color=colors["text"], command=lambda cn=name: self._apply_preset(cn)).grid(row=row, column=col, pady=3, padx=3, sticky="ew")

    def _toggle_filter_controls(self, enabled):
        """ Ativa ou desativa todos os controlos do filtro HSV. """
        state = "normal" if enabled else "disabled"
        for widget in self.hsv_sliders + [self.checkbox_mask]:
            widget.configure(state=state)
        self.filter_enabled = enabled

    def _disable_filter(self):
        """ Desativa o filtro de cor. """
        self._toggle_filter_controls(False)
        if self.video_handler: self.video_handler.update_once()

    def _apply_preset(self, color_name):
        """ Aplica um preset de cor aos sliders e valores. """
        self._toggle_filter_controls(True)
        preset = HSV_PRESETS.get(color_name)
        if not preset: return
        vals = [preset[k] for k in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max"]]
        for i, val in enumerate(vals):
            self.hsv_sliders[i].set(val)
            self.hsv_values[i].set(str(val))
        if self.video_handler: self.video_handler.update_once()

    def _on_toggle_mask(self):
        """ Atualiza o VideoHandler quando a checkbox 'Mostrar Máscara' é alterada. """
        if self.video_handler:
            self.video_handler.show_mask = self.show_mask_var.get()

    def get_hsv_bounds(self):
        """ Obtém os limites inferior e superior do HSV a partir dos sliders. """
        try:
            vals = [int(v.get()) for v in self.hsv_values]
        except (ValueError, tk.TclError): # Proteção contra valores inválidos ou widgets destruídos
            vals = [0, 255, 0, 255, 0, 255] # Retorna um filtro que não filtra nada
        return np.array([vals[0], vals[2], vals[4]]), np.array([vals[1], vals[3], vals[5]])

    def load_hsv(self, hsv_dict):
        """ Carrega valores HSV de um dicionário (e.g., do ficheiro de config). """
        self._toggle_filter_controls(True)
        vals = [hsv_dict.get(k, d) for k, d in [("h_min",0),("h_max",255),("s_min",0),("s_max",255),("v_min",0),("v_max",255)]]
        for i, val in enumerate(vals):
            self.hsv_sliders[i].set(val)
            self.hsv_values[i].set(str(val))

    def save_hsv(self):
        """ Retorna um dicionário com os valores HSV atuais para ser guardado. """
        lower, upper = self.get_hsv_bounds()
        return {
            "h_min": int(lower[0]), "h_max": int(upper[0]),
            "s_min": int(lower[1]), "s_max": int(upper[1]),
            "v_min": int(lower[2]), "v_max": int(upper[2])
        }

class VideoHandler:
    """ Gere a captura de vídeo, processamento de imagem e envio de comandos PID. """
    def __init__(self, parent, hsv_frame, serial_manager, app_ref):
        # --- Referências a outros objetos ---
        self.parent = parent
        self.hsv_frame = hsv_frame
        self.serial_manager = serial_manager
        self.app = app_ref # Referência à aplicação principal para aceder a dados globais como a calibração
        
        # --- Estado do VideoHandler ---
        self.camera_index = 0
        self.show_mask = False
        self.tracking_enabled = False
        self.running = False
        self.cap = None  # Objeto de captura de vídeo do OpenCV
        self.thread = None # Thread para o loop de vídeo
        
        # --- Controladores PID ---
        self.pid_x = PIDController(0, 0, 0)
        self.pid_y = PIDController(0, 0, 0)
        
        # --- Elementos da UI ---
        self.canvas = ctk.CTkCanvas(parent, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, bg="gray20", highlightthickness=0)
        self.canvas.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        
        status_frame = ctk.CTkFrame(parent, corner_radius=5)
        status_frame.grid(row=3, column=0, pady=(0, 10), padx=10, sticky="ew")
        status_frame.grid_columnconfigure((0, 1), weight=1)
        
        self.servo_x_var = ctk.StringVar(value="Erro X: 0")
        self.servo_y_var = ctk.StringVar(value="Erro Y: 0")
        self.pid_x_var = ctk.StringVar(value="PID X: 0.00")
        self.pid_y_var = ctk.StringVar(value="PID Y: 0.00")
        
        ctk.CTkLabel(status_frame, textvariable=self.servo_x_var).grid(row=0, column=0, padx=10, pady=2, sticky="w")
        ctk.CTkLabel(status_frame, textvariable=self.servo_y_var).grid(row=1, column=0, padx=10, pady=2, sticky="w")
        ctk.CTkLabel(status_frame, textvariable=self.pid_x_var).grid(row=0, column=1, padx=10, pady=2, sticky="w")
        ctk.CTkLabel(status_frame, textvariable=self.pid_y_var).grid(row=1, column=1, padx=10, pady=2, sticky="w")
        
        self._initialize_camera()

    def _process_and_draw(self, frame_bgr):
        """ Função principal do processamento de imagem para um único frame. """
        # Converte a cor de BGR (padrão OpenCV) para RGB (padrão PIL/Tkinter)
        display_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        
        if self.hsv_frame.filter_enabled:
            # 1. Converte para o espaço de cor HSV
            hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            lower, upper = self.hsv_frame.get_hsv_bounds()
            
            # 2. Cria uma máscara binária (preto e branco) com os pixels que estão dentro do intervalo HSV
            mask = cv2.inRange(hsv_frame, lower, upper)
            
            # 3. Limpa a máscara: remove pequenos ruídos (OPEN) e preenche buracos (CLOSE)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPse, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            if self.show_mask:
                display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
            
            # 4. Encontra os contornos (formas) na máscara
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            ball_detected = False
            if contours:
                # 5. Assume que o maior contorno é a bola
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 500: # Ignora contornos muito pequenos
                    ball_detected = True
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    
                    # 6. Calcula o erro em relação ao centro do vídeo
                    erro_x = int(x) - (VIDEO_WIDTH // 2)
                    erro_y = -(int(y) - (VIDEO_HEIGHT // 2)) # O eixo Y é invertido no sistema de coordenadas da imagem
                    self.servo_x_var.set(f"Erro X: {erro_x}")
                    self.servo_y_var.set(f"Erro Y: {erro_y}")
                    
                    # 7. Se o seguimento estiver ativo, calcula e envia os comandos
                    if self.tracking_enabled:
                        pid_output_x = self.pid_x.update(erro_x)
                        pid_output_y = self.pid_y.update(erro_y)
                        self.pid_x_var.set(f"PID X: {pid_output_x:.2f}")
                        self.pid_y_var.set(f"PID Y: {pid_y_var:.2f}")
                        self.send_servo_command(pid_output_x, pid_output_y)
                        
                    cv2.circle(display_img, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            
            if not ball_detected: # Se nenhuma bola for detetada, reinicia os valores
                self.servo_x_var.set("Erro X: 0")
                self.servo_y_var.set("Erro Y: 0")
                self.pid_x_var.set("PID X: 0.00")
                self.pid_y_var.set("PID Y: 0.00")
        else:
            self.servo_x_var.set("Erro X: (sem filtro)")
            self.servo_y_var.set("Erro Y: (sem filtro)")
            
        # Desenha uma cruz no centro do ecrã
        cx, cy = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2
        cv2.line(display_img, (cx - 10, cy), (cx + 10, cy), (255, 255, 0), 1)
        cv2.line(display_img, (cx, cy - 10), (cx, cy + 10), (255, 255, 0), 1)
        
        # 8. Exibe a imagem final no canvas da UI
        img_pil = Image.fromarray(display_img)
        img_tk = ImageTk.PhotoImage(image=img_pil)
        self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
        # Esta linha é CRUCIAL: impede que a imagem seja "recolhida" pelo garbage collector do Python.
        self.canvas.image = img_tk

    def send_servo_command(self, pid_output_x, pid_output_y):
        """ Mapeia a saída do PID para valores de servo e envia o comando. """
        calib = self.app.calibration_data
        pid_min, pid_max = -1000, 1000 # Intervalo arbitrário para a saída do PID
        
        # Mapeia a saída do PID para o intervalo de microssegundos definido na calibração
        target_x_us = self.map_value(pid_output_x, pid_min, pid_max, calib['x_min_us'], calib['x_max_us'])
        target_y_us = self.map_value(pid_output_y, pid_min, pid_max, calib['y_min_us'], calib['y_max_us'])
        
        # Adiciona o offset de centro para garantir que a plataforma está nivelada quando o erro é zero
        target_x_us += calib['x_center_offset_us']
        target_y_us += calib['y_center_offset_us']

        # Constrói e envia a string de comando para o Arduino
        cmd_str = f"{CMD_CALIBRATION_PREFIX},{int(target_x_us)},{int(target_y_us)}\n"
        self.serial_manager.send(cmd_str)
        
    @staticmethod
    def map_value(v, from_min, from_max, to_min, to_max):
        """ Mapeia um valor de um intervalo para outro (interpolação linear). """
        return (v - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
        
    def _initialize_camera(self):
        """ Para o loop atual, inicializa a câmara e inicia um novo loop. """
        self.stop() # Garante que a câmara anterior é libertada
        self.camera_index = int(self.app.cam_index_var.get())
        
        # CAP_DSHOW é uma API de captura do Windows que pode ser mais estável
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        
        if not self.cap.isOpened():
            messagebox.showerror("Erro de Câmara", f"Não foi possível aceder à câmara (índice {self.camera_index}).")
            self.cap = None
        else:
            self.running = True
            # Inicia o loop de vídeo numa thread separada para não bloquear a UI
            self.thread = threading.Thread(target=self._video_loop, daemon=True)
            self.thread.start()

    def _video_loop(self):
        """ O loop principal que continuamente lê frames da câmara. """
        while self.running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self._process_and_draw(frame)
            else: # Se a leitura falhar, para o loop
                self.running = False

    def stop(self):
        """ Para a thread de vídeo e liberta os recursos da câmara. """
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1) # Espera um pouco pela thread terminar
        if self.cap:
            self.cap.release()
        self.cap = None
        self.thread = None

    def update_once(self):
        """ Lê e processa um único frame (útil quando o loop não está a correr). """
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self._process_and_draw(frame)

    def change_camera(self, new_index):
        """ Muda para um novo índice de câmara. """
        if self.running:
            self._initialize_camera()

class ControlFrame(ctk.CTkFrame):
    """ Frame com os controlos principais: PID, motores e calibração. """
    def __init__(self, parent, serial_manager, app_ref, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.serial_manager = serial_manager
        self.app = app_ref
        
        # Usa um TabView para organizar os diferentes tipos de controlos
        self.tab_view = ctk.CTkTabview(self)
        self.tab_view.pack(expand=True, fill="both")
        self._create_operation_tab(self.tab_view.add("Operação"))
        self._create_calibration_tab(self.tab_view.add("Calibração"))

    def _create_operation_tab(self, tab):
        """ Cria o separador 'Operação' com controlos PID, motores e série. """
        tab.grid_columnconfigure(0, weight=1)
        
        # --- Grupo de Ganhos PID ---
        pid_group = ctk.CTkFrame(tab, fg_color="transparent")
        pid_group.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        pid_group.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(pid_group, text="Ganhos do Controlador", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=3, pady=5)
        
        self.pid_params_widgets = {}
        for i, param in enumerate(["Kp", "Ki", "Kd"]):
            ctk.CTkLabel(pid_group, text=param).grid(row=i+1, column=0, padx=(5,10), pady=5, sticky="w")
            slider = ctk.CTkSlider(pid_group, from_=0, to=1, number_of_steps=1000)
            slider.grid(row=i+1, column=1, padx=5, pady=5, sticky="ew")
            entry = ctk.CTkEntry(pid_group, width=60)
            entry.grid(row=i+1, column=2, padx=(5,10), pady=5, sticky="e")
            
            # Funções para sincronizar o slider e a caixa de texto
            def update_from_slider(v, p=param):
                w = self.pid_params_widgets[p]['entry']
                w.delete(0, 'end'); w.insert(0, f"{v:.3f}")
            def update_from_entry(e, p=param):
                try: self.pid_params_widgets[p]['slider'].set(float(self.pid_params_widgets[p]['entry'].get()))
                except(ValueError, KeyError, tk.TclError): pass
            
            slider.configure(command=update_from_slider)
            entry.bind("<Return>", update_from_entry)
            entry.bind("<FocusOut>", update_from_entry)
            self.pid_params_widgets[param] = {'slider': slider, 'entry': entry}
            self.pid_params_widgets[param]['entry'].insert(0, "0.000")
            
        ctk.CTkButton(pid_group, text="Aplicar Ganhos PID", command=self._apply_pid_gains, **COLOR_NEUTRAL).grid(row=4, column=0, columnspan=3, pady=10, sticky="ew")
        
        # --- Grupo de Controlo do Sistema ---
        motor_group = ctk.CTkFrame(tab, fg_color="transparent")
        motor_group.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        motor_group.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(motor_group, text="Controlo do Sistema", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=5)
        ctk.CTkButton(motor_group, text="Ligar Motores", command=lambda: self.serial_manager.send(CMD_MOTORS_ON), **COLOR_SUCCESS).grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Desligar Motores", command=lambda: self.serial_manager.send(CMD_MOTORS_OFF), **COLOR_DANGER).grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        self.btn_ligar_seguimento = ctk.CTkButton(motor_group, text="Ligar Seguimento", command=self._toggle_seguimento, **COLOR_SECONDARY)
        self.btn_ligar_seguimento.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        # --- Grupo de Comunicação Série ---
        serial_group = ctk.CTkFrame(tab, fg_color="transparent")
        serial_group.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        serial_group.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(serial_group, text="Comunicação Arduino", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=5)
        portas = self.serial_manager.list_ports()
        self.porta_var = tk.StringVar(value=portas[0] if portas else "Nenhuma porta")
        self.option_menu_porta = ctk.CTkOptionMenu(serial_group, values=portas if portas else ["Nenhuma porta"], variable=self.porta_var)
        self.option_menu_porta.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="🔄", width=30, command=self._atualizar_portas).grid(row=1, column=1, padx=5, pady=5)
        ctk.CTkButton(serial_group, text="Ligar", command=self._ligar_arduino, **COLOR_SUCCESS).grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="Desligar", command=self._desligar_arduino, **COLOR_DANGER).grid(row=2, column=1, padx=5, pady=5, sticky="ew")

    def _create_calibration_tab(self, tab):
        """ Cria o separador 'Calibração' para ajustar os limites e centro dos servos. """
        tab.grid_columnconfigure(0, weight=1)
        
        # --- Grupo de Nivelamento Central ---
        center_group = ctk.CTkFrame(tab, fg_color="transparent")
        center_group.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
        center_group.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(center_group, text="Nivelamento Central (Offset μs)", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=5)
        
        self.calib_vars = {} # Dicionário para guardar as variáveis Tkinter dos campos de calibração
        for i, name in enumerate(['x_center_offset_us', 'y_center_offset_us']):
            var = tk.IntVar(value=self.app.calibration_data.get(name, 0))
            self.calib_vars[name] = var
            label = "X" if "x_" in name else "Y"
            ctk.CTkLabel(center_group, text=f"Offset {label}:").grid(row=i+1, column=0, sticky="w", padx=5)
            entry = ctk.CTkEntry(center_group, textvariable=var, width=60)
            entry.grid(row=i+1, column=1, sticky="e", padx=5)
            # Envia o comando tanto ao pressionar Enter como ao sair do campo (FocusOut)
            entry.bind("<Return>", self._send_center_calib_command)
            entry.bind("<FocusOut>", self._send_center_calib_command)
            
        ctk.CTkButton(center_group, text="Testar Posição Central", command=self._send_center_calib_command, **COLOR_NEUTRAL).grid(row=3, column=0, columnspan=2, sticky="ew", pady=10, padx=5)
        
        # ## MELHORIA v4.2: UI de Limites com Botões de Teste ##
        # Esta secção cria uma UI intuitiva para calibrar os limites físicos dos servos.
        limits_group = ctk.CTkFrame(tab, fg_color="transparent")
        limits_group.grid(row=1, column=0, sticky="ew", padx=10, pady=10)
        limits_group.grid_columnconfigure((1, 3), weight=1) # Colunas das caixas de texto expandem
        ctk.CTkLabel(limits_group, text="Limites de Movimento (μs)", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=5, pady=5)
        
        for name in ['x_min_us', 'x_max_us', 'y_min_us', 'y_max_us']:
            var = tk.IntVar(value=self.app.calibration_data.get(name, 1500))
            self.calib_vars[name] = var
            
            # Determina a linha, coluna e etiqueta com base no nome do parâmetro
            axis, limit, row = ("X", "Mín", 1) if name == 'x_min_us' else \
                               (("X", "Máx", 1) if name == 'x_max_us' else \
                               (("Y", "Mín", 2) if name == 'y_min_us' else ("Y", "Máx", 2)))
            col_offset = 0 if "min" in name else 2
            
            ctk.CTkLabel(limits_group, text=f"{axis} {limit}:").grid(row=row, column=col_offset, sticky="w", padx=5)
            ctk.CTkEntry(limits_group, textvariable=var, width=70).grid(row=row, column=col_offset+1, sticky="ew", padx=5)
            
            # Adiciona um botão de teste "T" ao lado de cada campo.
            # O lambda n=name é crucial para passar o nome correto do limite ('x_min_us', etc.)
            # para a função de callback no momento da sua criação.
            ctk.CTkButton(limits_group, text="T", width=25, command=lambda n=name: self._send_limit_test_command(n)).grid(row=row, column=col_offset+2, padx=(0,5))
            
    def _apply_pid_gains(self):
        """ Aplica os ganhos PID introduzidos na UI aos controladores. """
        try:
            kp = float(self.pid_params_widgets['Kp']['entry'].get())
            ki = float(self.pid_params_widgets['Ki']['entry'].get())
            kd = float(self.pid_params_widgets['Kd']['entry'].get())
            self.app.video_handler.pid_x.set_gains(kp, ki, kd)
            self.app.video_handler.pid_y.set_gains(kp, ki, kd)
            self.app.status_bar.show_message("Ganhos PID aplicados.")
        except (ValueError, AttributeError, tk.TclError):
            self.app.status_bar.show_message("Erro: Ganhos PID inválidos.", is_error=True)

    def _send_center_calib_command(self, _=None):
        """ Envia o comando para mover os servos para a posição central definida. """
        # O valor base de um servo é 1500μs, o offset ajusta esse valor.
        x_us = 1500 + self.calib_vars['x_center_offset_us'].get()
        y_us = 1500 + self.calib_vars['y_center_offset_us'].get()
        self.serial_manager.send(f"{CMD_CALIBRATION_PREFIX},{x_us},{y_us}\n")

    def _send_limit_test_command(self, limit_name):
        """ Envia um comando para testar um limite específico (min/max) de um eixo. """
        # Começa com a posição central
        x_us = 1500 + self.calib_vars['x_center_offset_us'].get()
        y_us = 1500 + self.calib_vars['y_center_offset_us'].get()
        
        # Substitui o valor de um dos eixos pelo limite a ser testado
        if "x" in limit_name: x_us = self.calib_vars[limit_name].get()
        if "y" in limit_name: y_us = self.calib_vars[limit_name].get()
        
        self.serial_manager.send(f"{CMD_CALIBRATION_PREFIX},{x_us},{y_us}\n")

    def _toggle_seguimento(self):
        """ Liga ou desliga o seguimento da bola. """
        vh = self.app.video_handler
        vh.tracking_enabled = not vh.tracking_enabled
        if vh.tracking_enabled:
            self.btn_ligar_seguimento.configure(text="Parar Seguimento", **COLOR_DANGER)
            self.app.status_bar.show_message("Seguimento da bola ativado.")
        else:
            self.btn_ligar_seguimento.configure(text="Ligar Seguimento", **COLOR_SECONDARY)
            self.app.status_bar.show_message("Seguimento da bola desativado.")
            
    def _atualizar_portas(self):
        """ Atualiza a lista de portas série disponíveis na dropdown. """
        novas_portas = self.serial_manager.list_ports()
        self.option_menu_porta.configure(values=novas_portas if novas_portas else ["Nenhuma porta"])
        self.porta_var.set(novas_portas[0] if novas_portas else "Nenhuma porta")

    def _ligar_arduino(self):
        """ Tenta ligar-se à porta série selecionada. """
        porta = self.porta_var.get()
        if porta and porta != "Nenhuma porta":
            if self.serial_manager.connect(porta):
                self.app.status_bar.show_message(f"Conectado a {porta}.", 5000)
            else:
                self.app.status_bar.show_message(f"Falha ao conectar a {porta}.", 5000, is_error=True)

    def _desligar_arduino(self):
        """ Desliga a conexão série. """
        self.serial_manager.disconnect()
        self.app.status_bar.show_message("Conexão serial encerrada.")

    def get_pid_config(self):
        """ Retorna um dicionário com os ganhos PID atuais. """
        try:
            return {p: float(w['entry'].get()) for p, w in self.pid_params_widgets.items()}
        except (ValueError, KeyError, tk.TclError):
            return {"kp": 0.0, "ki": 0.0, "kd": 0.0}

    def load_pid_config(self, pid_dict):
        """ Carrega os ganhos PID de um dicionário para a UI. """
        for param, value in pid_dict.items():
            if param in self.pid_params_widgets:
                w = self.pid_params_widgets[param]
                w['slider'].set(value)
                w['entry'].delete(0, 'end'); w['entry'].insert(0, f"{value:.3f}")
        self._apply_pid_gains()

# -----------------------------------------------------------------------------
# 4. CLASSE PRINCIPAL DA APLICAÇÃO
# -----------------------------------------------------------------------------
class OpenBalanceApp:
    """ A classe principal que orquestra toda a aplicação. """
    def __init__(self):
        # --- Configuração Inicial da Janela ---
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.app = ctk.CTk()
        self.app.title("OpenBalance Dashboard 4.2 - Edição Final")
        self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        
        # --- Configuração da Grelha (Grid) Principal ---
        # A janela é dividida em 3 colunas (HSV, Vídeo, Controlo) e 2 linhas (principal, barra de status).
        self.app.grid_rowconfigure(0, weight=1) # Linha principal expande-se verticalmente
        self.app.grid_rowconfigure(1, weight=0) # Barra de status tem altura fixa
        self.app.grid_columnconfigure(0, weight=1, minsize=320) # Painel Esquerdo
        self.app.grid_columnconfigure(1, weight=4, minsize=VIDEO_WIDTH + 40) # Painel Central (Vídeo)
        self.app.grid_columnconfigure(2, weight=1, minsize=320) # Painel Direito

        # --- Instanciação dos Componentes Principais ---
        self.serial_manager = SerialManager()
        self.video_handler = None # Será inicializado em _create_frames

        # --- Carregamento e Criação da UI ---
        self._create_status_bar()
        self._load_app_config()
        self._create_menu()
        self._create_frames()
        
        # Define uma função para ser chamada quando a janela é fechada pelo utilizador
        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _load_app_config(self):
        """ Carrega as configurações (HSV, PID, Calibração) do ficheiro config.json. """
        # Valores padrão caso o ficheiro não exista
        self.calibration_data = {
            "x_center_offset_us": 0, "y_center_offset_us": 0,
            "x_min_us": 1000, "x_max_us": 2000,
            "y_min_us": 1000, "y_max_us": 2000
        }
        try:
            with open("config.json", 'r', encoding='utf-8') as f:
                config = json.load(f)
            self.hsv_config = config.get("hsv", {})
            self.pid_config = config.get("pid", {})
            self.calibration_data.update(config.get("calibration", {}))
        except (FileNotFoundError, json.JSONDecodeError):
            self.hsv_config = {}
            self.pid_config = {}
            self.status_bar.show_message("config.json não encontrado. Usando valores padrão.", is_error=True)

    def _save_app_config(self):
        """ Salva a configuração atual no ficheiro config.json. """
        # Atualiza os dados de calibração a partir das variáveis da UI antes de salvar
        if hasattr(self, 'control_frame') and self.control_frame:
            for name, var in self.control_frame.calib_vars.items():
                self.calibration_data[name] = var.get()
                
        config = {
            "hsv": self.hsv_frame.save_hsv() if hasattr(self, 'hsv_frame') else {},
            "pid": self.control_frame.get_pid_config() if hasattr(self, 'control_frame') else {},
            "calibration": self.calibration_data
        }
        try:
            with open("config.json", 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=4) # indent=4 para um ficheiro legível
            self.status_bar.show_message("Configuração salva em config.json.")
        except Exception as e:
            self.status_bar.show_message(f"Erro ao salvar configuração: {e}", is_error=True)
            
    def _create_frames(self):
        """ Cria e posiciona todos os frames principais da aplicação. """
        # 1. Frame de Configurações HSV (Esquerda)
        self.hsv_frame = HSVSettingsFrame(self.app, None, corner_radius=10)
        self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)
        
        # 2. Frame de Vídeo (Centro)
        frame_video_container = ctk.CTkFrame(self.app, corner_radius=10)
        frame_video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        frame_video_container.grid_columnconfigure(0, weight=1)
        frame_video_container.grid_rowconfigure(2, weight=1)
        
        ctk.CTkLabel(frame_video_container, text="Área de Visualização", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10, 5))
        
        # Controlo de seleção de câmara
        cam_select_frame = ctk.CTkFrame(frame_video_container, fg_color="transparent")
        cam_select_frame.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(cam_select_frame, text="Câmera:").pack(side="left", padx=(0, 5))
        self.cam_index_var = tk.StringVar(value="0")
        self.option_menu_camera = ctk.CTkOptionMenu(cam_select_frame, values=["0", "1", "2", "3"], variable=self.cam_index_var, command=lambda x: self.video_handler.change_camera(x))
        self.option_menu_camera.pack(side="left")
        
        # Instancia o VideoHandler, que cria o canvas de vídeo dentro do container
        self.video_handler = VideoHandler(frame_video_container, self.hsv_frame, self.serial_manager, self)
        self.hsv_frame.video_handler = self.video_handler # Cria a referência cruzada
        
        # 3. Frame de Controlo (Direita)
        self.control_frame = ControlFrame(self.app, self.serial_manager, self, corner_radius=10)
        self.control_frame.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)
        
        # Carrega as configurações nos frames depois de terem sido criados
        self.hsv_frame.load_hsv(self.hsv_config)
        self.control_frame.load_pid_config(self.pid_config)

    def _on_close(self):
        """ Função de limpeza executada ao fechar a aplicação. """
        self._save_app_config()
        if self.video_handler:
            self.video_handler.stop()
        self.serial_manager.disconnect()
        self.app.destroy()

    def _create_menu(self):
        """ Cria o menu superior da aplicação (Arquivo -> Salvar, Sair). """
        menu_bar = tk.Menu(self.app)
        arquivo_menu = tk.Menu(menu_bar, tearoff=0)
        arquivo_menu.add_command(label="Salvar Configuração Agora", command=self._save_app_config)
        arquivo_menu.add_separator()
        arquivo_menu.add_command(label="Sair", command=self._on_close)
        menu_bar.add_cascade(label="Arquivo", menu=arquivo_menu)
        self.app.config(menu=menu_bar)

    def _create_status_bar(self):
        """ Cria a barra de status na parte inferior. """
        self.status_bar = StatusBar(self.app)
        self.status_bar.grid(row=1, column=0, columnspan=3, sticky="sew")

    def run(self):
        """ Inicia o loop principal da aplicação. """
        self.app.mainloop()

# -----------------------------------------------------------------------------
# 5. PONTO DE ENTRADA DA APLICAÇÃO
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    # Este bloco só é executado quando o script é corrido diretamente.
    app = OpenBalanceApp()
    app.run()