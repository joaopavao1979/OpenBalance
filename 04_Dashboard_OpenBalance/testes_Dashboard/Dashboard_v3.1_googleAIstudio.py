"""
OpenBalance Dashboard (Versão 3.0 - Refatorado)
Descrição: Dashboard para controlo de bola numa plataforma inclinável,
utilizando visão computacional (HSV) e PID.
Autor: João Pavão (Refatorado por Engenheiro de Software/Designer Gráfico AI)
Finalidade: Interface gráfica com UI/UX melhorada para ajuste de parâmetros HSV,
controlo PID, visualização de vídeo, presets de cor, comunicação serial robusta,
e gestão de estado explícita.
Uso: Código aberto, disponível livremente sob licença MIT.
"""

import json
import threading
import cv2
import numpy as np
import customtkinter as ctk
import serial
import serial.tools.list_ports
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import filedialog, messagebox
import csv

# -----------------------------------------------------------------------------
# --------------------------- CONSTANTES E CONFIGURAÇÕES ---------------------
# -----------------------------------------------------------------------------
WINDOW_WIDTH = 1280  # ALTERADO: Largura aumentada para acomodar melhor os painéis
WINDOW_HEIGHT = 720 # ALTERADO: Altura aumentada para a barra de status
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
DEFAULT_BAUDRATE = 9600
CONFIG_FILE_TYPES = [("JSON Files", "*.json")]
CSV_FILE_TYPES = [("CSV Files", "*.csv")]

# Valores HSV de exemplo para presets (ajustar conforme iluminação/câmera)
HSV_PRESETS = {
    "Vermelho": {"h_min": 0, "h_max": 10, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Laranja": {"h_min": 10, "h_max": 25, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Verde": {"h_min": 40, "h_max": 80, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Cinzento": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 50, "v_min": 50, "v_max": 200},
    "Azul": {"h_min": 90, "h_max": 130, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 255},
    "Branco": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 30, "v_min": 200, "v_max": 255}
}

# Mapeamento de nome para cor de botão (hex)
BUTTON_COLORS = {
    "Vermelho": {"fg": "#FF0000", "hover": "#CC0000", "text": "white"},
    "Laranja": {"fg": "#FFA500", "hover": "#CC8400", "text": "white"},
    "Verde": {"fg": "#00B200", "hover": "#008000", "text": "white"},
    "Cinzento": {"fg": "#808080", "hover": "#666666", "text": "white"},
    "Azul": {"fg": "#0000FF", "hover": "#0000CC", "text": "white"},
    "Branco": {"fg": "#FFFFFF", "hover": "#DDDDDD", "text": "black"}
}

# NOVO: Paleta de Cores para consistência da UI nos painéis de controlo
# Verde para ações positivas (Ligar, Conectar), Vermelho para negativas (Desligar), Azul para neutras.
COLOR_SUCCESS = {"fg_color": "green", "hover_color": "darkgreen"}
COLOR_DANGER = {"fg_color": "#D32F2F", "hover_color": "#B71C1C"}
COLOR_NEUTRAL = {"fg_color": "#1E88E5", "hover_color": "#1565C0"}
COLOR_SECONDARY = {"fg_color": "gray50", "hover_color": "gray30"}

# NOVO: Constantes para comandos seriais para evitar "Magic Strings"
CMD_MOTORS_ON = "M1\n"
CMD_MOTORS_OFF = "M0\n"

# -----------------------------------------------------------------------------
# ---------------------------- GERENCIADOR SERIAL ----------------------------
# -----------------------------------------------------------------------------
class SerialManager:
    """
    Classe para gerenciamento de conexão serial com Arduino.
    Permite listar portas, conectar, desconectar e enviar dados.
    """
    def __init__(self):
        self.serial_conn = None

    def list_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]

    # ALTERADO: O método connect agora retorna um booleano em vez de mostrar um messagebox.
    # A responsabilidade de notificar o utilizador passa para a UI.
    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        """Abre conexão serial e retorna True em sucesso, False em caso de falha."""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            return True
        except Exception as e:
            self.serial_conn = None
            # O erro será tratado e exibido pela classe que chama este método.
            print(f"Erro Serial ao conectar: {e}") # Log no terminal
            return False

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.serial_conn = None

    def send(self, data_str):
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        try:
            self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException:
            # NOVO: Se a porta for desconectada fisicamente, o objeto de conexão é invalidado.
            self.disconnect()

# -----------------------------------------------------------------------------
# ----------------------- FRAME DE CONFIGURAÇÕES HSV --------------------------
# -----------------------------------------------------------------------------
class HSVSettingsFrame(ctk.CTkFrame):
    """
    Frame para ajuste de parâmetros HSV e preview de máscara.
    Inclui sliders HMin, HMax, SMin, SMax, VMin, VMax, checkbox 'Mostrar Máscara'
    e botões de presets de cor para valores predefinidos de HSV.
    """
    # (Nenhuma alteração significativa nesta classe, continua funcional como antes)
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.video_handler = video_handler
        self.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 16, "bold")).grid(
            row=0, column=0, pady=(10, 5), sticky="n"
        )

        self.hsv_sliders = []
        self.hsv_values = []
        labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
        for idx, label in enumerate(labels):
            ctk.CTkLabel(self, text=label).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(master=self, value="0")
            self.hsv_values.append(val_var)

            slider = ctk.CTkSlider(self, from_=0, to=255, number_of_steps=255, command=lambda val, var=val_var: var.set(f"{int(val)}"))
            slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew")
            self.hsv_sliders.append(slider)

            ctk.CTkLabel(self, textvariable=val_var).grid(row=2 + 2*idx, column=0, padx=(270,10), sticky="e")

        self.show_mask_var = ctk.BooleanVar(master=self, value=False)
        ctk.CTkCheckBox(self, text="Mostrar Máscara", variable=self.show_mask_var, command=self._on_toggle_mask).grid(row=13, column=0, pady=(10,5), padx=10, sticky="w")

        row_base = 14
        for i, color_name in enumerate(HSV_PRESETS.keys()):
            colors = BUTTON_COLORS[color_name]
            ctk.CTkButton(self, text=color_name, fg_color=colors["fg"], hover_color=colors["hover"], text_color=colors["text"], command=lambda cn=color_name: self._apply_preset(cn)).grid(row=row_base + i, column=0, pady=(2,2), padx=10, sticky="ew")

    def _apply_preset(self, color_name):
        preset = HSV_PRESETS.get(color_name, None)
        if not preset: return
        sliders_vals = [preset["h_min"], preset["h_max"], preset["s_min"], preset["s_max"], preset["v_min"], preset["v_max"]]
        for i, val in enumerate(sliders_vals):
            self.hsv_sliders[i].set(val)
            self.hsv_values[i].set(str(val))
        if self.video_handler: self.video_handler.update_once()

    def _on_toggle_mask(self):
        if self.video_handler: self.video_handler.show_mask = self.show_mask_var.get()

    def get_hsv_bounds(self):
        try:
            h_min, h_max, s_min, s_max, v_min, v_max = (
                int(self.hsv_values[0].get()), int(self.hsv_values[1].get()),
                int(self.hsv_values[2].get()), int(self.hsv_values[3].get()),
                int(self.hsv_values[4].get()), int(self.hsv_values[5].get())
            )
        except ValueError: h_min, h_max, s_min, s_max, v_min, v_max = 0, 255, 0, 255, 0, 255
        return np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max])

    def load_hsv(self, hsv_dict):
        try:
            sliders_vals = [hsv_dict.get("h_min", 0), hsv_dict.get("h_max", 255), hsv_dict.get("s_min", 0), hsv_dict.get("s_max", 255), hsv_dict.get("v_min", 0), hsv_dict.get("v_max", 255)]
            for i, val in enumerate(sliders_vals):
                self.hsv_sliders[i].set(val)
                self.hsv_values[i].set(str(val))
        except Exception as e: messagebox.showerror("Erro HSV", f"Falha ao carregar HSV: {e}")

    def save_hsv(self):
        lower, upper = self.get_hsv_bounds()
        return {"h_min": int(lower[0]), "h_max": int(upper[0]), "s_min": int(lower[1]), "s_max": int(upper[1]), "v_min": int(lower[2]), "v_max": int(upper[2])}

# -----------------------------------------------------------------------------
# ---------------------- FRAME PID CONTROLLER (REFATORADO) --------------------
# -----------------------------------------------------------------------------
class PIDControllerFrame(ctk.CTkFrame):
    """
    Frame para ajuste de parâmetros do controlador PID.
    (Versão Refatorada com Melhor Organização e UX)
    """
    def __init__(self, parent, serial_manager: SerialManager, app_ref, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.serial_manager = serial_manager
        self.app = app_ref  # NOVO: Referência à aplicação principal para usar a status bar
        self.grid_columnconfigure(0, weight=1)

        # --- GRUPO 1: Ganhos PID ---
        pid_group = ctk.CTkFrame(self, fg_color="transparent")
        pid_group.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        pid_group.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(pid_group, text="Ganhos do Controlador", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=3, pady=(5, 10))
        
        self.pid_params_widgets = {}
        for i, param in enumerate(["Kp", "Ki", "Kd"]):
            ctk.CTkLabel(pid_group, text=param).grid(row=i+1, column=0, padx=(5, 10), pady=5, sticky="w")
            
            slider = ctk.CTkSlider(pid_group, from_=0, to=10, number_of_steps=1000)
            slider.grid(row=i+1, column=1, padx=5, pady=5, sticky="ew")
            
            entry = ctk.CTkEntry(pid_group, width=60)
            entry.grid(row=i+1, column=2, padx=(5, 10), pady=5, sticky="e")
            
            # NOVO: Sincronização bidirecional entre slider e entry
            def update_from_slider(value, p=param): self.pid_params_widgets[p]['entry'].delete(0, 'end'); self.pid_params_widgets[p]['entry'].insert(0, f"{value:.2f}")
            def update_from_entry(event, p=param):
                try: self.pid_params_widgets[p]['slider'].set(float(self.pid_params_widgets[p]['entry'].get()))
                except ValueError: pass # Ignora se o valor for inválido
            
            slider.configure(command=update_from_slider)
            entry.bind("<Return>", update_from_entry)
            entry.bind("<FocusOut>", update_from_entry)

            self.pid_params_widgets[param] = {'slider': slider, 'entry': entry}
            self.pid_params_widgets[param]['entry'].insert(0, "0.00")

        # --- GRUPO 2: Controlo de Motores e Seguimento ---
        motor_group = ctk.CTkFrame(self, fg_color="transparent")
        motor_group.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        motor_group.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(motor_group, text="Controlo do Sistema", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(5, 10))

        ctk.CTkButton(motor_group, text="Ligar Motores", command=self._ligar_motores, **COLOR_SUCCESS).grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Desligar Motores", command=self._desligar_motores, **COLOR_DANGER).grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        # NOVO: Botão "Ligar Seguimento" agora é funcional e controla o estado
        self.btn_ligar_seguimento = ctk.CTkButton(motor_group, text="Ligar Seguimento", command=self._toggle_seguimento, **COLOR_SECONDARY)
        self.btn_ligar_seguimento.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        ctk.CTkButton(motor_group, text="Enviar PID", command=self._enviar_pid_arduino, **COLOR_NEUTRAL).grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # --- GRUPO 3: Comunicação Serial ---
        serial_group = ctk.CTkFrame(self, fg_color="transparent")
        serial_group.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        serial_group.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(serial_group, text="Comunicação Arduino", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(5, 10))

        portas = self.serial_manager.list_ports()
        self.porta_var = tk.StringVar(master=self, value=portas[0] if portas else "Nenhuma porta")
        self.option_menu_porta = ctk.CTkOptionMenu(serial_group, values=portas if portas else ["Nenhuma porta"], variable=self.porta_var)
        self.option_menu_porta.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="🔄", width=30, command=self._atualizar_portas).grid(row=1, column=1, padx=5, pady=5)

        ctk.CTkButton(serial_group, text="Ligar", command=self._ligar_arduino, **COLOR_SUCCESS).grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="Desligar", command=self._desligar_arduino, **COLOR_DANGER).grid(row=2, column=1, padx=5, pady=5, sticky="ew")

        # Adicionado espaço para expansão no final
        self.grid_rowconfigure(3, weight=1)

    # NOVO: Método para alternar o estado de seguimento da bola
    def _toggle_seguimento(self):
        vh = self.app.video_handler
        if not vh: return

        if not vh.tracking_enabled:
            vh.tracking_enabled = True
            self.btn_ligar_seguimento.configure(text="Parar Seguimento", **COLOR_DANGER)
            self.app.status_bar.show_message("Seguimento da bola ativado.")
        else:
            vh.tracking_enabled = False
            self.btn_ligar_seguimento.configure(text="Ligar Seguimento", **COLOR_SECONDARY)
            self.app.status_bar.show_message("Seguimento da bola desativado.")
    
    # ALTERADO: Métodos usam a status bar em vez de messagebox e constantes de comando
    def _ligar_motores(self):
        self.serial_manager.send(CMD_MOTORS_ON)
        self.app.status_bar.show_message("Comando para ligar motores enviado.")

    def _desligar_motores(self):
        self.serial_manager.send(CMD_MOTORS_OFF)
        self.app.status_bar.show_message("Comando para desligar motores enviado.")

    def _atualizar_portas(self):
        novas_portas = self.serial_manager.list_ports()
        self.option_menu_porta.configure(values=novas_portas if novas_portas else ["Nenhuma porta"])
        self.porta_var.set(novas_portas[0] if novas_portas else "Nenhuma porta")
        self.app.status_bar.show_message(f"Portas disponíveis atualizadas.")

    def _ligar_arduino(self):
        porta = self.porta_var.get()
        if porta and porta != "Nenhuma porta":
            if self.serial_manager.connect(porta):
                self.app.status_bar.show_message(f"Conectado a {porta} com sucesso.", 5000)
            else:
                self.app.status_bar.show_message(f"Falha ao conectar a {porta}.", 5000, is_error=True)
        else:
            self.app.status_bar.show_message("Nenhuma porta serial selecionada.", 3000, is_error=True)

    def _desligar_arduino(self):
        self.serial_manager.disconnect()
        self.app.status_bar.show_message("Conexão serial encerrada.")
    
    def get_pid_config(self):
        try:
            kp = float(self.pid_params_widgets['Kp']['entry'].get())
            ki = float(self.pid_params_widgets['Ki']['entry'].get())
            kd = float(self.pid_params_widgets['Kd']['entry'].get())
        except (ValueError, KeyError):
            kp, ki, kd = 0.0, 0.0, 0.0
        # NOVO: Os outros parâmetros foram removidos desta frame para simplificar.
        # Podem ser adicionados novamente se necessário.
        return {"kp": kp, "ki": ki, "kd": kd}

    def load_pid_config(self, pid_dict):
        try:
            for param, value in pid_dict.items():
                if param in self.pid_params_widgets:
                    self.pid_params_widgets[param]['slider'].set(value)
                    self.pid_params_widgets[param]['entry'].delete(0, 'end')
                    self.pid_params_widgets[param]['entry'].insert(0, f"{value:.2f}")
        except Exception as e:
            self.app.status_bar.show_message(f"Erro ao carregar PID: {e}", 5000, is_error=True)

    def _enviar_pid_arduino(self):
        pid_cfg = self.get_pid_config()
        # Assumindo que o Arduino espera Kp, Ki, Kd. Outros params podem ser adicionados aqui.
        cmd_str = f"PID,{pid_cfg['kp']:.2f},{pid_cfg['ki']:.2f},{pid_cfg['kd']:.2f}\n"
        self.serial_manager.send(cmd_str)
        self.app.status_bar.show_message(f"Parâmetros PID enviados: Kp={pid_cfg['kp']:.2f}, Ki={pid_cfg['ki']:.2f}, Kd={pid_cfg['kd']:.2f}")

# -----------------------------------------------------------------------------
# -------------------------- GERENCIADOR DE VÍDEO -----------------------------
# -----------------------------------------------------------------------------
class VideoHandler:
    """
    Classe que gerencia a captura de vídeo, processamento de imagem e envio de dados.
    """
    def __init__(self, parent, hsv_frame: HSVSettingsFrame, serial_manager: SerialManager, camera_index=0):
        self.parent = parent
        self.hsv_frame = hsv_frame
        self.serial_manager = serial_manager
        self.show_mask = False
        self.camera_index = camera_index
        self.cap = None
        self.thread = None
        self.running = False
        self.tracking_enabled = False # NOVO: Flag para controlar o envio de coordenadas

        self.canvas = ctk.CTkCanvas(parent, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, bg="gray20", highlightthickness=0)
        self.canvas.grid(row=2, column=0, padx=10, pady=10)
        
        # (Outros labels de status como Servo/PID foram removidos para simplificar, podem ser re-adicionados aqui)

        self._initialize_camera()

    def _initialize_camera(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.running = False
            if self.thread and self.thread.is_alive(): self.thread.join(timeout=1)

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW) # NOVO: CAP_DSHOW pode melhorar a compatibilidade no Windows
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            messagebox.showerror("Erro de Vídeo", f"Não foi possível aceder à webcam (índice {self.camera_index}).")
            self.cap = None
        else:
            self.running = True
            self.thread = threading.Thread(target=self._video_loop, daemon=True)
            self.thread.start()

    def change_camera(self, new_index):
        try: idx = int(new_index)
        except ValueError: messagebox.showerror("Câmera", f"Índice de câmera inválido: {new_index}"); return
        self.camera_index = idx
        self._initialize_camera()

    def update_once(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret: self._process_and_draw(frame)

    def _video_loop(self):
        while self.running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                self.running = False
                break
            self._process_and_draw(frame)

    def _process_and_draw(self, frame_bgr):
        hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower, upper = self.hsv_frame.get_hsv_bounds()
        mask = cv2.inRange(hsv_frame, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_center, ball_radius = None, None

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                ball_center, ball_radius = (int(x), int(y)), int(radius)
                
                erro_x = ball_center[0] - (VIDEO_WIDTH // 2)
                erro_y = -(ball_center[1] - (VIDEO_HEIGHT // 2)) # NOVO: Invertido Y para seguir convenção matemática (positivo para cima)

                # ALTERADO: Só envia coordenadas se o seguimento estiver explicitamente ativado
                if self.tracking_enabled and self.serial_manager.serial_conn:
                    cmd_str = f"E,{erro_x},{erro_y}\n"
                    self.serial_manager.send(cmd_str)

        display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB) if self.show_mask else cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        
        cx, cy = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2
        cv2.line(display_img, (cx - 10, cy), (cx + 10, cy), (255, 255, 0), 1)
        cv2.line(display_img, (cx, cy - 10), (cx, cy + 10), (255, 255, 0), 1)

        if ball_center and ball_radius:
            cv2.circle(display_img, ball_center, ball_radius, (0, 255, 0), 2)
            cv2.circle(display_img, ball_center, 3, (255, 0, 0), -1)

        img_pil = Image.fromarray(display_img)
        img_tk = ImageTk.PhotoImage(image=img_pil)
        self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
        self.canvas.image = img_tk

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive(): self.thread.join(timeout=1)
        if self.cap: self.cap.release()

# NOVO: Classe para a Barra de Status no rodapé da aplicação
class StatusBar(ctk.CTkFrame):
    """Uma barra de status simples para exibir mensagens temporárias."""
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, corner_radius=0, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w")
        self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None

    def show_message(self, message, duration_ms=4000, is_error=False):
        """Exibe uma mensagem por um período. Mensagens de erro são mostradas em vermelho."""
        if self._job: self.after_cancel(self._job)
        
        original_color = self.label.cget("text_color")
        text_color = "#FF5555" if is_error else original_color
        
        self.label.configure(text=message, text_color=text_color)
        self._job = self.after(duration_ms, lambda: self.label.configure(text="", text_color=original_color))

# -----------------------------------------------------------------------------
# ----------------------------- APLICAÇÃO PRINCIPAL ---------------------------
# -----------------------------------------------------------------------------
class OpenBalanceApp:
    def __init__(self):
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        self.app = ctk.CTk()
        self.app.title("OpenBalance 3.0")
        self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        
        # ALTERADO: Configuração de grid para incluir a barra de status na linha 1
        self.app.grid_rowconfigure(0, weight=1)
        self.app.grid_rowconfigure(1, weight=0) # Linha para a barra de status
        self.app.grid_columnconfigure(0, weight=1)
        self.app.grid_columnconfigure(1, weight=3)
        self.app.grid_columnconfigure(2, weight=1)

        self.serial_manager = SerialManager()
        self.video_handler = None # Será inicializado em _create_frames

        self._create_menu()
        self._create_frames()
        self._create_status_bar() # NOVO: Cria a barra de status

        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _create_menu(self):
        menu_bar = tk.Menu(self.app)
        arquivo_menu = tk.Menu(menu_bar, tearoff=0)
        arquivo_menu.add_command(label="Salvar Configuração", command=self._salvar_config)
        arquivo_menu.add_command(label="Carregar Configuração", command=self._carregar_config)
        arquivo_menu.add_separator()
        arquivo_menu.add_command(label="Sair", command=self._on_close)
        menu_bar.add_cascade(label="Arquivo", menu=arquivo_menu)
        # (Outros menus podem ser adicionados aqui)
        self.app.config(menu=menu_bar)

    # NOVO: Método para criar a barra de status
    def _create_status_bar(self):
        self.status_bar = StatusBar(self.app)
        self.status_bar.grid(row=1, column=0, columnspan=3, sticky="sew")

    def _create_frames(self):
        # Frame HSV (coluna 0)
        self.hsv_frame = HSVSettingsFrame(self.app, video_handler=None)
        self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)

        # Frame de vídeo (coluna 1)
        frame_video_container = ctk.CTkFrame(self.app, corner_radius=10)
        frame_video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        frame_video_container.grid_rowconfigure(2, weight=1)
        frame_video_container.grid_columnconfigure(0, weight=1)
        
        ctk.CTkLabel(frame_video_container, text="Área de Visualização", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10,5))
        
        cam_select_frame = ctk.CTkFrame(frame_video_container, fg_color="transparent")
        cam_select_frame.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(cam_select_frame, text="Câmera:").pack(side="left", padx=(0, 5))
        self.cam_index_var = tk.StringVar(value="0")
        self.option_menu_camera = ctk.CTkOptionMenu(cam_select_frame, values=["0", "1", "2", "3"], variable=self.cam_index_var, command=self._on_camera_change)
        self.option_menu_camera.pack(side="left")

        # Instancia o VideoHandler e passa a referência para o HSV frame
        self.video_handler = VideoHandler(frame_video_container, self.hsv_frame, self.serial_manager, camera_index=int(self.cam_index_var.get()))
        self.hsv_frame.video_handler = self.video_handler

        # Frame PID (coluna 2)
        self.pid_frame = PIDControllerFrame(self.app, self.serial_manager, app_ref=self)
        self.pid_frame.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)

    def _on_camera_change(self, new_index):
        if self.video_handler: self.video_handler.change_camera(new_index)
    
    # ALTERADO: Métodos usam a status bar para feedback
    def _salvar_config(self):
        config = {"hsv": self.hsv_frame.save_hsv(), "pid": self.pid_frame.get_pid_config()}
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=CONFIG_FILE_TYPES)
        if not filepath: return
        try:
            with open(filepath, 'w', encoding='utf-8') as f: json.dump(config, f, indent=4)
            self.status_bar.show_message(f"Configuração salva em {filepath}")
        except Exception as e:
            self.status_bar.show_message(f"Erro ao salvar: {e}", is_error=True)

    def _carregar_config(self):
        filepath = filedialog.askopenfilename(filetypes=CONFIG_FILE_TYPES)
        if not filepath: return
        try:
            with open(filepath, 'r', encoding='utf-8') as f: config = json.load(f)
            self.hsv_frame.load_hsv(config.get("hsv", {}))
            self.pid_frame.load_pid_config(config.get("pid", {}))
            self.status_bar.show_message("Configuração carregada com sucesso.")
            if self.video_handler: self.video_handler.update_once()
        except Exception as e:
            self.status_bar.show_message(f"Erro ao carregar: {e}", is_error=True)

    def _on_close(self):
        if self.video_handler: self.video_handler.stop()
        self.serial_manager.disconnect()
        self.app.destroy()

    def run(self):
        self.app.mainloop()

# -----------------------------------------------------------------------------
# -------------------------- PONTO DE ENTRADA DO SCRIPT -----------------------
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    app = OpenBalanceApp()
    app.run()