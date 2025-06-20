"""
OpenBalance Dashboard (Versão 3.2 - Edição Académica com Foco na Visualização)
=============================================================================

Descrição:
    Dashboard para o controlo em tempo real de uma plataforma de equilíbrio de bola.
    Esta versão refatora a interface para dar maior proeminência ao feed de vídeo,
    aumentando o seu tamanho e movendo controlos secundários para o painel lateral,
    melhorando a hierarquia visual e a experiência do utilizador.

Autor:
    João Pavão (Conceito Original)
    Refatorado e Comentado por Professor AI (para fins didáticos)

Arquitetura e Conceitos-Chave:
    (Os conceitos da v3.1 mantêm-se, com ênfase na melhoria da UI/UX)
"""

# -----------------------------------------------------------------------------
# Módulo 1: Importações e Configurações Globais
# -----------------------------------------------------------------------------
import json
import threading
import csv
import cv2
import numpy as np
import customtkinter as ctk
import serial
import serial.tools.list_ports
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import filedialog, messagebox

# --- Configurações da Janela e Vídeo ---
## ALTERAÇÃO DIDÁTICA v3.2: Aumento das dimensões ##
# Para dar mais destaque ao vídeo, aumentamos a sua resolução base e o tamanho
# da janela principal para o acomodar confortavelmente.
WINDOW_WIDTH = 1500
WINDOW_HEIGHT = 850
VIDEO_WIDTH = 800
VIDEO_HEIGHT = 600

# --- Configurações de Comunicação e Ficheiros ---
DEFAULT_BAUDRATE = 9600
CONFIG_FILE_TYPES = [("JSON Files", "*.json")]
CSV_FILE_TYPES = [("CSV Files", "*.csv")]

# --- Constantes de Protocolo Serial ---
CMD_MOTORS_ON = "M1\n"
CMD_MOTORS_OFF = "M0\n"
CMD_ERROR_PREFIX = "E"
CMD_PID_PREFIX = "PID"

# --- Paleta de Cores da UI ---
COLOR_SUCCESS = {"fg_color": "#2E7D32", "hover_color": "#1B5E20"}
COLOR_DANGER = {"fg_color": "#C62828", "hover_color": "#B71C1C"}
COLOR_NEUTRAL = {"fg_color": "#1E88E5", "hover_color": "#1565C0"}
COLOR_SECONDARY = {"fg_color": "#616161", "hover_color": "#424242"}

# --- Presets de Deteção de Cor (HSV) ---
HSV_PRESETS = {
    "Vermelho": {"h_min": 0, "h_max": 10, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Laranja": {"h_min": 10, "h_max": 25, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Verde": {"h_min": 40, "h_max": 80, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Cinzento": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 50, "v_min": 50, "v_max": 200},
    "Azul": {"h_min": 90, "h_max": 130, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 255},
    "Branco": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 30, "v_min": 200, "v_max": 255}
}
BUTTON_COLORS = {
    "Vermelho": {"fg": "#FF0000", "hover": "#CC0000", "text": "white"},
    "Laranja": {"fg": "#FFA500", "hover": "#CC8400", "text": "white"},
    "Verde": {"fg": "#00B200", "hover": "#008000", "text": "white"},
    "Cinzento": {"fg": "#808080", "hover": "#666666", "text": "white"},
    "Azul": {"fg": "#0000FF", "hover": "#0000CC", "text": "white"},
    "Branco": {"fg": "#FFFFFF", "hover": "#DDDDDD", "text": "black"}
}

# (As classes SerialManager, HSVSettingsFrame, StatusBar e VideoHandler não necessitam de alterações
# para esta refatoração, pois foram desenhadas para se adaptarem às constantes e à estrutura que as contém.
# Iremos reescrevê-las aqui para que o ficheiro esteja completo.)

class SerialManager:
    def __init__(self): self.serial_conn = None
    def list_ports(self): return [port.device for port in serial.tools.list_ports.comports()]
    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        try:
            if self.serial_conn and self.serial_conn.is_open: self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            return True
        except Exception as e:
            self.serial_conn = None; print(f"Erro ao conectar na porta serial: {e}"); return False
    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open: self.serial_conn.close()
        self.serial_conn = None
    def send(self, data_str):
        if not self.serial_conn or not self.serial_conn.is_open: return
        try: self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException: self.disconnect()

class HSVSettingsFrame(ctk.CTkFrame):
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.video_handler = video_handler; self.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10, 15), sticky="n")
        self.hsv_sliders = []; self.hsv_values = []
        labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
        for idx, label in enumerate(labels):
            ctk.CTkLabel(self, text=label).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(value="0"); self.hsv_values.append(val_var)
            slider = ctk.CTkSlider(self, from_=0, to=255, command=lambda val, var=val_var: var.set(f"{int(val)}")); slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew"); self.hsv_sliders.append(slider)
            ctk.CTkLabel(self, textvariable=val_var).grid(row=2 + 2*idx, column=0, padx=(270,10), sticky="e")
        self.show_mask_var = ctk.BooleanVar(value=False); ctk.CTkCheckBox(self, text="Mostrar Máscara", variable=self.show_mask_var, command=self._on_toggle_mask).grid(row=13, column=0, pady=(15,10), padx=10, sticky="w")
        preset_frame = ctk.CTkFrame(self, fg_color="transparent"); preset_frame.grid(row=14, column=0, sticky="ew", padx=10); preset_frame.grid_columnconfigure((0, 1), weight=1)
        for i, (color_name, colors) in enumerate(BUTTON_COLORS.items()):
            row, col = divmod(i, 2)
            ctk.CTkButton(preset_frame, text=color_name, fg_color=colors["fg"], hover_color=colors["hover"], text_color=colors["text"], command=lambda cn=color_name: self._apply_preset(cn)).grid(row=row, column=col, pady=3, padx=3, sticky="ew")
    def _apply_preset(self, color_name):
        preset = HSV_PRESETS.get(color_name);
        if not preset: return
        vals = [preset["h_min"], preset["h_max"], preset["s_min"], preset["s_max"], preset["v_min"], preset["v_max"]]
        for i, val in enumerate(vals): self.hsv_sliders[i].set(val); self.hsv_values[i].set(str(val))
        if self.video_handler: self.video_handler.update_once()
    def _on_toggle_mask(self):
        if self.video_handler: self.video_handler.show_mask = self.show_mask_var.get()
    def get_hsv_bounds(self):
        try: vals = [int(v.get()) for v in self.hsv_values]
        except ValueError: vals = [0, 255, 0, 255, 0, 255]
        return np.array([vals[0], vals[2], vals[4]]), np.array([vals[1], vals[3], vals[5]])
    def load_hsv(self, hsv_dict):
        vals = [hsv_dict.get(k, d) for k, d in [("h_min",0),("h_max",255),("s_min",0),("s_max",255),("v_min",0),("v_max",255)]]
        for i, val in enumerate(vals): self.hsv_sliders[i].set(val); self.hsv_values[i].set(str(val))
    def save_hsv(self):
        l, u = self.get_hsv_bounds(); return {"h_min": int(l[0]),"h_max": int(u[0]),"s_min": int(l[1]),"s_max": int(u[1]),"v_min": int(l[2]),"v_max": int(u[2])}

class StatusBar(ctk.CTkFrame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w"); self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None
    def show_message(self, message, duration_ms=4000, is_error=False):
        if self._job: self.after_cancel(self._job)
        text_color = COLOR_DANGER["fg_color"] if is_error else "gray70"
        self.label.configure(text=message, text_color=text_color)
        self._job = self.after(duration_ms, lambda: self.label.configure(text=""))

class VideoHandler:
    def __init__(self, parent, hsv_frame: HSVSettingsFrame, serial_manager: SerialManager, camera_index=0):
        self.parent = parent; self.hsv_frame = hsv_frame; self.serial_manager = serial_manager; self.camera_index = camera_index
        self.show_mask = False; self.tracking_enabled = False; self.running = False
        self.cap = None; self.thread = None
        self.canvas = ctk.CTkCanvas(parent, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, bg="gray20", highlightthickness=0); self.canvas.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        self.status_frame = ctk.CTkFrame(parent, corner_radius=5); self.status_frame.grid(row=3, column=0, pady=(0, 10), padx=10, sticky="ew"); self.status_frame.grid_columnconfigure((0, 1), weight=1)
        self.servo_x_var = ctk.StringVar(value="Servo X: 0"); self.servo_y_var = ctk.StringVar(value="Servo Y: 0"); self.pid_x_var = ctk.StringVar(value="PID X: 0.00"); self.pid_y_var = ctk.StringVar(value="PID Y: 0.00")
        ctk.CTkLabel(self.status_frame, textvariable=self.servo_x_var).grid(row=0, column=0, padx=10, pady=2, sticky="w"); ctk.CTkLabel(self.status_frame, textvariable=self.servo_y_var).grid(row=1, column=0, padx=10, pady=2, sticky="w")
        ctk.CTkLabel(self.status_frame, textvariable=self.pid_x_var).grid(row=0, column=1, padx=10, pady=2, sticky="w"); ctk.CTkLabel(self.status_frame, textvariable=self.pid_y_var).grid(row=1, column=1, padx=10, pady=2, sticky="w")
        self._initialize_camera()
    def _initialize_camera(self):
        self.stop(); self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW); self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        if not self.cap.isOpened(): messagebox.showerror("Erro de Câmara", f"Não foi possível aceder à câmara (índice {self.camera_index})."); self.cap = None
        else: self.running = True; self.thread = threading.Thread(target=self._video_loop, daemon=True); self.thread.start()
    def _video_loop(self):
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret: self.running = False; break
            self._process_and_draw(frame)
    def _process_and_draw(self, frame_bgr):
        hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower, upper = self.hsv_frame.get_hsv_bounds(); mask = cv2.inRange(hsv_frame, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)); mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel); mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_detected = False
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                ball_detected = True; ((x, y), radius) = cv2.minEnclosingCircle(c)
                erro_x = int(x) - (VIDEO_WIDTH // 2); erro_y = - (int(y) - (VIDEO_HEIGHT // 2))
                self.servo_x_var.set(f"Servo X: {erro_x}"); self.servo_y_var.set(f"Servo Y: {erro_y}")
                if self.tracking_enabled and self.serial_manager.serial_conn: self.serial_manager.send(f"{CMD_ERROR_PREFIX},{erro_x},{erro_y}\n")
                display_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB) if not self.show_mask else cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                cv2.circle(display_img, (int(x), int(y)), int(radius), (0, 255, 0), 2); cv2.circle(display_img, (int(x), int(y)), 3, (255, 0, 0), -1)
        if not ball_detected:
            self.servo_x_var.set("Servo X: 0"); self.servo_y_var.set("Servo Y: 0"); self.pid_x_var.set("PID X: 0.00"); self.pid_y_var.set("PID Y: 0.00")
            display_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB) if not self.show_mask else cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        cx, cy = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2; cv2.line(display_img, (cx-10, cy), (cx+10, cy), (255,255,0), 1); cv2.line(display_img, (cx, cy-10), (cx, cy+10), (255,255,0), 1)
        img_pil = Image.fromarray(display_img); img_tk = ImageTk.PhotoImage(image=img_pil)
        self.canvas.create_image(0, 0, anchor="nw", image=img_tk); self.canvas.image = img_tk
    def change_camera(self, new_index):
        try: self.camera_index = int(new_index); self._initialize_camera()
        except ValueError: messagebox.showerror("Erro", f"Índice de câmara inválido: {new_index}")
    def update_once(self):
        if self.cap and self.cap.isOpened(): ret, frame = self.cap.read();_ = ret and self._process_and_draw(frame)
    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive(): self.thread.join(timeout=1)
        if self.cap: self.cap.release()
        self.cap = None; self.thread = None

# -----------------------------------------------------------------------------
# Classe PIDControllerFrame (ALTERADA)
# -----------------------------------------------------------------------------
class PIDControllerFrame(ctk.CTkFrame):
    ## ALTERAÇÃO DIDÁTICA v3.2: Reorganização da UI ##
    # Esta classe agora também é responsável pelos "Controlos de Trajetória",
    # que foram movidos do painel central para aqui. Isto centraliza a lógica
    # de controlo e liberta espaço visual no painel de vídeo.
    def __init__(self, parent, serial_manager: SerialManager, app_ref, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.serial_manager = serial_manager; self.app = app_ref
        self.grid_columnconfigure(0, weight=1)

        # GRUPO 1: Ganhos do Controlador PID
        pid_group = ctk.CTkFrame(self, fg_color="transparent"); pid_group.grid(row=0, column=0, padx=10, pady=10, sticky="ew"); pid_group.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(pid_group, text="Ganhos do Controlador", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=3, pady=(5, 10))
        self.pid_params_widgets = {}
        for i, param in enumerate(["Kp", "Ki", "Kd"]):
            ctk.CTkLabel(pid_group, text=param).grid(row=i+1, column=0, padx=(5, 10), pady=5, sticky="w")
            slider = ctk.CTkSlider(pid_group, from_=0, to=10, number_of_steps=1000); slider.grid(row=i+1, column=1, padx=5, pady=5, sticky="ew")
            entry = ctk.CTkEntry(pid_group, width=60); entry.grid(row=i+1, column=2, padx=(5, 10), pady=5, sticky="e")
            def update_from_slider(value, p=param): w=self.pid_params_widgets[p]['entry']; w.delete(0,'end'); w.insert(0,f"{value:.2f}")
            def update_from_entry(event, p=param):
                try: self.pid_params_widgets[p]['slider'].set(float(self.pid_params_widgets[p]['entry'].get()))
                except (ValueError, KeyError): pass
            slider.configure(command=update_from_slider); entry.bind("<Return>", update_from_entry); entry.bind("<FocusOut>", update_from_entry)
            self.pid_params_widgets[param] = {'slider': slider, 'entry': entry}; self.pid_params_widgets[param]['entry'].insert(0, "0.00")

        # GRUPO 2: Controlo do Sistema Físico
        motor_group = ctk.CTkFrame(self, fg_color="transparent"); motor_group.grid(row=1, column=0, padx=10, pady=10, sticky="ew"); motor_group.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(motor_group, text="Controlo do Sistema", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(5, 10))
        ctk.CTkButton(motor_group, text="Ligar Motores", command=self._ligar_motores, **COLOR_SUCCESS).grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Desligar Motores", command=self._desligar_motores, **COLOR_DANGER).grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        self.btn_ligar_seguimento = ctk.CTkButton(motor_group, text="Ligar Seguimento", command=self._toggle_seguimento, **COLOR_SECONDARY); self.btn_ligar_seguimento.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Enviar PID", command=self._enviar_pid_arduino, **COLOR_NEUTRAL).grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        ## ALTERAÇÃO DIDÁTICA v3.2: Novo grupo para trajetória ##
        # Este novo frame agrupa os controlos de trajetória, que antes estavam no painel central.
        traj_group = ctk.CTkFrame(self, fg_color="transparent"); traj_group.grid(row=2, column=0, padx=10, pady=10, sticky="ew"); traj_group.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(traj_group, text="Controlo de Trajetória", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(5, 10))
        self.ctrl_menu = ctk.CTkOptionMenu(traj_group, values=["Manual", "Círculo", "Oito", "Quadrado"]); self.ctrl_menu.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(traj_group, text="Executar", command=lambda: self.app._executar_trajetoria(), **COLOR_NEUTRAL).grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(traj_group, text="Parar", command=lambda: self.app._parar_trajetoria(), **COLOR_SECONDARY).grid(row=2, column=1, padx=5, pady=5, sticky="ew")

        # GRUPO 3: Comunicação Serial
        serial_group = ctk.CTkFrame(self, fg_color="transparent"); serial_group.grid(row=3, column=0, padx=10, pady=10, sticky="ew"); serial_group.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(serial_group, text="Comunicação Arduino", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(5, 10))
        portas = self.serial_manager.list_ports(); self.porta_var = tk.StringVar(value=portas[0] if portas else "Nenhuma porta")
        self.option_menu_porta = ctk.CTkOptionMenu(serial_group, values=portas if portas else ["Nenhuma porta"], variable=self.porta_var); self.option_menu_porta.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="🔄", width=30, command=self._atualizar_portas).grid(row=1, column=1, padx=5, pady=5)
        ctk.CTkButton(serial_group, text="Ligar", command=self._ligar_arduino, **COLOR_SUCCESS).grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="Desligar", command=self._desligar_arduino, **COLOR_DANGER).grid(row=2, column=1, padx=5, pady=5, sticky="ew")

    # (Os métodos desta classe não precisam de alterações, apenas o __init__ foi modificado)
    def _toggle_seguimento(self):
        vh = self.app.video_handler
        if not vh: return
        vh.tracking_enabled = not vh.tracking_enabled
        if vh.tracking_enabled: self.btn_ligar_seguimento.configure(text="Parar Seguimento", **COLOR_DANGER); self.app.status_bar.show_message("Seguimento da bola ativado.")
        else: self.btn_ligar_seguimento.configure(text="Ligar Seguimento", **COLOR_SECONDARY); self.app.status_bar.show_message("Seguimento da bola desativado.")
    def _ligar_motores(self): self.serial_manager.send(CMD_MOTORS_ON); self.app.status_bar.show_message("Comando para ligar motores enviado.")
    def _desligar_motores(self): self.serial_manager.send(CMD_MOTORS_OFF); self.app.status_bar.show_message("Comando para desligar motores enviado.")
    def _atualizar_portas(self):
        novas_portas = self.serial_manager.list_ports(); self.option_menu_porta.configure(values=novas_portas if novas_portas else ["Nenhuma porta"]); self.porta_var.set(novas_portas[0] if novas_portas else "Nenhuma porta")
        self.app.status_bar.show_message("Lista de portas seriais atualizada.")
    def _ligar_arduino(self):
        porta = self.porta_var.get()
        if porta and porta != "Nenhuma porta":
            if self.serial_manager.connect(porta): self.app.status_bar.show_message(f"Conectado a {porta} com sucesso.", 5000)
            else: self.app.status_bar.show_message(f"Falha ao conectar a {porta}.", 5000, is_error=True)
        else: self.app.status_bar.show_message("Nenhuma porta serial selecionada.", 3000, is_error=True)
    def _desligar_arduino(self): self.serial_manager.disconnect(); self.app.status_bar.show_message("Conexão serial encerrada.")
    def get_pid_config(self):
        try: return {p: float(w['entry'].get()) for p, w in self.pid_params_widgets.items()}
        except (ValueError, KeyError): return {"kp": 0.0, "ki": 0.0, "kd": 0.0}
    def load_pid_config(self, pid_dict):
        for param, value in pid_dict.items():
            if param in self.pid_params_widgets: w = self.pid_params_widgets[param]; w['slider'].set(value); w['entry'].delete(0, 'end'); w['entry'].insert(0, f"{value:.2f}")
    def _enviar_pid_arduino(self):
        pid_cfg = self.get_pid_config(); cmd_str = f"{CMD_PID_PREFIX},{pid_cfg['kp']:.2f},{pid_cfg['ki']:.2f},{pid_cfg['kd']:.2f}\n"; self.serial_manager.send(cmd_str)
        self.app.status_bar.show_message(f"Parâmetros PID enviados: Kp={pid_cfg['kp']:.2f}, Ki={pid_cfg['ki']:.2f}, Kd={pid_cfg['kd']:.2f}")

# -----------------------------------------------------------------------------
# Classe OpenBalanceApp (ALTERADA)
# -----------------------------------------------------------------------------
class OpenBalanceApp:
    def __init__(self):
        ctk.set_appearance_mode("dark"); ctk.set_default_color_theme("blue")
        self.app = ctk.CTk(); self.app.title("OpenBalance Dashboard 3.2"); self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.app.grid_rowconfigure(0, weight=1); self.app.grid_rowconfigure(1, weight=0)
        ## ALTERAÇÃO DIDÁTICA v3.2: Ajuste dos pesos e tamanhos mínimos das colunas ##
        # A coluna central (1) agora tem um peso maior e um tamanho mínimo que corresponde
        # ao novo tamanho do vídeo, garantindo que a UI se comporta como esperado.
        self.app.grid_columnconfigure(0, weight=1, minsize=320)
        self.app.grid_columnconfigure(1, weight=4, minsize=VIDEO_WIDTH + 40)
        self.app.grid_columnconfigure(2, weight=1, minsize=320)
        self.serial_manager = SerialManager(); self.video_handler = None
        self._create_menu(); self._create_frames(); self._create_status_bar()
        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _create_menu(self):
        menu_bar = tk.Menu(self.app); arquivo_menu = tk.Menu(menu_bar, tearoff=0)
        arquivo_menu.add_command(label="Salvar Configuração", command=self._salvar_config); arquivo_menu.add_command(label="Carregar Configuração", command=self._carregar_config)
        arquivo_menu.add_separator(); arquivo_menu.add_command(label="Sair", command=self._on_close)
        menu_bar.add_cascade(label="Arquivo", menu=arquivo_menu); self.app.config(menu=menu_bar)

    def _create_status_bar(self):
        self.status_bar = StatusBar(self.app); self.status_bar.grid(row=1, column=0, columnspan=3, sticky="sew")

    def _create_frames(self):
        # Painel HSV (Esquerda)
        self.hsv_frame = HSVSettingsFrame(self.app, None, corner_radius=10); self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)

        ## ALTERAÇÃO DIDÁTICA v3.2: Simplificação do painel central ##
        # Este painel agora é muito mais "limpo". A sua única responsabilidade é
        # conter o título, o seletor de câmara e o VideoHandler. Os controlos de
        # trajetória foram movidos.
        frame_video_container = ctk.CTkFrame(self.app, corner_radius=10); frame_video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        frame_video_container.grid_columnconfigure(0, weight=1)
        frame_video_container.grid_rowconfigure(2, weight=1) # Permite que o canvas do vídeo expanda
        ctk.CTkLabel(frame_video_container, text="Área de Visualização", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10,5))
        cam_select_frame = ctk.CTkFrame(frame_video_container, fg_color="transparent"); cam_select_frame.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(cam_select_frame, text="Câmera:").pack(side="left", padx=(0, 5)); self.cam_index_var = tk.StringVar(value="0")
        self.option_menu_camera = ctk.CTkOptionMenu(cam_select_frame, values=["0", "1", "2", "3"], variable=self.cam_index_var, command=self._on_camera_change); self.option_menu_camera.pack(side="left")
        
        self.video_handler = VideoHandler(frame_video_container, self.hsv_frame, self.serial_manager, int(self.cam_index_var.get()))
        self.hsv_frame.video_handler = self.video_handler

        # Painel PID (Direita) - Agora contém também os controlos de trajetória
        self.pid_frame = PIDControllerFrame(self.app, self.serial_manager, self, corner_radius=10)
        self.pid_frame.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)

    ## ALTERAÇÃO DIDÁTICA v3.2: Obtenção do modo de trajetória ##
    # O método agora precisa de aceder ao menu dentro do pid_frame para obter o valor selecionado.
    def _executar_trajetoria(self):
        modo = self.pid_frame.ctrl_menu.get()
        self.status_bar.show_message(f"Função 'Executar Trajetória: {modo}' ainda não implementada.")
    def _parar_trajetoria(self):
        self.status_bar.show_message("Função 'Parar Movimento' ainda não implementada.")

    def _on_camera_change(self, new_index):
        if self.video_handler: self.video_handler.change_camera(new_index)
    def _salvar_config(self):
        config = {"hsv": self.hsv_frame.save_hsv(), "pid": self.pid_frame.get_pid_config()}; filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=CONFIG_FILE_TYPES)
        if not filepath: return
        try:
            with open(filepath, 'w', encoding='utf-8') as f: json.dump(config, f, indent=4)
            self.status_bar.show_message(f"Configuração salva em {filepath.split('/')[-1]}")
        except Exception as e: self.status_bar.show_message(f"Erro ao salvar: {e}", is_error=True)
    def _carregar_config(self):
        filepath = filedialog.askopenfilename(filetypes=CONFIG_FILE_TYPES)
        if not filepath: return
        try:
            with open(filepath, 'r', encoding='utf-8') as f: config = json.load(f)
            self.hsv_frame.load_hsv(config.get("hsv", {})); self.pid_frame.load_pid_config(config.get("pid", {}))
            self.status_bar.show_message("Configuração carregada com sucesso.");
            if self.video_handler: self.video_handler.update_once()
        except Exception as e: self.status_bar.show_message(f"Erro ao carregar: {e}", is_error=True)
    def _on_close(self):
        if self.video_handler: self.video_handler.stop()
        self.serial_manager.disconnect(); self.app.destroy()
    def run(self): self.app.mainloop()

# -----------------------------------------------------------------------------
# Ponto de Entrada do Script
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    app = OpenBalanceApp()
    app.run()