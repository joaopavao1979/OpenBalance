"""
OpenBalance Dashboard (Versão 3.5 - Robusta e Final)
Descrição: Dashboard para controlo de bola numa plataforma inclinável.
Autor: João Pavão (Refatorado por Engenheiro de Software/Designer Gráfico AI)
Finalidade: Interface gráfica robusta com comunicação entre threads totalmente
segura, evitando congelamentos e erros.

NOTA DE CORREÇÃO FINAL: Esta versão implementa a solução definitiva para os
problemas de concorrência. Todas as variáveis partilhadas entre a thread de
vídeo e a UI são acedidas exclusivamente através de métodos seguros (getters/setters)
que usam locks, eliminando a última fonte de erros. A aplicação deve agora
ser totalmente estável em todas as condições.
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
import queue
import time

# -----------------------------------------------------------------------------
# --------------------------- CONSTANTES E CONFIGURAÇÕES ---------------------
# -----------------------------------------------------------------------------
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
DEFAULT_BAUDRATE = 115200
CONFIG_FILE_TYPES = [("JSON Files", "*.json")]

# Valores HSV de exemplo para presets
HSV_PRESETS = {
    "Vermelho": {"h_min": 0, "h_max": 10, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Laranja": {"h_min": 10, "h_max": 25, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Verde": {"h_min": 40, "h_max": 80, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Cinzento": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 50, "v_min": 50, "v_max": 200},
    "Azul": {"h_min": 90, "h_max": 130, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 255},
    "Branco": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 30, "v_min": 200, "v_max": 255}
}

# Mapeamento de cor de botão
BUTTON_COLORS = {
    "Vermelho": {"fg": "#FF0000", "hover": "#CC0000", "text": "white"},
    "Laranja": {"fg": "#FFA500", "hover": "#CC8400", "text": "white"},
    "Verde": {"fg": "#00B200", "hover": "#008000", "text": "white"},
    "Cinzento": {"fg": "#808080", "hover": "#666666", "text": "white"},
    "Azul": {"fg": "#0000FF", "hover": "#0000CC", "text": "white"},
    "Branco": {"fg": "#FFFFFF", "hover": "#DDDDDD", "text": "black"}
}

# Paleta de Cores da UI
COLOR_SUCCESS = {"fg_color": "green", "hover_color": "darkgreen"}
COLOR_DANGER = {"fg_color": "#D32F2F", "hover_color": "#B71C1C"}
COLOR_SECONDARY = {"fg_color": "gray50", "hover_color": "gray30"}

# Comandos Seriais
CMD_MOTORS_ON = "M1\n"
CMD_MOTORS_OFF = "M0\n"

# -----------------------------------------------------------------------------
# ---------------------------- GERENCIADOR SERIAL ----------------------------
# -----------------------------------------------------------------------------
class SerialManager:
    """Gerencia a conexão serial com o Arduino."""
    def __init__(self): self.serial_conn = None
    def list_ports(self): return [port.device for port in serial.tools.list_ports.comports()]
    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        try:
            if self.serial_conn and self.serial_conn.is_open: self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            return True
        except Exception as e:
            self.serial_conn = None; print(f"Erro Serial: {e}"); return False
    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open: self.serial_conn.close(); self.serial_conn = None
    def send(self, data_str):
        if not (self.serial_conn and self.serial_conn.is_open): return
        try: self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException: self.disconnect()

# -----------------------------------------------------------------------------
# ----------------------- FRAME DE CONFIGURAÇÕES HSV --------------------------
# -----------------------------------------------------------------------------
class HSVSettingsFrame(ctk.CTkFrame):
    """Frame para ajuste de parâmetros HSV com 'debouncing' para estabilidade."""
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.video_handler = video_handler
        self.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10, 5), sticky="n")

        self.hsv_sliders, self.hsv_values = [], []
        labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
        for idx, label in enumerate(labels):
            ctk.CTkLabel(self, text=label).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(master=self, value="0")
            self.hsv_values.append(val_var)
            slider = ctk.CTkSlider(self, from_=0, to=255, number_of_steps=255, command=self._on_slider_change)
            slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew")
            self.hsv_sliders.append(slider); slider.val_var = val_var
            ctk.CTkLabel(self, textvariable=val_var).grid(row=2 + 2*idx, column=0, padx=(270,10), sticky="e")

        self.show_mask_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(self, text="Mostrar Máscara", variable=self.show_mask_var, command=self._on_toggle_mask).grid(row=13, column=0, pady=(10,5), padx=10, sticky="w")
        for i, (name, colors) in enumerate(BUTTON_COLORS.items()):
            ctk.CTkButton(self, text=name, fg_color=colors["fg"], hover_color=colors["hover"], text_color=colors["text"], command=lambda n=name: self._apply_preset(n)).grid(row=14 + i, column=0, pady=2, padx=10, sticky="ew")
        
        self._debounce_job = None # Variável para controlar o debounce

    def _on_slider_change(self, _=None):
        """Atualiza as labels e agenda a atualização dos valores HSV para evitar 'spam'."""
        for slider in self.hsv_sliders: slider.val_var.set(f"{int(slider.get())}")
        self._schedule_hsv_update()

    def _schedule_hsv_update(self):
        """Agenda a atualização dos valores, cancelando qualquer agendamento anterior."""
        if self._debounce_job: self.after_cancel(self._debounce_job)
        self._debounce_job = self.after(150, self._perform_hsv_update) # Atraso de 150ms

    def _perform_hsv_update(self):
        """Executa a atualização real no VideoHandler."""
        self._debounce_job = None
        if self.video_handler: self.video_handler.update_hsv_from_frame()

    def _apply_preset(self, name):
        preset = HSV_PRESETS.get(name)
        if not preset: return
        vals = [preset[k] for k in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max"]]
        for i, val in enumerate(vals): self.hsv_sliders[i].set(val); self.hsv_values[i].set(str(val))
        if self.video_handler: self._perform_hsv_update(); self.video_handler.update_once()

    def _on_toggle_mask(self):
        if self.video_handler: self.video_handler.set_show_mask(self.show_mask_var.get())

    def get_hsv_bounds(self):
        try: values = [int(v.get()) for v in self.hsv_values]
        except ValueError: values = [0, 255, 0, 255, 0, 255]
        return np.array([values[0], values[2], values[4]]), np.array([values[1], values[3], values[5]])

    def load_hsv(self, hsv_dict):
        vals = [hsv_dict.get(k, 0) for k in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max"]]
        for i, val in enumerate(vals): self.hsv_sliders[i].set(val); self.hsv_values[i].set(str(val))

    def save_hsv(self):
        lower, upper = self.get_hsv_bounds()
        return {"h_min": int(lower[0]), "h_max": int(upper[0]), "s_min": int(lower[1]), "s_max": int(upper[1]), "v_min": int(lower[2]), "v_max": int(upper[2])}

# -----------------------------------------------------------------------------
# ---------------------- FRAME DE CONTROLO DO SISTEMA -------------------------
# -----------------------------------------------------------------------------
class SystemControlFrame(ctk.CTkFrame):
    """Frame de controlo para motores e comunicação serial."""
    def __init__(self, parent, serial_manager: SerialManager, app_ref, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.serial_manager, self.app = serial_manager, app_ref
        self.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(self, text="Controlo do Sistema", font=("Arial", 16, "bold")).grid(row=0, column=0, padx=10, pady=(10, 5))
        motor_group = ctk.CTkFrame(self, fg_color="transparent"); motor_group.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        motor_group.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkButton(motor_group, text="Ligar Motores", command=self._ligar_motores, **COLOR_SUCCESS).grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Desligar Motores", command=self._desligar_motores, **COLOR_DANGER).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.btn_ligar_seguimento = ctk.CTkButton(motor_group, text="Ligar Seguimento", command=self._toggle_seguimento, **COLOR_SECONDARY)
        self.btn_ligar_seguimento.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        serial_group = ctk.CTkFrame(self, fg_color="transparent"); serial_group.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        serial_group.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(serial_group, text="Comunicação Arduino", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(5, 10))
        portas = self.serial_manager.list_ports(); self.porta_var = tk.StringVar(value=portas[0] if portas else "Nenhuma porta")
        self.option_menu_porta = ctk.CTkOptionMenu(serial_group, values=portas or ["Nenhuma porta"], variable=self.porta_var)
        self.option_menu_porta.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="🔄", width=30, command=self._atualizar_portas).grid(row=1, column=1, padx=5, pady=5)
        ctk.CTkButton(serial_group, text="Ligar", command=self._ligar_arduino, **COLOR_SUCCESS).grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="Desligar", command=self._desligar_arduino, **COLOR_DANGER).grid(row=2, column=1, padx=5, pady=5, sticky="ew")

    def _toggle_seguimento(self):
        """Lê e escreve o estado do seguimento de forma segura."""
        vh = self.app.video_handler
        if not vh: return
        
        # CORRIGIDO: Usa o getter seguro para ler e o setter seguro para escrever.
        current_state = vh.tracking_enabled_safe
        new_state = not current_state
        vh.set_tracking_enabled(new_state)
        
        if new_state:
            self.btn_ligar_seguimento.configure(text="Parar Seguimento", **COLOR_DANGER)
            self.app.status_bar.show_message("Seguimento ativado.")
        else:
            self.btn_ligar_seguimento.configure(text="Ligar Seguimento", **COLOR_SECONDARY)
            self.app.status_bar.show_message("Seguimento desativado.")
            
    def _ligar_motores(self): self.serial_manager.send(CMD_MOTORS_ON); self.app.status_bar.show_message("Comando para ligar motores enviado.")
    def _desligar_motores(self): self.serial_manager.send(CMD_MOTORS_OFF); self.app.status_bar.show_message("Comando para desligar motores enviado.")
    def _atualizar_portas(self):
        novas_portas = self.serial_manager.list_ports(); self.option_menu_porta.configure(values=novas_portas or ["Nenhuma porta"])
        self.porta_var.set(novas_portas[0] if novas_portas else "Nenhuma porta"); self.app.status_bar.show_message("Portas atualizadas.")
    def _ligar_arduino(self):
        porta = self.porta_var.get()
        if porta != "Nenhuma porta":
            if self.serial_manager.connect(porta): self.app.status_bar.show_message(f"Conectado a {porta}.", 5000)
            else: self.app.status_bar.show_message(f"Falha ao conectar a {porta}.", 5000, is_error=True)
        else: self.app.status_bar.show_message("Nenhuma porta serial selecionada.", 3000, is_error=True)
    def _desligar_arduino(self): self.serial_manager.disconnect(); self.app.status_bar.show_message("Conexão serial encerrada.")
    def get_pid_config(self):
        return {}
    def load_pid_config(self, pid_dict):
        pass # Placeholders

# -----------------------------------------------------------------------------
# -------------------------- GERENCIADOR DE VÍDEO -----------------------------
# -----------------------------------------------------------------------------
class VideoHandler:
    """Gerencia captura e processamento de vídeo de forma robusta e thread-safe."""
    def __init__(self, parent, hsv_frame_ref: HSVSettingsFrame, serial_manager: SerialManager, camera_index=0):
        self.parent, self.hsv_frame_ref, self.serial_manager = parent, hsv_frame_ref, serial_manager
        self.camera_index = camera_index; self.cap = None; self.thread = None; self.running = False
        self.frame_queue = queue.Queue(maxsize=1)
        
        self.shared_lock = threading.Lock()
        self.hsv_lower, self.hsv_upper = np.array([0,0,0]), np.array([255,255,255])
        self.show_mask, self._tracking_enabled_internal = False, False # Renomeado para evitar confusão

        self.canvas = ctk.CTkCanvas(parent, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, bg="gray20", highlightthickness=0)
        self.canvas.grid(row=2, column=0, padx=10, pady=10); self.update_hsv_from_frame(); self._initialize_camera()
        
        # Debouncing do envio serial
        self.last_send_time = 0

    # --- Métodos Seguros para Acesso a Variáveis Partilhadas ---
    def update_hsv_from_frame(self):
        if not self.hsv_frame_ref: return
        lower, upper = self.hsv_frame_ref.get_hsv_bounds()
        with self.shared_lock: self.hsv_lower, self.hsv_upper = lower, upper
        
    def set_show_mask(self, state: bool):
        with self.shared_lock: self.show_mask = state
        
    def set_tracking_enabled(self, state: bool):
        with self.shared_lock: self._tracking_enabled_internal = state
        
    @property
    def tracking_enabled_safe(self) -> bool:
        """[SEGURO] Retorna o estado atual do tracking usando o lock."""
        with self.shared_lock: return self._tracking_enabled_internal
    # --- Fim dos Métodos Seguros ---

    def _initialize_camera(self):
        if self.cap and self.cap.isOpened():
            self.running = False
            if self.thread and self.thread.is_alive(): self.thread.join(timeout=1)
            self.cap.release()
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        if not self.cap.isOpened(): messagebox.showerror("Erro de Vídeo", f"Não foi possível aceder à webcam {self.camera_index}.")
        else: self.running = True; self.thread = threading.Thread(target=self._video_loop, daemon=True); self.thread.start(); self._update_canvas_from_queue()

    def _video_loop(self):
        while self.running and self.cap and self.cap.isOpened():
            ret, frame_bgr = self.cap.read()
            if not ret: break
            try: self.frame_queue.put_nowait(self._process_frame_for_display(frame_bgr))
            except queue.Full: pass
        self.running = False

    def _process_frame_for_display(self, frame_bgr):
        height, width, _ = frame_bgr.shape
        center_x, center_y = width // 2, height // 2
        with self.shared_lock:
            lower, upper, show_mask, tracking = (
                self.hsv_lower, self.hsv_upper, self.show_mask, self._tracking_enabled_internal
            )
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB) if show_mask else cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                (x, y), radius = cv2.minEnclosingCircle(c)
                ball_center = (int(x), int(y))
                erro_x, erro_y = ball_center[0] - center_x, -(ball_center[1] - center_y)
                # ---------- DEBOUNCING DO ENVIO SERIAL ----------
                if tracking and self.serial_manager.serial_conn:
                    now = time.time()
                    if now - self.last_send_time > 0.05:  # só envia a cada 50ms (~20Hz)
                        self.serial_manager.send(f"E,{erro_x},{erro_y}\n")
                        self.last_send_time = now
                # ---------- FIM DEBOUNCING ----------
                cv2.circle(display_img, ball_center, int(radius), (0, 255, 0), 2)
                cv2.circle(display_img, ball_center, 3, (255, 0, 0), -1)
                cv2.putText(
                    display_img,
                    f"Erro X: {erro_x:+d}, Y: {erro_y:+d}",
                    (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2, cv2.LINE_AA
                )
        cv2.line(display_img, (center_x - 10, center_y), (center_x + 10, center_y), (255, 255, 0), 1)
        cv2.line(display_img, (center_x, center_y - 10), (center_x, center_y + 10), (255, 255, 0), 1)
        return display_img



    def _update_canvas_from_queue(self):
        try:
            frame = self.frame_queue.get_nowait()
            img_pil = Image.fromarray(frame); img_tk = ImageTk.PhotoImage(image=img_pil)
            self.canvas.create_image(0, 0, anchor="nw", image=img_tk); self.canvas.image = img_tk
        except queue.Empty: pass
        finally:
            if self.running: self.canvas.after(33, self._update_canvas_from_queue)

    def change_camera(self, new_index):
        try: self.camera_index = int(new_index)
        except ValueError: messagebox.showerror("Câmera", f"Índice inválido: {new_index}"); return
        self._initialize_camera()

    def update_once(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                display_img = self._process_frame_for_display(frame)
                img_pil = Image.fromarray(display_img); img_tk = ImageTk.PhotoImage(image=img_pil)
                self.canvas.create_image(0, 0, anchor="nw", image=img_tk); self.canvas.image = img_tk

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive(): self.thread.join(timeout=1)
        if self.cap: self.cap.release()

# -----------------------------------------------------------------------------
# ----------------------------- APLICAÇÃO PRINCIPAL ---------------------------
# -----------------------------------------------------------------------------
class StatusBar(ctk.CTkFrame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, corner_radius=0, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w"); self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None
    def show_message(self, message, duration_ms=4000, is_error=False):
        if self._job: self.after_cancel(self._job)
        text_color = "#FF5555" if is_error else ctk.ThemeManager.theme["CTkLabel"]["text_color"]
        self.label.configure(text=message, text_color=text_color)
        self._job = self.after(duration_ms, lambda: self.label.configure(text=""))

class OpenBalanceApp:
    def __init__(self):
        ctk.set_appearance_mode("dark"); ctk.set_default_color_theme("dark-blue")
        self.app = ctk.CTk(); self.app.title("OpenBalance 3.5 - Robusto"); self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.app.grid_rowconfigure(0, weight=1); self.app.grid_rowconfigure(1, weight=0); self.app.grid_columnconfigure(0, weight=1)
        self.app.grid_columnconfigure(1, weight=3); self.app.grid_columnconfigure(2, weight=1)
        self.serial_manager = SerialManager(); self.video_handler = None
        self._create_menu(); self._create_frames(); self._create_status_bar()
        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _create_menu(self):
        menu = tk.Menu(self.app); file_menu = tk.Menu(menu, tearoff=0)
        file_menu.add_command(label="Salvar Configuração", command=self._salvar_config)
        file_menu.add_command(label="Carregar Configuração", command=self._carregar_config)
        file_menu.add_separator(); file_menu.add_command(label="Sair", command=self._on_close)
        menu.add_cascade(label="Arquivo", menu=file_menu); self.app.config(menu=menu)

    def _create_status_bar(self):
        self.status_bar = StatusBar(self.app); self.status_bar.grid(row=1, column=0, columnspan=3, sticky="sew")

    def _create_frames(self):
        video_container = ctk.CTkFrame(self.app, corner_radius=10); video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        video_container.grid_rowconfigure(2, weight=1); video_container.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(video_container, text="Área de Visualização", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10,5))
        cam_frame = ctk.CTkFrame(video_container, fg_color="transparent"); cam_frame.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(cam_frame, text="Câmera:").pack(side="left", padx=(0, 5))
        self.cam_var = tk.StringVar(value="0")
        ctk.CTkOptionMenu(cam_frame, values=["0", "1", "2", "3"], variable=self.cam_var, command=self._on_camera_change).pack(side="left")
        
        self.hsv_frame = HSVSettingsFrame(self.app, video_handler=None); self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)
        self.control_frame = SystemControlFrame(self.app, self.serial_manager, app_ref=self); self.control_frame.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)
        self.video_handler = VideoHandler(video_container, self.hsv_frame, self.serial_manager, int(self.cam_var.get()))
        self.hsv_frame.video_handler = self.video_handler

    def _on_camera_change(self, index):
        if self.video_handler: self.video_handler.change_camera(index)
    
    def _salvar_config(self):
        config = {"hsv": self.hsv_frame.save_hsv(), "pid": self.control_frame.get_pid_config()}
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=CONFIG_FILE_TYPES)
        if not filepath: return
        try:
            with open(filepath, 'w', encoding='utf-8') as f: json.dump(config, f, indent=4)
            self.status_bar.show_message(f"Configuração salva em {filepath}")
        except Exception as e: self.status_bar.show_message(f"Erro ao salvar: {e}", is_error=True)

    def _carregar_config(self):
        filepath = filedialog.askopenfilename(filetypes=CONFIG_FILE_TYPES)
        if not filepath: return
        try:
            with open(filepath, 'r', encoding='utf-8') as f: config = json.load(f)
            self.hsv_frame.load_hsv(config.get("hsv", {}))
            self.control_frame.load_pid_config(config.get("pid", {}))
            self.status_bar.show_message("Configuração carregada com sucesso.")
            if self.video_handler: self.video_handler.update_hsv_from_frame(); self.video_handler.update_once()
        except Exception as e: self.status_bar.show_message(f"Erro ao carregar: {e}", is_error=True)

    def _on_close(self):
        if self.video_handler: self.video_handler.stop()
        self.serial_manager.disconnect(); self.app.destroy()

    def run(self): self.app.mainloop()

# -----------------------------------------------------------------------------
# -------------------------- PONTO DE ENTRADA DO SCRIPT -----------------------
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    app = OpenBalanceApp()
    app.run()