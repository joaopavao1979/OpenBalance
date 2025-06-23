"""
===============================================================================
Projeto: OpenBalance – Dashboard Python com Visão e Controlo PID

Ficheiro: Dashboard_OpenBalance_V1.1_Trajectory.py

Descrição: Interface gráfica para controlo em tempo real de uma plataforma
           equilibradora de bola, com deteção por OpenCV e controlo PID via Arduino.

Autor: João Pavão

Data: 22/06/2025

Disciplina: Laboratório de Aplicações em Robótica e Aprendizagem (PRIA)
Instituição: Universidade dos Açores
Licença: MIT License

===============================================================================
Notas da versão 1.1:
- Integra uma janela de gráficos com duas visualizações paralelas:
  1. Gráfico de Erros (X/Y vs. Tempo) – útil para análise e afinação de PID.
  2. Gráfico de Trajetória 2D (Y vs. X) – representação espacial da bola.
- Interface robusta com ajuste de HSV, seleção de câmaras, comandos de controlo,
  e visualização clara da posição da bola e do alvo.
- Comunicação com Arduino através de comandos: M1 (motores ON), M0 (OFF), E,x,y (erro).
===============================================================================

# Índice de Estrutura e Comentários do Código
Este índice resume a organização da aplicação `Dashboard_OpenBalance_V1.1_Trajectory.py`,
facilitando a navegação por classes e blocos principais.

Estrutura modular:
================================================================================================
| Secção | Classe / Bloco             | Descrição resumida                                     |
|--------|----------------------------|--------------------------------------------------------|
| 1      | IMPORTAÇÕES                | Bibliotecas padrão e externas (OpenCV, Tkinter, etc.)  |
| 2      | CONSTANTES                 | Dimensões, comandos, presets HSV, cores                |
| 3.1    | SerialManager              | Gestão da comunicação serial                           |
| 3.2    | RealTimePlotFrame          | Gráfico PID (erro X/Y vs. tempo)                       |
| 3.2.1  | TrajectoryPlotFrame        | Gráfico 2D com origem centrada (posição Y vs. X)       |
| 3.2.2  | PlotWindow                 | Janela com ambos os gráficos                           |
| 3.3    | HSVSettingsFrame           | Sliders de HSV com presets interativos                 |
| 3.4    | SystemControlFrame         | Botões de controlo e ligação à porta COM               |
| 3.5    | VideoHandler               | Captura, processamento de vídeo e envio de erros       |
| 3.6    | StatusBar                  | Mensagens temporárias no rodapé                        |
| 4      | OpenBalanceApp             | Classe principal da aplicação                          |
| 5      | if __name__ == "__main__"  | Entrada principal da aplicação (executável)            |
================================================================================================
"""

# =============================================================================
# 1. IMPORTAÇÕES DE BIBLIOTECAS
# =============================================================================
# Importações padrão e de sistema
import json
import threading
import queue
import time
from collections import deque

# Importações de bibliotecas de terceiros (requerem instalação)
import cv2  # OpenCV para processamento de imagem e vídeo
import numpy as np  # NumPy para operações numéricas eficientes, especialmente com arrays
import serial  # PySerial para comunicação com a porta serial (Arduino)
import serial.tools.list_ports  # Ferramenta para listar portas seriais disponíveis
from PIL import Image, ImageTk  # Pillow para conversão entre formatos de imagem (OpenCV -> Tkinter)
import customtkinter as ctk  # Biblioteca para a criação da interface gráfica moderna
import tkinter as tk  # Tkinter para elementos base da GUI e diálogos de ficheiro
from tkinter import filedialog, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg # Ponte entre Matplotlib e Tkinter
import matplotlib.pyplot as plt  # Matplotlib para a criação de gráficos

# =============================================================================
# 2. CONSTANTES E CONFIGURAÇÕES GLOBAIS
# =============================================================================
# --- Dimensões da UI ---
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480

# --- Configurações de Comunicação ---
DEFAULT_BAUDRATE = 115200
CONFIG_FILE_TYPES = [("Ficheiros JSON", "*.json")]

# --- Comandos Seriais para o Arduino ---
CMD_MOTORS_ON = "M1\n"  # Comando para ligar os motores
CMD_MOTORS_OFF = "M0\n" # Comando para desligar os motores

# --- Presets de Deteção de Cor (HSV) ---
HSV_PRESETS = {
    "Vermelho": {"h_min": 0, "h_max": 10, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Laranja": {"h_min": 10, "h_max": 25, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Verde": {"h_min": 40, "h_max": 80, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 255},
    "Cinzento": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 50, "v_min": 50, "v_max": 200},
    "Azul": {"h_min": 90, "h_max": 130, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 255},
    "Branco": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 30, "v_min": 200, "v_max": 255}
}

# --- Estilos dos Botões de Presets ---
BUTTON_COLORS = {
    "Vermelho": {"fg": "#FF0000", "hover": "#CC0000", "text": "white"},
    "Laranja": {"fg": "#FFA500", "hover": "#CC8400", "text": "white"},
    "Verde": {"fg": "#00B200", "hover": "#008000", "text": "white"},
    "Cinzento": {"fg": "#808080", "hover": "#666666", "text": "white"},
    "Azul": {"fg": "#0000FF", "hover": "#0000CC", "text": "white"},
    "Branco": {"fg": "#FFFFFF", "hover": "#DDDDDD", "text": "black"}
}

# --- Paleta de Cores da UI ---
COLOR_SUCCESS = {"fg_color": "green", "hover_color": "darkgreen"}
COLOR_DANGER = {"fg_color": "#D32F2F", "hover_color": "#B71C1C"}
COLOR_SECONDARY = {"fg_color": "gray50", "hover_color": "gray30"}
COLOR_INFO = {"fg_color": "#0288D1", "hover_color": "#01579B"}


# =============================================================================
# 3. CLASSES MODULARES DA APLICAÇÃO
# =============================================================================

# -----------------------------------------------------------------------------
# 3.1. MÓDULO DE GESTÃO SERIAL
# -----------------------------------------------------------------------------
class SerialManager:
    def __init__(self):
        self.serial_conn = None
    def list_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]
    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            return True
        except Exception as e:
            self.serial_conn = None
            print(f"Erro ao conectar na porta serial: {e}")
            return False
    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.serial_conn = None
    def send(self, data_str):
        if not (self.serial_conn and self.serial_conn.is_open):
            return
        try:
            self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException:
            self.disconnect()

# -----------------------------------------------------------------------------
# 3.2. MÓDULO DE VISUALIZAÇÃO GRÁFICA (GRÁFICO DE ERRO)
# -----------------------------------------------------------------------------
class RealTimePlotFrame(ctk.CTkFrame):
    def __init__(self, parent, max_points=200, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.data_x = deque(maxlen=max_points)
        self.data_y = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)
        self.start_time = time.time()
        self.fig, self.ax = plt.subplots(figsize=(5, 3))
        self.fig.patch.set_facecolor('#2B2B2B')
        self.ax.set_facecolor('#2B2B2B')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')
        [spine.set_color('white') for spine in self.ax.spines.values()]
        self.ax.yaxis.label.set_color('white')
        self.ax.xaxis.label.set_color('white')
        self.ax.title.set_color('white')
        self.line_x, = self.ax.plot([], [], label="Erro X", color='#E53935')
        self.line_y, = self.ax.plot([], [], label="Erro Y", color='#1E88E5')
        legend = self.ax.legend(loc="upper right", facecolor='#333333', edgecolor='white')
        for text in legend.get_texts(): text.set_color("white")
        self.ax.set_ylim(-350, 350)
        self.ax.set_xlim(0, 10)
        self.ax.set_title("Erro vs. Tempo")
        self.ax.set_xlabel("Tempo (s)")
        self.ax.set_ylabel("Erro (pixels)")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True)
        self._update_plot()
    def add_data_point(self, erro_x, erro_y):
        now = time.time() - self.start_time
        self.timestamps.append(now)
        self.data_x.append(erro_x)
        self.data_y.append(erro_y)
    def _update_plot(self):
        if self.timestamps:
            self.line_x.set_data(self.timestamps, self.data_x)
            self.line_y.set_data(self.timestamps, self.data_y)
            current_time = self.timestamps[-1]
            self.ax.set_xlim(max(0, current_time - 10), current_time + 1)
        self.ax.relim()
        self.ax.autoscale_view(True, True, True)
        self.ax.set_ylim(-350, 350)
        self.canvas.draw()
        self.after(100, self._update_plot)


# -----------------------------------------------------------------------------
# 3.2.1. ### NOVO ### MÓDULO DE GRÁFICO DE TRAJETÓRIA
# -----------------------------------------------------------------------------
class TrajectoryPlotFrame(ctk.CTkFrame):
    """
    Frame que contém o gráfico Matplotlib para visualização da trajetória 2D da bola.
    """
    def __init__(self, parent, max_points=500, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)

        self.trajectory_x = deque(maxlen=max_points)
        self.trajectory_y = deque(maxlen=max_points)

        self.fig, self.ax = plt.subplots(figsize=(5, 4))
        self.fig.patch.set_facecolor('#2B2B2B')
        self.ax.set_facecolor('#2B2B2B')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')
        [spine.set_color('white') for spine in self.ax.spines.values()]
        self.ax.yaxis.label.set_color('white')
        self.ax.xaxis.label.set_color('white')
        self.ax.title.set_color('white')

        # Cria os elementos do gráfico: trajetória, posição atual e alvo
        self.trajectory_line, = self.ax.plot([], [], color='yellow', linewidth=1, label='Trajetória')
        self.current_pos_marker, = self.ax.plot([], [], 'o', markersize=12, color='lime', label='Pos. Atual')
        self.target_pos_marker, = self.ax.plot([], [], 'o', markersize=12, color='blue', markeredgewidth=2, label='Alvo')
        
        # Cruz branca no centro da imagem (referência visual)
        self.center_marker, = self.ax.plot(
            [VIDEO_WIDTH // 2],
            [VIDEO_HEIGHT // 2],
            marker='+',
            color='red',
            markersize=12,
            linewidth=10,
            label='Centro'
        )

        # Configura limites e eixos
        self.ax.set_title("Trajetória da Bola (Vista de Cima)")
        self.ax.set_xlabel("Eixo X (pixels)")
        self.ax.set_ylabel("Eixo Y (pixels)")
        self.ax.set_xlim(0, VIDEO_WIDTH)
        self.ax.set_ylim(VIDEO_HEIGHT, 0)
        self.ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        self.ax.set_aspect('equal', adjustable='box') # Garante que o gráfico seja quadrado

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True)

    def update_plot(self, ball_pos, target_pos):
        """Atualiza o gráfico com as novas posições da bola e do alvo."""
        if ball_pos:
            self.trajectory_x.append(ball_pos[0])
            self.trajectory_y.append(ball_pos[1])
            self.current_pos_marker.set_data([ball_pos[0]], [ball_pos[1]])

        else:
            # Se a bola não for detetada, esconde o marcador
            self.current_pos_marker.set_data([], [])

        self.trajectory_line.set_data(self.trajectory_x, self.trajectory_y)
        self.target_pos_marker.set_data([target_pos[0]], [target_pos[1]])
        
        # Usa draw_idle para uma atualização mais eficiente
        self.canvas.draw_idle()


# -----------------------------------------------------------------------------
# 3.2.2. ### ALTERADO ### JANELA DE GRÁFICOS COMBINADA
# -----------------------------------------------------------------------------
class PlotWindow(ctk.CTkToplevel):
    """
    Janela pop-up que combina o gráfico de erro e o gráfico de trajetória.
    """
    def __init__(self, master, *args, **kwargs):
        super().__init__(master, *args, **kwargs)
        self.title("Análise Gráfica em Tempo Real")
        self.geometry("1100x550") # Tamanho ajustado para dois gráficos
        
        # Grelha com 2 colunas, ambas com o mesmo peso
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # Gráfico de Erro vs. Tempo (esquerda)
        self.plot_frame_error = RealTimePlotFrame(self)
        self.plot_frame_error.grid(row=0, column=0, padx=(10, 5), pady=10, sticky="nsew")

        # Gráfico de Trajetória (direita)
        self.plot_frame_trajectory = TrajectoryPlotFrame(self)
        self.plot_frame_trajectory.grid(row=0, column=1, padx=(5, 10), pady=10, sticky="nsew")

        self.protocol("WM_DELETE_WINDOW", self.hide_window)

    def hide_window(self):
        self.withdraw()


# -----------------------------------------------------------------------------
# 3.3. MÓDULO DE INTERFACE: PAINEL DE CONFIGURAÇÕES HSV
# -----------------------------------------------------------------------------
class HSVSettingsFrame(ctk.CTkFrame):
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.video_handler = video_handler
        self.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 20, "bold")).grid(row=0, column=0, pady=(10, 5), sticky="n")
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
        self._debounce_job = None
    def _on_slider_change(self, _=None):
        for slider in self.hsv_sliders: slider.val_var.set(f"{int(slider.get())}")
        self._schedule_hsv_update()
    def _schedule_hsv_update(self):
        if self._debounce_job: self.after_cancel(self._debounce_job)
        self._debounce_job = self.after(150, self._perform_hsv_update)
    def _perform_hsv_update(self):
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
# 3.4. MÓDULO DE INTERFACE: PAINEL DE CONTROLO DO SISTEMA
# -----------------------------------------------------------------------------
class SystemControlFrame(ctk.CTkFrame):
    def __init__(self, parent, serial_manager: SerialManager, app_ref, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.serial_manager, self.app = serial_manager, app_ref
        self.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(self, text="Controlo do Sistema", font=("Arial", 20, "bold")).grid(row=0, column=0, padx=10, pady=(10, 5))
        motor_group = ctk.CTkFrame(self, fg_color="transparent"); motor_group.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        motor_group.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkButton(motor_group, text="Ligar Motores", command=self._ligar_motores, **COLOR_SUCCESS).grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Desligar Motores", command=self._desligar_motores, **COLOR_DANGER).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.btn_ligar_seguimento = ctk.CTkButton(motor_group, text="Ligar Seguimento", command=self._toggle_seguimento, **COLOR_SECONDARY)
        self.btn_ligar_seguimento.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Reset Centro", command=self._reset_centro_manual, fg_color="#AA00AA", hover_color="#880088").grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Ver Gráfico", command=self.app._show_plot_window, **COLOR_INFO).grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        serial_group = ctk.CTkFrame(self, fg_color="transparent"); serial_group.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        serial_group.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(serial_group, text="Comunicação Arduino", font=("Arial", 18, "bold")).grid(row=0, column=0, columnspan=2, pady=(5, 10))
        portas = self.serial_manager.list_ports(); self.porta_var = tk.StringVar(value=portas[0] if portas else "Nenhuma porta")
        self.option_menu_porta = ctk.CTkOptionMenu(serial_group, values=portas or ["Nenhuma porta"], variable=self.porta_var)
        self.option_menu_porta.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="🔄", width=30, command=self._atualizar_portas).grid(row=1, column=1, padx=5, pady=5)
        ctk.CTkButton(serial_group, text="Ligar", command=self._ligar_arduino, **COLOR_SUCCESS).grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="Desligar", command=self._desligar_arduino, **COLOR_DANGER).grid(row=2, column=1, padx=5, pady=5, sticky="ew")
    def _toggle_seguimento(self):
        vh = self.app.video_handler
        if not vh: return
        new_state = not vh.tracking_enabled_safe
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
    def _reset_centro_manual(self):
        if self.app.video_handler:
            self.app.video_handler.reset_center()
            self.app.status_bar.show_message("Centro redefinido para o centro da imagem.")
    def get_pid_config(self): return {}
    def load_pid_config(self, pid_dict): pass

# -----------------------------------------------------------------------------
# 3.5. ### ALTERADO ### MÓDULO DE VÍDEO E PROCESSAMENTO DE IMAGEM
# -----------------------------------------------------------------------------
class VideoHandler:
    def __init__(self, parent, hsv_frame_ref: HSVSettingsFrame, serial_manager: SerialManager, error_queue: queue.Queue, camera_index=0):
        self.parent, self.hsv_frame_ref, self.serial_manager, self.error_queue = parent, hsv_frame_ref, serial_manager, error_queue
        self.camera_index = camera_index; self.cap = None; self.thread = None; self.running = False
        self.frame_queue = queue.Queue(maxsize=1)
        self.center_x, self.center_y = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2
        self.shared_lock = threading.Lock()
        self.hsv_lower, self.hsv_upper = np.array([0,0,0]), np.array([255,255,255])
        self.show_mask = False
        self._tracking_enabled_internal = False
        self.last_send_time = 0
        self.canvas = ctk.CTkCanvas(parent, width=768, height=576, bg="gray20", highlightthickness=0)
        self.canvas.grid(row=2, column=0, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self._on_mouse_click)
        self.update_hsv_from_frame()
        self._initialize_camera()
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
        with self.shared_lock: return self._tracking_enabled_internal
    def reset_center(self):
        with self.shared_lock: self.center_x, self.center_y = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2
    def _on_mouse_click(self, event):
        with self.shared_lock: self.center_x, self.center_y = event.x, event.y
    def _initialize_camera(self):
        if self.cap and self.cap.isOpened():
            self.running = False
            if self.thread and self.thread.is_alive(): self.thread.join(timeout=1)
            self.cap.release()
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        if not self.cap.isOpened():
            messagebox.showerror("Erro de Vídeo", f"Não foi possível aceder à webcam {self.camera_index}.")
        else:
            self.running = True
            self.thread = threading.Thread(target=self._video_loop, daemon=True)
            self.thread.start()
            self._update_canvas_from_queue()
    def _video_loop(self):
        while self.running and self.cap and self.cap.isOpened():
            ret, frame_bgr = self.cap.read()
            if not ret: break
            try:
                self.frame_queue.put_nowait(self._process_frame_for_display(frame_bgr))
            except queue.Full:
                pass
        self.running = False

    def _process_frame_for_display(self, frame_bgr):
        height, width, _ = frame_bgr.shape
        with self.shared_lock:
            center_x, center_y = self.center_x, self.center_y
            lower, upper, show_mask, tracking = self.hsv_lower, self.hsv_upper, self.show_mask, self._tracking_enabled_internal
        
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB) if show_mask else cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        erro_x, erro_y = 0, 0
        ball_pos = None # ### NOVO ###: Inicializa a posição da bola como Nula

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                (x, y), radius = cv2.minEnclosingCircle(c)
                ball_pos = (int(x), int(y)) # ### NOVO ###: Guarda a posição da bola
                erro_x, erro_y = ball_pos[0] - center_x, -(ball_pos[1] - center_y)

                if tracking and self.serial_manager.serial_conn:
                    now = time.time()
                    if now - self.last_send_time > 0.05:
                        self.serial_manager.send(f"E,{erro_x},{erro_y}\n")
                        self.last_send_time = now

                cv2.circle(display_img, ball_pos, int(radius), (0, 255, 0), 2)
                cv2.circle(display_img, ball_pos, 3, (255, 0, 0), -1)
                cv2.putText(display_img, f"Erro X: {erro_x:+d}, Y: {erro_y:+d}", (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        
        # ### ALTERADO ###: Envia um pacote de dados mais completo para a queue
        target_pos = (center_x, center_y)
        try:
            # Pacote: (erro_x, erro_y, pos_bola, pos_alvo)
            self.error_queue.put_nowait((erro_x, erro_y, ball_pos, target_pos))
        except queue.Full:
            pass

        cv2.line(display_img, (width//2 - 10, height//2), (width//2 + 10, height//2), (255, 0, 0), 4)
        cv2.line(display_img, (width//2, height//2 - 10), (width//2, height//2 + 10), (255, 0, 0), 4)
        cv2.circle(display_img, (center_x, center_y), 5, (0, 0, 255), -1)
        return display_img

    def _update_canvas_from_queue(self):
        try:
            frame = self.frame_queue.get_nowait()
            img_pil = Image.fromarray(frame).resize((768, 576), Image.BICUBIC); img_tk = ImageTk.PhotoImage(image=img_pil)
            self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
            self.canvas.image = img_tk
        except queue.Empty:
            pass
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
# 3.6. MÓDULO DE INTERFACE: BARRA DE STATUS
# -----------------------------------------------------------------------------
class StatusBar(ctk.CTkFrame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, corner_radius=0, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w", font=("Arial", 16)); self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None
    def show_message(self, message, duration_ms=5000, is_error=False):
        if self._job: self.after_cancel(self._job)
        text_color = "#FF5555" if is_error else "#FFD700"
        self.label.configure(text=message, text_color=text_color)
        self._job = self.after(duration_ms, lambda: self.label.configure(text=""))

# =============================================================================
# 4. CLASSE PRINCIPAL DA APLICAÇÃO
# =============================================================================
class OpenBalanceApp:
    def __init__(self):
        ctk.set_appearance_mode("dark"); ctk.set_default_color_theme("dark-blue")
        self.app = ctk.CTk(); self.app.title("OpenBalance Dashboard"); self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.app.grid_rowconfigure(0, weight=1); self.app.grid_rowconfigure(1, weight=0); self.app.grid_columnconfigure(0, weight=1)
        self.app.grid_columnconfigure(1, weight=3); self.app.grid_columnconfigure(2, weight=1)
        self.serial_manager = SerialManager(); self.video_handler = None
        self.plot_window = None
        self.error_queue = queue.Queue()
        self._create_menu(); self._create_frames(); self._create_status_bar()
        self._process_error_queue() 
        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _create_menu(self):
        menu = tk.Menu(self.app)
        file_menu = tk.Menu(menu, tearoff=0)
        file_menu.add_command(label="Salvar Configuração", command=self._salvar_config)
        file_menu.add_command(label="Carregar Configuração", command=self._carregar_config)
        file_menu.add_separator()
        file_menu.add_command(label="Sair", command=self._on_close)
        menu.add_cascade(label="Arquivo", menu=file_menu)
        ajuda_menu = tk.Menu(menu, tearoff=0)
        ajuda_menu.add_command(label="Sobre o Projeto", command=self._mostrar_sobre)
        ajuda_menu.add_command(label="Manual de Utilização", command=self._mostrar_manual)
        ajuda_menu.add_command(label="Créditos", command=self._mostrar_creditos)
        menu.add_cascade(label="ℹ️ Ajuda", menu=ajuda_menu)
        self.app.config(menu=menu)

    def _create_status_bar(self):
        self.status_bar = StatusBar(self.app); self.status_bar.grid(row=1, column=0, columnspan=3, sticky="sew")

    def _create_frames(self):
        video_container = ctk.CTkFrame(self.app, corner_radius=10); video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        video_container.grid_rowconfigure(2, weight=1); video_container.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(video_container, text="Área de Visualização", font=("Arial", 22, "bold")).grid(row=0, column=0, pady=(10,5))
        cam_frame = ctk.CTkFrame(video_container, fg_color="transparent"); cam_frame.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(cam_frame, text="Câmera:").pack(side="left", padx=(0, 5))
        self.cam_var = tk.StringVar(value="0")
        ctk.CTkOptionMenu(cam_frame, values=["0", "1", "2", "3"], variable=self.cam_var, command=self._on_camera_change).pack(side="left")
        self.hsv_frame = HSVSettingsFrame(self.app, video_handler=None); self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)
        self.control_frame = SystemControlFrame(self.app, self.serial_manager, app_ref=self); self.control_frame.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)
        self.video_handler = VideoHandler(video_container, self.hsv_frame, self.serial_manager, self.error_queue, int(self.cam_var.get()))
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

    def _mostrar_sobre(self):
        messagebox.showinfo("Sobre o Projeto", "OpenBalance Dashboard\nVersão 1.1\nControlo PID com visão computacional.\nJoão Pavão – Universidade dos Açores")

    def _mostrar_manual(self):
        messagebox.showinfo("Manual de Utilização", "1. Ligue o Arduino e selecione a porta COM\n2. Clique em 'Ligar Motores'\n3. Ajuste os parâmetros HSV\n4. Clique em 'Ligar Seguimento' para iniciar o controlo\n5. Use 'Ver Gráfico' para análise de erros e trajetória.")

    def _mostrar_creditos(self):
        messagebox.showinfo("Créditos", "Desenvolvido por João Pavão\nSupervisão: PRIA - Universidade dos Açores\nLicença: MIT License")

    def _show_plot_window(self):
        if self.plot_window is None:
            self.plot_window = PlotWindow(self.app)
            self.plot_window.withdraw()
        if not self.plot_window.winfo_viewable():
             self.plot_window.deiconify()
        self.plot_window.lift()

    # ### ALTERADO ###: Processa a queue com os novos dados de posição
    def _process_error_queue(self):
        try:
            while not self.error_queue.empty():
                erro_x, erro_y, ball_pos, target_pos = self.error_queue.get_nowait()
                if self.plot_window and self.plot_window.winfo_viewable():
                    # Envia os dados para os gráficos correspondentes
                    self.plot_window.plot_frame_error.add_data_point(erro_x, erro_y)
                    self.plot_window.plot_frame_trajectory.update_plot(ball_pos, target_pos)
        except queue.Empty:
            pass
        finally:
            self.app.after(100, self._process_error_queue)

    def _on_close(self):
        if self.video_handler: self.video_handler.stop()
        self.serial_manager.disconnect()
        self.app.destroy()

    def run(self):
        self.app.mainloop()

# =============================================================================
# 5. PONTO DE ENTRADA DO SCRIPT
# =============================================================================
if __name__ == "__main__":
    app = OpenBalanceApp()
    app.run()