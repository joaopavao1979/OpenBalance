```python
"""
OpenBalance Dashboard (Versão 3.0 - Atualizada)
Descrição: Dashboard para controlo de bola numa plataforma inclinável,
utilizando visão computacional (HSV) e PID.
Autor: João Pavão (Refatorado por Engenheiro de Software/Designer Gráfico AI)
Finalidade: Interface gráfica com UI/UX melhorada para ajuste de parâmetros HSV,
controlo PID, visualização de vídeo, presets de cor, comunicação serial robusta,
e gestão de estado explícita.
Uso: Código aberto, disponível livremente sob licença MIT.
Atualizações:
- Taxa de baud ajustada para 115200.
- Mapeamento de erros de pixels para pulsos.
- Sincronização de canvas com lock e after.
- Limitação de envio serial a 20 Hz.
- Resolução de vídeo reduzida para 320x240.
- Tratamento robusto de falhas seriais.
- Adição de logging e deadband.
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
import time
import logging
from threading import Lock
from collections import deque

# Configuração de logging para depuração
logging.basicConfig(
    filename='openbalance.log',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# -----------------------------------------------------------------------------
# --------------------------- CONSTANTES E CONFIGURAÇÕES ---------------------
# -----------------------------------------------------------------------------
WINDOW_WIDTH = 1280  # Largura da janela para acomodar painéis
WINDOW_HEIGHT = 720  # Altura da janela com barra de status
VIDEO_WIDTH = 320    # Reduzido para otimizar desempenho
VIDEO_HEIGHT = 240   # Reduzido para otimizar desempenho
DEFAULT_BAUDRATE = 115200  # Alinhado com o Arduino
CONFIG_FILE_TYPES = [("JSON Files", "*.json")]
CSV_FILE_TYPES = [("CSV Files", "*.csv")]

# Limites de pulso do Arduino para mapeamento de erros
PULSE_MIN_X = 1000
PULSE_CENTER_X = 1250  # Calculado com ANG_EQUIL_X = 95
PULSE_MAX_X = 2000
PULSE_MIN_Y = 1000
PULSE_CENTER_Y = 1333  # Calculado com ANG_EQUIL_Y = 80
PULSE_MAX_Y = 2000

# Deadband para ignorar erros pequenos (em pixels)
DEADBAND_THRESHOLD = 10

# Intervalo mínimo entre envios seriais (segundos)
SERIAL_SEND_INTERVAL = 0.05  # 50 ms (20 Hz)

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

# Paleta de cores para UI
COLOR_SUCCESS = {"fg_color": "green", "hover_color": "darkgreen"}
COLOR_DANGER = {"fg_color": "#D32F2F", "hover_color": "#B71C1C"}
COLOR_NEUTRAL = {"fg_color": "#1E88E5", "hover_color": "#1565C0"}
COLOR_SECONDARY = {"fg_color": "gray50", "hover_color": "gray30"}

# Comandos seriais
CMD_MOTORS_ON = "M1\n"
CMD_MOTORS_OFF = "M0\n"

# -----------------------------------------------------------------------------
# ---------------------------- GERENCIADOR SERIAL ----------------------------
# -----------------------------------------------------------------------------
class SerialManager:
    """
    Gerencia a conexão serial com o Arduino, incluindo listagem de portas,
    conexão/desconexão e envio de dados com tratamento de falhas.
    """
    def __init__(self, parent):
        self.serial_conn = None
        self.parent = parent  # Referência ao OpenBalanceApp para notificações

    def list_ports(self):
        """Retorna lista de portas seriais disponíveis."""
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        """Tenta conectar à porta serial e retorna True em sucesso."""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            logging.info(f"Conectado à porta {port} com baudrate {baudrate}")
            return True
        except Exception as e:
            self.serial_conn = None
            logging.error(f"Erro ao conectar à porta {port}: {e}")
            return False

    def disconnect(self):
        """Desconecta a porta serial, se aberta."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.serial_conn = None
            logging.info("Conexão serial desconectada")

    def send(self, data_str):
        """Envia string pela serial, tratando falhas e desativando seguimento se necessário."""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.parent.status_bar.show_message("Sem conexão serial.", is_error=True)
            logging.warning("Tentativa de envio sem conexão serial ativa")
            if self.parent.video_handler:
                self.parent.video_handler.tracking_enabled = False
                self.parent.pid_frame.btn_ligar_seguimento.configure(
                    text="Ligar Seguimento", **COLOR_SECONDARY
                )
            return
        try:
            self.serial_conn.write(data_str.encode('utf-8'))
            logging.debug(f"Enviado: {data_str.strip()}")
        except serial.SerialException as e:
            self.disconnect()
            self.parent.status_bar.show_message("Conexão serial perdida.", is_error=True)
            logging.error(f"Erro ao enviar dados: {e}")
            if self.parent.video_handler:
                self.parent.video_handler.tracking_enabled = False
                self.parent.pid_frame.btn_ligar_seguimento.configure(
                    text="Ligar Seguimento", **COLOR_SECONDARY
                )

# -----------------------------------------------------------------------------
# ----------------------- FRAME DE CONFIGURAÇÕES HSV --------------------------
# -----------------------------------------------------------------------------
class HSVSettingsFrame(ctk.CTkFrame):
    """
    Frame para ajuste de parâmetros HSV com sliders, presets de cor e preview de máscara.
    Inclui opção para mostrar máscara e salvar/carregar configurações.
    """
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.video_handler = video_handler
        self.grid_columnconfigure(0, weight=1)

        # Título
        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 16, "bold")).grid(
            row=0, column=0, pady=(10, 5), sticky="n"
        )

        # Sliders para H, S, V
        self.hsv_sliders = []
        self.hsv_values = []
        labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
        for idx, label in enumerate(labels):
            ctk.CTkLabel(self, text=label).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(master=self, value="0")
            self.hsv_values.append(val_var)

            slider = ctk.CTkSlider(
                self, from_=0, to=255, number_of_steps=255,
                command=lambda val, var=val_var: var.set(f"{int(val)}")
            )
            slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew")
            self.hsv_sliders.append(slider)

            ctk.CTkLabel(self, textvariable=val_var).grid(
                row=2 + 2*idx, column=0, padx=(270,10), sticky="e"
            )

        # Checkbox para mostrar máscara
        self.show_mask_var = ctk.BooleanVar(master=self, value=False)
        ctk.CTkCheckBox(
            self, text="Mostrar Máscara", variable=self.show_mask_var,
            command=self._on_toggle_mask
        ).grid(row=13, column=0, pady=(10,5), padx=10, sticky="w")

        # Botões de presets de cor
        row_base = 14
        for i, color_name in enumerate(HSV_PRESETS.keys()):
            colors = BUTTON_COLORS[color_name]
            ctk.CTkButton(
                self, text=color_name, fg_color=colors["fg"],
                hover_color=colors["hover"], text_color=colors["text"],
                command=lambda cn=color_name: self._apply_preset(cn)
            ).grid(row=row_base + i, column=0, pady=(2,2), padx=10, sticky="ew")

    def _apply_preset(self, color_name):
        """Aplica valores HSV de um preset selecionado."""
        preset = HSV_PRESETS.get(color_name, None)
        if not preset:
            return
        sliders_vals = [
            preset["h_min"], preset["h_max"], preset["s_min"],
            preset["s_max"], preset["v_min"], preset["v_max"]
        ]
        for i, val in enumerate(sliders_vals):
            self.hsv_sliders[i].set(val)
            self.hsv_values[i].set(str(val))
        if self.video_handler:
            self.video_handler.update_once()
        logging.info(f"Preset HSV aplicado: {color_name}")

    def _on_toggle_mask(self):
        """Ativa/desativa a exibição da máscara."""
        if self.video_handler:
            self.video_handler.show_mask = self.show_mask_var.get()

    def get_hsv_bounds(self):
        """Retorna limites HSV (lower, upper) dos sliders."""
        try:
            h_min, h_max, s_min, s_max, v_min, v_max = (
                int(self.hsv_values[0].get()), int(self.hsv_values[1].get()),
                int(self.hsv_values[2].get()), int(self.hsv_values[3].get()),
                int(self.hsv_values[4].get()), int(self.hsv_values[5].get())
            )
        except ValueError:
            h_min, h_max, s_min, s_max, v_min, v_max = 0, 255, 0, 255, 0, 255
            logging.warning("Valores HSV inválidos, usando padrões")
        return np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max])

    def load_hsv(self, hsv_dict):
        """Carrega configuração HSV de um dicionário."""
        try:
            sliders_vals = [
                hsv_dict.get("h_min", 0), hsv_dict.get("h_max", 255),
                hsv_dict.get("s_min", 0), hsv_dict.get("s_max", 255),
                hsv_dict.get("v_min", 0), hsv_dict.get("v_max", 255)
            ]
            for i, val in enumerate(sliders_vals):
                self.hsv_sliders[i].set(val)
                self.hsv_values[i].set(str(val))
            logging.info("Configuração HSV carregada")
        except Exception as e:
            messagebox.showerror("Erro HSV", f"Falha ao carregar HSV: {e}")
            logging.error(f"Erro ao carregar HSV: {e}")

    def save_hsv(self):
        """Salva configuração HSV atual como dicionário."""
        lower, upper = self.get_hsv_bounds()
        return {
            "h_min": int(lower[0]), "h_max": int(upper[0]),
            "s_min": int(lower[1]), "s_max": int(upper[1]),
            "v_min": int(lower[2]), "v_max": int(upper[2])
        }

# -----------------------------------------------------------------------------
# ---------------------- FRAME PID CONTROLLER ---------------------------------
# -----------------------------------------------------------------------------
class PIDControllerFrame(ctk.CTkFrame):
    """
    Frame para controle de motores e comunicação serial.
    Inclui botões para ligar/desligar motores, ativar seguimento e gerenciar conexão.
    """
    def __init__(self, parent, serial_manager: SerialManager, app_ref, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.serial_manager = serial_manager
        self.app = app_ref
        self.grid_columnconfigure(0, weight=1)

        # Nota sobre PID no Arduino
        ctk.CTkLabel(
            self, text="Controlo PID feito no Arduino", font=("Arial", 14, "italic")
        ).grid(row=0, column=0, padx=10, pady=(10, 5))

        # Grupo de controle de motores
        motor_group = ctk.CTkFrame(self, fg_color="transparent")
        motor_group.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        motor_group.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkLabel(
            motor_group, text="Controlo do Sistema", font=("Arial", 14, "bold")
        ).grid(row=0, column=0, columnspan=2, pady=(5, 10))

        ctk.CTkButton(
            motor_group, text="Ligar Motores", command=self._ligar_motores, **COLOR_SUCCESS
        ).grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(
            motor_group, text="Desligar Motores", command=self._desligar_motores, **COLOR_DANGER
        ).grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        self.btn_ligar_seguimento = ctk.CTkButton(
            motor_group, text="Ligar Seguimento", command=self._toggle_seguimento, **COLOR_SECONDARY
        )
        self.btn_ligar_seguimento.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # Grupo de comunicação serial
        serial_group = ctk.CTkFrame(self, fg_color="transparent")
        serial_group.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        serial_group.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(
            serial_group, text="Comunicação Arduino", font=("Arial", 14, "bold")
        ).grid(row=0, column=0, columnspan=2, pady=(5, 10))

        portas = self.serial_manager.list_ports()
        self.porta_var = tk.StringVar(master=self, value=portas[0] if portas else "Nenhuma porta")
        self.option_menu_porta = ctk.CTkOptionMenu(
            serial_group, values=portas if portas else ["Nenhuma porta"], variable=self.porta_var
        )
        self.option_menu_porta.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(
            serial_group, text="🔄", width=30, command=self._atualizar_portas
        ).grid(row=1, column=1, padx=5, pady=5)

        ctk.CTkButton(
            serial_group, text="Ligar", command=self._ligar_arduino, **COLOR_SUCCESS
        ).grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(
            serial_group, text="Desligar", command=self._desligar_arduino, **COLOR_DANGER
        ).grid(row=2, column=1, padx=5, pady=5, sticky="ew")

        self.grid_rowconfigure(3, weight=1)

    def _toggle_seguimento(self):
        """Ativa/desativa o seguimento da bola."""
        vh = self.app.video_handler
        if not vh:
            return
        if not vh.tracking_enabled:
            vh.tracking_enabled = True
            self.btn_ligar_seguimento.configure(text="Parar Seguimento", **COLOR_DANGER)
            self.app.status_bar.show_message("Seguimento da bola ativado.")
            logging.info("Seguimento ativado")
        else:
            vh.tracking_enabled = False
            self.btn_ligar_seguimento.configure(text="Ligar Seguimento", **COLOR_SECONDARY)
            self.app.status_bar.show_message("Seguimento da bola desativado.")
            logging.info("Seguimento desativado")

    def _ligar_motores(self):
        """Envia comando para ligar motores."""
        self.serial_manager.send(CMD_MOTORS_ON)
        self.app.status_bar.show_message("Comando para ligar motores enviado.")
        logging.info("Comando M1 enviado")

    def _desligar_motores(self):
        """Envia comando para desligar motores."""
        self.serial_manager.send(CMD_MOTORS_OFF)
        self.app.status_bar.show_message("Comando para desligar motores enviado.")
        logging.info("Comando M0 enviado")

    def _atualizar_portas(self):
        """Atualiza lista de portas seriais disponíveis."""
        novas_portas = self.serial_manager.list_ports()
        self.option_menu_porta.configure(
            values=novas_portas if novas_portas else ["Nenhuma porta"]
        )
        self.porta_var.set(novas_portas[0] if novas_portas else "Nenhuma porta")
        self.app.status_bar.show_message("Portas disponíveis atualizadas.")
        logging.info("Portas seriais atualizadas")

    def _ligar_arduino(self):
        """Tenta conectar ao Arduino na porta selecionada."""
        porta = self.porta_var.get()
        if porta and porta != "Nenhuma porta":
            if self.serial_manager.connect(porta):
                self.app.status_bar.show_message(f"Conectado a {porta} com sucesso.", 5000)
            else:
                self.app.status_bar.show_message(f"Falha ao conectar a {porta}.", 5000, is_error=True)
        else:
            self.app.status_bar.show_message("Nenhuma porta serial selecionada.", 3000, is_error=True)

    def _desligar_arduino(self):
        """Desconecta do Arduino."""
        self.serial_manager.disconnect()
        self.app.status_bar.show_message("Conexão serial encerrada.")
        logging.info("Arduino desconectado")

    def get_pid_config(self):
        """Retorna configuração PID (não usada, PID no Arduino)."""
        return {"kp": 0.0, "ki": 0.0, "kd": 0.0}

    def load_pid_config(self, pid_dict):
        """Carrega configuração PID (não usada)."""
        pass

# -----------------------------------------------------------------------------
# -------------------------- GERENCIADOR DE VÍDEO -----------------------------
# -----------------------------------------------------------------------------
class VideoHandler:
    """
    Gerencia captura de vídeo, processamento de imagem e envio de erros ao Arduino.
    Inclui detecção de bola via HSV, mapeamento de erros para pulsos e atualização da GUI.
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
        self.tracking_enabled = False
        self.last_send_time = 0
        self.canvas_lock = Lock()  # Lock para sincronizar acesso ao canvas
        self.errX_history = deque(maxlen=5)  # Filtro de média móvel para erros
        self.errY_history = deque(maxlen=5)

        # Canvas para exibir vídeo
        self.canvas = ctk.CTkCanvas(
            parent, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, bg="gray20", highlightthickness=0
        )
        self.canvas.grid(row=2, column=0, padx=10, pady=10)

        self._initialize_camera()

    def _initialize_camera(self):
        """Inicializa a webcam com resolução otimizada."""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.running = False
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=1)

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            messagebox.showerror("Erro de Vídeo", f"Não foi possível aceder à webcam (índice {self.camera_index}).")
            self.cap = None
            logging.error(f"Falha ao abrir webcam {self.camera_index}")
        else:
            self.running = True
            self.thread = threading.Thread(target=self._video_loop, daemon=True)
            self.thread.start()
            logging.info(f"Webcam {self.camera_index} inicializada")

    def change_camera(self, new_index):
        """Altera a webcam usada."""
        try:
            idx = int(new_index)
        except ValueError:
            messagebox.showerror("Câmera", f"Índice de câmera inválido: {new_index}")
            logging.error(f"Índice de câmera inválido: {new_index}")
            return
        self.camera_index = idx
        self._initialize_camera()

    def update_once(self):
        """Processa um único frame para atualizar a visualização."""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self._process_and_draw(frame)

    def _video_loop(self):
        """Loop principal para captura e processamento de vídeo."""
        while self.running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                self.running = False
                logging.error("Falha na captura de vídeo")
                break
            self._process_and_draw(frame)

    def _process_and_draw(self, frame_bgr):
        """Processa frame, detecta bola, calcula erros e atualiza canvas."""
        # Converter para HSV e aplicar máscara
        hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower, upper = self.hsv_frame.get_hsv_bounds()
        mask = cv2.inRange(hsv_frame, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))  # Kernel menor
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Detectar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_center, ball_radius = None, None

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                ball_center, ball_radius = (int(x), int(y)), int(radius)

                # Calcular erros em pixels
                erro_x = ball_center[0] - (VIDEO_WIDTH // 2)
                erro_y = -(ball_center[1] - (VIDEO_HEIGHT // 2))  # Y invertido

                # Aplicar filtro de média móvel
                self.errX_history.append(erro_x)
                self.errY_history.append(erro_y)
                erro_x = sum(self.errX_history) / len(self.errX_history)
                erro_y = sum(self.errY_history) / len(self.errY_history)

                # Aplicar deadband
                if abs(erro_x) < DEADBAND_THRESHOLD and abs(erro_y) < DEADBAND_THRESHOLD:
                    erro_x, erro_y = 0, 0

                # Mapear erros para pulsos
                max_err_pixels_x = VIDEO_WIDTH // 2
                max_err_pulses_x = PULSE_MAX_X - PULSE_CENTER_X
                errX_mapped = (erro_x / max_err_pixels_x) * max_err_pulses_x
                max_err_pixels_y = VIDEO_HEIGHT // 2
                max_err_pulses_y = PULSE_MAX_Y - PULSE_CENTER_Y
                errY_mapped = (erro_y / max_err_pixels_y) * max_err_pulses_y

                # Enviar erros se seguimento ativado e intervalo respeitado
                if self.tracking_enabled and self.serial_manager.serial_conn:
                    current_time = time.time()
                    if current_time - self.last_send_time >= SERIAL_SEND_INTERVAL:
                        cmd_str = f"E,{errX_mapped:.2f},{errY_mapped:.2f}\n"
                        self.serial_manager.send(cmd_str)
                        self.last_send_time = current_time

        # Preparar imagem para exibição
        display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB) if self.show_mask else cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Desenhar cruz no centro
        cx, cy = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2
        cv2.line(display_img, (cx - 10, cy), (cx + 10, cy), (255, 255, 0), 1)
        cv2.line(display_img, (cx, cy - 10), (cx, cy + 10), (255, 255, 0), 1)

        # Desenhar bola detectada
        if ball_center and ball_radius:
            cv2.circle(display_img, ball_center, ball_radius, (0, 255, 0), 2)
            cv2.circle(display_img, ball_center, 3, (255, 0, 0), -1)
            erro_txt = f"Erro X: {erro_x:+.0f}, Y: {erro_y:+.0f}"
            cv2.putText(
                display_img, erro_txt, (10, VIDEO_HEIGHT - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA
            )

        # Converter imagem para Tkinter
        img_pil = Image.fromarray(display_img)
        img_tk = ImageTk.PhotoImage(image=img_pil)

        # Atualizar canvas na thread principal com lock
        with self.canvas_lock:
            def update_canvas():
                self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
                self.canvas.image = img_tk  # Evitar garbage collection
            self.canvas.after(0, update_canvas)

    def stop(self):
        """Para captura de vídeo e libera recursos."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)
        if self.cap:
            self.cap.release()
        logging.info("Captura de vídeo parada")

# -----------------------------------------------------------------------------
# -------------------------- BARRA DE STATUS ----------------------------------
# -----------------------------------------------------------------------------
class StatusBar(ctk.CTkFrame):
    """Barra de status para exibir mensagens temporárias com logging."""
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, corner_radius=0, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w")
        self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None

    def show_message(self, message, duration_ms=4000, is_error=False):
        """Exibe mensagem com duração e registra no log."""
        if self._job:
            self.after_cancel(self._job)

        original_color = self.label.cget("text_color")
        text_color = "#FF5555" if is_error else original_color

        self.label.configure(text=message, text_color=text_color)
        logging.info(message) if not is_error else logging.error(message)
        self._job = self.after(duration_ms, lambda: self.label.configure(text="", text_color=original_color))

# -----------------------------------------------------------------------------
# ----------------------------- APLICAÇÃO PRINCIPAL ---------------------------
# -----------------------------------------------------------------------------
class OpenBalanceApp:
    """Classe principal que integra todos os componentes da dashboard."""
    def __init__(self):
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        self.app = ctk.CTk()
        self.app.title("OpenBalance 3.0")
        self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")

        # Configuração do grid
        self.app.grid_rowconfigure(0, weight=1)
        self.app.grid_rowconfigure(1, weight=0)
        self.app.grid_columnconfigure(0, weight=1)
        self.app.grid_columnconfigure(1, weight=3)
        self.app.grid_columnconfigure(2, weight=1)

        self.serial_manager = SerialManager(self)
        self.video_handler = None

        self._create_menu()
        self._create_frames()
        self._create_status_bar()

        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _create_menu(self):
        """Cria menu superior com opções de arquivo."""
        menu_bar = tk.Menu(self.app)
        arquivo_menu = tk.Menu(menu_bar, tearoff=0)
        arquivo_menu.add_command(label="Salvar Configuração", command=self._salvar_config)
        arquivo_menu.add_command(label="Carregar Configuração", command=self._carregar_config)
        arquivo_menu.add_separator()
        arquivo_menu.add_command(label="Sair", command=self._on_close)
        menu_bar.add_cascade(label="Arquivo", menu=arquivo_menu)
        self.app.config(menu=menu_bar)

    def _create_status_bar(self):
        """Cria barra de status no rodapé."""
        self.status_bar = StatusBar(self.app)
        self.status_bar.grid(row=1, column=0, columnspan=3, sticky="sew")

    def _create_frames(self):
        """Cria frames para HSV, vídeo e PID."""
        # Frame HSV
        self.hsv_frame = HSVSettingsFrame(self.app, video_handler=None)
        self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)

        # Frame de vídeo
        frame_video_container = ctk.CTkFrame(self.app, corner_radius=10)
        frame_video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        frame_video_container.grid_rowconfigure(2, weight=1)
        frame_video_container.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(
            frame_video_container, text="Área de Visualização", font=("Arial", 16, "bold")
        ).grid(row=0, column=0, pady=(10,5))

        cam_select_frame = ctk.CTkFrame(frame_video_container, fg_color="transparent")
        cam_select_frame.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(cam_select_frame, text="Câmera:").pack(side="left", padx=(0, 5))
        self.cam_index_var = tk.StringVar(value="0")
        self.option_menu_camera = ctk.CTkOptionMenu(
            cam_select_frame, values=["0", "1", "2", "3"],
            variable=self.cam_index_var, command=self._on_camera_change
        )
        self.option_menu_camera.pack(side="left")

        # Inicializar VideoHandler
        self.video_handler = VideoHandler(
            frame_video_container, self.hsv_frame, self.serial_manager,
            camera_index=int(self.cam_index_var.get())
        )
        self.hsv_frame.video_handler = self.video_handler

        # Frame PID
        self.pid_frame = PIDControllerFrame(self.app, self.serial_manager, app_ref=self)
        self.pid_frame.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)

    def _on_camera_change(self, new_index):
        """Altera a câmera selecionada."""
        if self.video_handler:
            self.video_handler.change_camera(new_index)

    def _salvar_config(self):
        """Salva configuração HSV em arquivo JSON."""
        config = {"hsv": self.hsv_frame.save_hsv(), "pid": self.pid_frame.get_pid_config()}
        filepath = filedialog.asksaveasfilename(
            defaultextension=".json", filetypes=CONFIG_FILE_TYPES
        )
        if not filepath:
            return
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=4)
            self.status_bar.show_message(f"Configuração salva em {filepath}")
        except Exception as e:
            self.status_bar.show_message(f"Erro ao salvar: {e}", is_error=True)

    def _carregar_config(self):
        """Carrega configuração de arquivo JSON."""
        filepath = filedialog.askopenfilename(filetypes=CONFIG_FILE_TYPES)
        if not filepath:
            return
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                config = json.load(f)
            self.hsv_frame.load_hsv(config.get("hsv", {}))
            self.pid_frame.load_pid_config(config.get("pid", {}))
            self.status_bar.show_message("Configuração carregada com sucesso.")
            if self.video_handler:
                self.video_handler.update_once()
        except Exception as e:
            self.status_bar.show_message(f"Erro ao carregar: {e}", is_error=True)

    def _on_close(self):
        """Fecha a aplicação, liberando recursos."""
        if self.video_handler:
            self.video_handler.stop()
        self.serial_manager.disconnect()
        self.app.destroy()
        logging.info("Aplicação encerrada")

    def run(self):
        """Inicia o loop principal da GUI."""
        self.app.mainloop()

# -----------------------------------------------------------------------------
# -------------------------- PONTO DE ENTRADA DO SCRIPT -----------------------
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    app = OpenBalanceApp()
    app.run()