"""
OpenBalance Dashboard (Versão Final com Presets de Cores e Detecção de Bola)
Descrição: Dashboard para controlo de bola numa plataforma inclinável,
utilizando visão computacional (HSV) e PID.
Autor: João Pavão (Refatorado por ChatGPT)
Finalidade: Interface gráfica para ajuste de parâmetros HSV, controlo PID,
visualização de vídeo com preview de máscara em tempo real, botões de presets de cor,
desenho de círculo ao redor da bola detectada, calibração básica, conexão serial e exportação de CSV.
Uso: Código aberto, disponível livremente sob licença MIT.
"""

import json
import threading
import cv2
import numpy as np
import customtkinter as ctk         # GUI estilizada com CustomTkinter
import serial                        # Comunicação serial com Arduino
import serial.tools.list_ports      # Para listar portas seriais
from PIL import Image, ImageTk      # Converter frames OpenCV em imagens Tkinter
import tkinter as tk                # GUI nativa para menus e diálogos
from tkinter import filedialog, messagebox
import csv                          # Biblioteca para manipulação de CSV

# -----------------------------------------------------------------------------
# --------------------------- CONSTANTES E CONFIGURAÇÕES ---------------------
# -----------------------------------------------------------------------------
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 700
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
DEFAULT_BAUDRATE = 9600
CONFIG_FILE_TYPES = [("JSON Files", "*.json")]
CSV_FILE_TYPES = [("CSV Files", "*.csv")]

# Valores HSV de exemplo para presets (ajustar conforme iluminação/câmera)
HSV_PRESETS = {
    "Vermelho": {
        "h_min": 0,   "h_max": 10,
        "s_min": 100, "s_max": 255,
        "v_min": 100, "v_max": 255
    },
    "Laranja": {
        "h_min": 10,  "h_max": 25,
        "s_min": 100, "s_max": 255,
        "v_min": 100, "v_max": 255
    },
    "Verde": {
        "h_min": 40,  "h_max": 80,
        "s_min": 100, "s_max": 255,
        "v_min": 100, "v_max": 255
    }
}

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
        """Retorna lista de portas seriais disponíveis no sistema."""
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        """
        Abre conexão serial na porta especificada.
        Exibe mensagem de sucesso ou erro via messagebox.
        """
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            messagebox.showinfo("Conexão Serial", f"Conectado à porta {port} @ {baudrate}bps")
        except Exception as e:
            messagebox.showerror("Erro Serial", f"Falha ao conectar: {e}")
            self.serial_conn = None

    def disconnect(self):
        """
        Fecha a conexão serial se estiver aberta.
        """
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            messagebox.showinfo("Conexão Serial", "Conexão encerrada.")
            self.serial_conn = None

    def send(self, data_str):
        """
        Envia string para o Arduino (se conectado).
        Exibe alerta se não estiver conectado.
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            messagebox.showwarning("Serial", "Não conectado ao Arduino")
            return
        try:
            self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException as e:
            messagebox.showerror("Serial", f"Erro ao enviar dados: {e}")
            self.serial_conn = None

# -----------------------------------------------------------------------------
# ----------------------- FRAME DE CONFIGURAÇÕES HSV --------------------------
# -----------------------------------------------------------------------------
class HSVSettingsFrame(ctk.CTkFrame):
    """
    Frame para ajuste de parâmetros HSV e preview de máscara.
    Inclui sliders HMin, HMax, SMin, SMax, VMin, VMax, checkbox 'Mostrar Máscara'
    e botões de presets de cor para valores predefinidos de HSV.
    """
    def __init__(self, parent, video_handler, *args, **kwargs):
        """
        :param parent: widget pai (geralmente a janela principal).
        :param video_handler: instância de VideoHandler para alternar máscara.
        """
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.video_handler = video_handler  # referência para acessar vídeo
        self.grid_columnconfigure(0, weight=1)

        # Título do painel HSV
        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 16)).grid(
            row=0, column=0, pady=(10, 5), sticky="n"
        )

        # Listas para armazenar sliders e variáveis associadas
        self.hsv_sliders = []   # widgets Slider para cada parâmetro
        self.hsv_values = []    # StringVars que guardam o valor atual do slider

        labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
        for idx, label in enumerate(labels):
            # Rótulo do parâmetro
            ctk.CTkLabel(self, text=label).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")

            # Variável que armazena o valor textual
            val_var = ctk.StringVar(master=self, value="0")
            self.hsv_values.append(val_var)

            # Slider de 0 a 255
            slider = ctk.CTkSlider(
                self,
                from_=0,
                to=255,
                number_of_steps=255,
                command=lambda val, var=val_var: var.set(f"{int(float(val))}")
            )
            slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew")
            self.hsv_sliders.append(slider)

            # Label que exibe o valor atual do slider, alinhado à direita
            ctk.CTkLabel(self, textvariable=val_var).grid(
                row=2 + 2*idx, column=0, padx=(270,10), sticky="e"
            )

        # Checkbox para mostrar/ocultar a máscara HSV
        self.show_mask_var = ctk.BooleanVar(master=self, value=False)
        ctk.CTkCheckBox(
            self,
            text="Mostrar Máscara",
            variable=self.show_mask_var,
            command=self._on_toggle_mask
        ).grid(row=13, column=0, pady=(10,5), padx=10, sticky="w")

        # Botões de presets de cor
        row_base = 14
        for i, color_name in enumerate(HSV_PRESETS.keys()):
            ctk.CTkButton(
                self,
                text=color_name,
                command=lambda cn=color_name: self._apply_preset(cn)
            ).grid(row=row_base + i, column=0, pady=(2,2), padx=10, sticky="ew")

    def _apply_preset(self, color_name):
        """
        Ajusta sliders de HSV para os valores predefinidos de um preset.
        :param color_name: chave em HSV_PRESETS (ex.: 'Vermelho', 'Laranja', 'Verde')
        """
        preset = HSV_PRESETS.get(color_name, None)
        if not preset:
            return
        sliders_vals = [
            preset["h_min"], preset["h_max"],
            preset["s_min"], preset["s_max"],
            preset["v_min"], preset["v_max"]
        ]
        for i, val in enumerate(sliders_vals):
            self.hsv_sliders[i].set(val)
            self.hsv_values[i].set(str(val))
        # Caso queira atualizar máscara imediatamente, pode forçar redraw:
        if self.video_handler:
            self.video_handler.update_once()

    def _on_toggle_mask(self):
        """
        Callback disparado ao marcar/desmarcar 'Mostrar Máscara'.
        Informa o VideoHandler para alternar entre feed normal e máscara.
        """
        if self.video_handler:
            self.video_handler.show_mask = self.show_mask_var.get()

    def get_hsv_bounds(self):
        """
        Retorna tupla (lower, upper) com arrays numpy dos limites HSV atuais.
        :return: (np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max]))
        """
        try:
            h_min = int(self.hsv_values[0].get())
            h_max = int(self.hsv_values[1].get())
            s_min = int(self.hsv_values[2].get())
            s_max = int(self.hsv_values[3].get())
            v_min = int(self.hsv_values[4].get())
            v_max = int(self.hsv_values[5].get())
        except ValueError:
            # Se conversão falhar, usar full range
            h_min, h_max, s_min, s_max, v_min, v_max = 0, 255, 0, 255, 0, 255

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        return lower, upper

    def load_hsv(self, hsv_dict):
        """
        Carrega valores de HSV a partir de dicionário:
        hsv_dict = {"h_min": int, "h_max": int, "s_min": int, "s_max": int, "v_min": int, "v_max": int}
        Atualiza sliders e StringVars.
        """
        try:
            sliders_vals = [
                hsv_dict.get("h_min", 0), hsv_dict.get("h_max", 255),
                hsv_dict.get("s_min", 0), hsv_dict.get("s_max", 255),
                hsv_dict.get("v_min", 0), hsv_dict.get("v_max", 255)
            ]
            for i, val in enumerate(sliders_vals):
                self.hsv_sliders[i].set(val)
                self.hsv_values[i].set(str(val))
        except Exception as e:
            messagebox.showerror("Erro HSV", f"Falha ao carregar HSV: {e}")

    def save_hsv(self):
        """
        Retorna dicionário com os valores atuais de HSV.
        """
        lower, upper = self.get_hsv_bounds()
        return {
            "h_min": int(lower[0]), "h_max": int(upper[0]),
            "s_min": int(lower[1]), "s_max": int(upper[1]),
            "v_min": int(lower[2]), "v_max": int(upper[2]),
        }

# -----------------------------------------------------------------------------
# ---------------------------- FRAME PID CONTROLLER ---------------------------
# -----------------------------------------------------------------------------
class PIDControllerFrame(ctk.CTkFrame):
    """
    Frame para ajuste de parâmetros do controlador PID.
    Inclui sliders Kp, Ki, Kd, Max Integral, campos de Offset, botões de controle,
    conexão Arduino e botão para salvar PID em CSV.
    """
    def __init__(self, parent, serial_manager: SerialManager, *args, **kwargs):
        """
        :param parent: widget pai (geralmente o container do PID).
        :param serial_manager: instância de SerialManager para gerenciar conexão.
        """
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.serial_manager = serial_manager  # referência para enviar comandos
        self.grid_columnconfigure(0, weight=1)

        # Título do painel PID
        ctk.CTkLabel(self, text="Controlador PID", font=("Arial", 16)).grid(
            row=0, column=0, pady=(10,5), sticky="n"
        )

        # Sliders e StringVars para Kp, Ki, Kd
        self.pid_values = []    # StringVars para exibir cada valor
        for idx, param in enumerate(["Kp", "Ki", "Kd"]):
            ctk.CTkLabel(self, text=param).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(master=self, value="0.00")
            slider = ctk.CTkSlider(
                self,
                from_=0,
                to=10,
                number_of_steps=100,
                command=lambda val, var=val_var: var.set(f"{float(val):.2f}")
            )
            slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew")
            ctk.CTkLabel(self, textvariable=val_var).grid(
                row=2 + 2*idx, column=0, padx=(270,10), sticky="e"
            )
            self.pid_values.append(val_var)

        # Slider para Max Integral
        ctk.CTkLabel(self, text="Max Integral").grid(row=7, column=0, padx=10, sticky="w")
        self.max_integral_var = ctk.StringVar(master=self, value="0")
        max_int_slider = ctk.CTkSlider(
            self,
            from_=0,
            to=500,
            number_of_steps=100,
            command=lambda val: self.max_integral_var.set(f"{int(float(val))}")
        )
        max_int_slider.grid(row=8, column=0, padx=10, pady=(0,5), sticky="ew")
        ctk.CTkLabel(self, textvariable=self.max_integral_var).grid(
            row=8, column=0, padx=(270,10), sticky="e"
        )

        # Campos de Offset X / Y
        ctk.CTkLabel(self, text="Offset X / Y").grid(row=9, column=0, pady=(10,2), padx=10, sticky="w")
        self.offset_x_entry = ctk.CTkEntry(self, placeholder_text="Offset X")
        self.offset_x_entry.grid(row=10, column=0, pady=2, padx=10, sticky="ew")
        self.offset_y_entry = ctk.CTkEntry(self, placeholder_text="Offset Y")
        self.offset_y_entry.grid(row=11, column=0, pady=2, padx=10, sticky="ew")

        # Botões principais (ainda sem lógica interna)
        ctk.CTkButton(self, text="Auto Calibrar", command=self._auto_calibrar).grid(
            row=12, column=0, pady=(10,5), padx=10, sticky="ew"
        )
        ctk.CTkButton(self, text="Iniciar Seguimento", command=self._iniciar_seguimento).grid(
            row=13, column=0, pady=5, padx=10, sticky="ew"
        )
        ctk.CTkButton(self, text="Parar Seguimento", command=self._parar_seguimento).grid(
            row=14, column=0, pady=5, padx=10, sticky="ew"
        )
        ctk.CTkButton(self, text="Ligar Motores", command=self._ligar_motores).grid(
            row=15, column=0, pady=5, padx=10, sticky="ew"
        )
        ctk.CTkButton(self, text="Desligar Motores", command=self._desligar_motores).grid(
            row=16, column=0, pady=(5,10), padx=10, sticky="ew"
        )

        # ------------------ Conexão Arduino ------------------
        ctk.CTkLabel(self, text="Conexão Arduino", font=("Arial", 14)).grid(
            row=17, column=0, pady=(10,5), padx=10, sticky="w"
        )
        # Listar portas disponíveis via SerialManager
        portas = self.serial_manager.list_ports()
        self.porta_var = tk.StringVar(master=self, value=portas[0] if portas else "")

        self.option_menu_porta = ctk.CTkOptionMenu(
            self,
            values=portas,
            variable=self.porta_var,
            fg_color="green",            # cor de fundo verde
            button_color="green",        # cor do botão verde
            button_hover_color="darkgreen"  # cor ao passar o mouse
        )
        self.option_menu_porta.grid(row=18, column=0, pady=(0,5), padx=10, sticky="ew")

        ctk.CTkButton(
            self,
            text="Ligar Arduino",
            command=self._ligar_arduino,
            fg_color="green",
            hover_color="darkgreen"
        ).grid(row=19, column=0, pady=5, padx=10, sticky="ew")

        ctk.CTkButton(
            self,
            text="Desligar Arduino",
            command=self._desligar_arduino,
            fg_color="green",
            hover_color="darkgreen"
        ).grid(row=20, column=0, pady=(0,10), padx=10, sticky="ew")

        # ------------------ Botão Salvar PID em CSV ------------------
        ctk.CTkLabel(self, text="", height=10).grid(row=21, column=0)  # espaçamento
        ctk.CTkButton(
            self,
            text="Guardar PID em CSV",
            command=self._save_pid_csv,
            fg_color="orange",
            hover_color="darkorange"
        ).grid(row=22, column=0, pady=(0,10), padx=10, sticky="ew")

        # Garantir que o scroll funcione corretamente
        for r in range(0, 23):
            self.grid_rowconfigure(r, weight=0)
        self.grid_rowconfigure(23, weight=1)  # linha final expansível

    # -------- Métodos de Callback dos Botões --------

    def _auto_calibrar(self):
        """Placeholder para auto-calibração."""
        messagebox.showinfo("Auto Calibrar", "Rotina de auto-calibração ainda não implementada.")

    def _iniciar_seguimento(self):
        """Placeholder para iniciar seguimento da bola via PID."""
        messagebox.showinfo("Seguimento", "Iniciar seguimento PID ainda não implementado.")

    def _parar_seguimento(self):
        """Placeholder para parar seguimento da bola."""
        messagebox.showinfo("Seguimento", "Parar seguimento ainda não implementado.")

    def _ligar_motores(self):
        """Placeholder para enviar comando de ligar motores."""
        messagebox.showinfo("Motores", "Ligar motores ainda não implementado.")

    def _desligar_motores(self):
        """Placeholder para enviar comando de desligar motores."""
        messagebox.showinfo("Motores", "Desligar motores ainda não implementado.")

    def _ligar_arduino(self):
        """Chama SerialManager.connect usando a porta selecionada."""
        porta = self.porta_var.get()
        if porta:
            self.serial_manager.connect(porta)
        else:
            messagebox.showwarning("Porta", "Nenhuma porta selecionada.")

    def _desligar_arduino(self):
        """Chama SerialManager.disconnect para fechar conexão."""
        self.serial_manager.disconnect()

    def _save_pid_csv(self):
        """
        Salva os valores atuais de PID e offsets num arquivo CSV.
        Usa get_pid_config() para obter dicionário de valores.
        """
        pid_cfg = self.get_pid_config()
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=CSV_FILE_TYPES
        )
        if filepath:
            try:
                with open(filepath, mode='w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    writer.writerow(["Kp", "Ki", "Kd", "MaxIntegral", "OffsetX", "OffsetY"])
                    writer.writerow([
                        f"{pid_cfg['kp']:.2f}",
                        f"{pid_cfg['ki']:.2f}",
                        f"{pid_cfg['kd']:.2f}",
                        f"{pid_cfg['max_integral']}",
                        f"{pid_cfg['offset_x']:.2f}",
                        f"{pid_cfg['offset_y']:.2f}"
                    ])
                messagebox.showinfo("Salvar CSV", f"Dados salvos em:\n{filepath}")
            except Exception as e:
                messagebox.showerror("Erro CSV", f"Falha ao salvar CSV: {e}")

    # -------- Métodos para Obtenção e Carregamento de Config --------

    def get_pid_config(self):
        """
        Retorna dicionário com valores atuais de PID e offsets:
        {"kp": float, "ki": float, "kd": float, "max_integral": int,
         "offset_x": float, "offset_y": float}
        """
        try:
            kp = float(self.pid_values[0].get())
            ki = float(self.pid_values[1].get())
            kd = float(self.pid_values[2].get())
            max_int = int(self.max_integral_var.get())
            offset_x = float(self.offset_x_entry.get()) if self.offset_x_entry.get() else 0.0
            offset_y = float(self.offset_y_entry.get()) if self.offset_y_entry.get() else 0.0
        except ValueError:
            # Caso conversão falhe, usar valores padrão
            kp, ki, kd, max_int, offset_x, offset_y = 0.0, 0.0, 0.0, 0, 0.0, 0.0
        return {
            "kp": kp, "ki": ki, "kd": kd,
            "max_integral": max_int,
            "offset_x": offset_x, "offset_y": offset_y
        }

    def load_pid_config(self, pid_dict):
        """
        Carrega valores de PID e offsets a partir de dicionário:
        pid_dict = {"kp": float, "ki": float, "kd": float,
                    "max_integral": int, "offset_x": float, "offset_y": float}
        Atualiza sliders e entradas de texto.
        """
        try:
            # Atualizar sliders Kp, Ki, Kd
            sliders_vals = [pid_dict.get("kp", 0.0), pid_dict.get("ki", 0.0), pid_dict.get("kd", 0.0)]
            for i, val in enumerate(sliders_vals):
                # Encontrar slider via grid_slaves
                slider_widget = [w for w in self.grid_slaves(row=2 + 2*i, column=0) if isinstance(w, ctk.CTkSlider)]
                if slider_widget:
                    slider_widget[0].set(val)
                self.pid_values[i].set(f"{val:.2f}")

            # Atualizar Max Integral
            max_int = pid_dict.get("max_integral", 0)
            max_int_slider = [w for w in self.grid_slaves(row=8, column=0) if isinstance(w, ctk.CTkSlider)]
            if max_int_slider:
                max_int_slider[0].set(max_int)
            self.max_integral_var.set(str(int(max_int)))

            # Atualizar offsets
            self.offset_x_entry.delete(0, tk.END)
            self.offset_x_entry.insert(0, str(pid_dict.get("offset_x", 0.0)))
            self.offset_y_entry.delete(0, tk.END)
            self.offset_y_entry.insert(0, str(pid_dict.get("offset_y", 0.0)))
        except Exception as e:
            messagebox.showerror("Erro PID", f"Falha ao carregar PID: {e}")

# -----------------------------------------------------------------------------
# -------------------------- GERENCIADOR DE VÍDEO -----------------------------
# -----------------------------------------------------------------------------
class VideoHandler:
    """
    Classe que cria um canvas para exibir feed de vídeo (ou máscara) e gerencia um thread
    de captura baseado em OpenCV. Alterna entre vídeo normal e máscara HSV, e desenha
    um círculo ao redor da bola detectada.
    """
    def __init__(self, parent, hsv_frame: HSVSettingsFrame):
        """
        :param parent: widget pai onde será colocado o canvas de vídeo.
        :param hsv_frame: referência ao HSVSettingsFrame para obter limites de máscara.
        """
        self.parent = parent
        self.hsv_frame = hsv_frame
        self.show_mask = False      # se True, exibe somente a máscara; se False, exibe vídeo com círculo
        self.cap = None             # objeto VideoCapture
        self.thread = None          # thread de captura de vídeo
        self.running = False        # flag indicando se o loop de vídeo está ativo

        # Configurações do canvas de vídeo
        self.canvas = ctk.CTkCanvas(
            parent,
            width=VIDEO_WIDTH,
            height=VIDEO_HEIGHT,
            bg="gray20",
            highlightthickness=0
        )
        self.canvas.grid(row=1, column=0, padx=10, pady=10)

        # Labels de status (servos / PID)
        self.status_frame = ctk.CTkFrame(parent, corner_radius=5)
        self.status_frame.grid(row=2, column=0, pady=5)
        self.servo_x_var = ctk.StringVar(master=parent, value="Servo X: 0.0°")
        self.servo_y_var = ctk.StringVar(master=parent, value="Servo Y: 0.0°")
        self.pid_x_var = ctk.StringVar(master=parent, value="PID X: 0.00")
        self.pid_y_var = ctk.StringVar(master=parent, value="PID Y: 0.00")

        ctk.CTkLabel(self.status_frame, textvariable=self.servo_x_var).grid(
            row=0, column=0, padx=10, pady=(5,2), sticky="w"
        )
        ctk.CTkLabel(self.status_frame, textvariable=self.servo_y_var).grid(
            row=1, column=0, padx=10, pady=(2,5), sticky="w"
        )
        ctk.CTkLabel(self.status_frame, textvariable=self.pid_x_var).grid(
            row=0, column=1, padx=10, pady=(5,2), sticky="w"
        )
        ctk.CTkLabel(self.status_frame, textvariable=self.pid_y_var).grid(
            row=1, column=1, padx=10, pady=(2,5), sticky="w"
        )

        # Tentar inicializar a webcam
        self._initialize_camera()

    def _initialize_camera(self):
        """
        Tenta abrir a câmera em índice 1 (ou 0 se necessário). Se falhar, exibe um único erro
        e desabilita o loop de vídeo.
        """
        self.cap = cv2.VideoCapture(1)  # 0 para padrão, 1 para outra fonte
        if not self.cap.isOpened():
            messagebox.showerror("Erro de Vídeo", "Não foi possível aceder à webcam. Vídeo desabilitado.")
            self.cap = None
        else:
            # Inicia thread de captura se a câmera estiver aberta
            self.running = True
            self.thread = threading.Thread(target=self._video_loop, daemon=True)
            self.thread.start()

    def update_once(self):
        """
        Método auxiliar para forçar cálculo de máscara e redesenho de frame 
        uma única vez, útil ao aplicar presets para ver efeito imediato.
        """
        # Executa uma iteração de vídeo sem loop contínuo
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                return
            frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT), interpolation=cv2.INTER_AREA)
            # Recalcula mask e desenha círculo (mesma lógica que no loop)
            self._process_and_draw(frame)

    def _video_loop(self):
        """
        Loop contínuo de captura de frames. Processa e exibe frame ou máscara,
        desenhando o círculo ao redor da bola detectada em tempo real.
        Executado em thread separado para não travar a GUI.
        """
        while self.running and self.cap:
            ret, frame = self.cap.read()
            if not ret:
                self.running = False
                messagebox.showerror("Erro de Vídeo", "Falha ao ler frame da webcam.")
                break

            frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT), interpolation=cv2.INTER_AREA)
            self._process_and_draw(frame)
            # Pequeno delay para reduzir uso de CPU (10ms aprox.)
            cv2.waitKey(10)

    def _process_and_draw(self, frame_bgr):
        """
        Processa um frame BGR:
        1) Converte para HSV, aplica máscara e encontra contornos.
        2) Identifica o maior contorno como a bola (se atender a área mínima).
        3) Desenha o círculo ao redor da bola no display.
        4) Exibe seja o feed normal (com círculo) ou só a máscara, conforme `self.show_mask`.
        """
        # Converter para HSV e calcular máscara
        hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower, upper = self.hsv_frame.get_hsv_bounds()
        mask = cv2.inRange(hsv_frame, lower, upper)

        # Operações morfológicas para reduzir ruído
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Encontrar contornos na máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_center = None
        ball_radius = None

        if contours:
            # Escolher maior contorno por área
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 500:  # limiar de área para ignorar ruídos pequenos (ajustar conforme necessário)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                ball_center = (int(x), int(y))
                ball_radius = int(radius)

        # Preparar imagem para exibir
        if self.show_mask:
            # Converter máscara para RGB (escala de cinza → 3 canais) para desenhar círculo
            display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        else:
            display_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Desenhar círculo ao redor da bola (se encontrada)
        if ball_center and ball_radius:
            # Cor do círculo: verde (0,255,0) no formato RGB
            cv2.circle(display_img, ball_center, ball_radius, (0, 255, 0), 2)
            # Opcional: marcar centro
            cv2.circle(display_img, ball_center, 3, (255, 0, 0), -1)

        # Converter para PIL ImageTk
        img_pil = Image.fromarray(display_img)
        img_tk = ImageTk.PhotoImage(image=img_pil)

        # Exibir no canvas (sempre na posição 0,0)
        self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
        # Manter referência para evitar garbage collection
        self.canvas.image = img_tk

    def stop(self):
        """
        Para o loop de vídeo, libera a câmera e aguarda término da thread.
        """
        self.running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)

# -----------------------------------------------------------------------------
# ----------------------------- APLICAÇÃO PRINCIPAL ---------------------------
# -----------------------------------------------------------------------------
class OpenBalanceApp:
    """
    Classe principal que monta a janela, menus e integra os frames de HSV, vídeo e PID.
    Implementa funcionalidades de salvar/carregar configuração, presets de cor,
    layout responsivo e cleanup.
    """
    def __init__(self):
        # Configurações gerais da janela
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        self.app = ctk.CTk()
        self.app.title("OpenBalance")
        self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.app.grid_rowconfigure(0, weight=1)
        self.app.grid_columnconfigure(0, weight=1)  # Coluna HSV
        self.app.grid_columnconfigure(1, weight=3)  # Coluna Vídeo
        self.app.grid_columnconfigure(2, weight=1)  # Coluna PID

        # Instanciar gerenciador serial (para conexão Arduino)
        self.serial_manager = SerialManager()

        # Criar menus (Arquivo, Configurações, Ajuda)
        self._create_menu()

        # Criar frames e widgets principais
        self._create_frames()

        # Protocolo para fechar a aplicação corretamente
        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _create_menu(self):
        """
        Cria a barra de menus superior com itens: Arquivo (Salvar/Carregar/Sair),
        Configurações (Resetar), Ajuda (Sobre).
        """
        menu_bar = tk.Menu(self.app)

        # --- Menu Arquivo ---
        arquivo_menu = tk.Menu(menu_bar, tearoff=0)
        arquivo_menu.add_command(label="Salvar Configuração", command=self._salvar_config)
        arquivo_menu.add_command(label="Carregar Configuração", command=self._carregar_config)
        arquivo_menu.add_separator()
        arquivo_menu.add_command(label="Sair", command=self.app.quit)
        menu_bar.add_cascade(label="Arquivo", menu=arquivo_menu)

        # --- Menu Configurações ---
        config_menu = tk.Menu(menu_bar, tearoff=0)
        config_menu.add_command(label="Resetar Valores", command=self._resetar_valores)
        menu_bar.add_cascade(label="Configurações", menu=config_menu)

        # --- Menu Ajuda ---
        ajuda_menu = tk.Menu(menu_bar, tearoff=0)
        ajuda_menu.add_command(label="Sobre", command=self._mostrar_sobre)
        menu_bar.add_cascade(label="Ajuda", menu=ajuda_menu)

        self.app.config(menu=menu_bar)

    def _create_frames(self):
        """
        Instancia e posiciona os frames: HSVSettingsFrame, VideoHandler e PIDControllerFrame
        usando grid para layout responsivo.
        """
        # Frame HSV (coluna 0)
        self.hsv_frame = HSVSettingsFrame(self.app, video_handler=None)
        self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)

        # Frame Vídeo (coluna 1): contêiner para rótulo + VideoHandler
        frame_video_container = ctk.CTkFrame(self.app, corner_radius=10)
        frame_video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        frame_video_container.grid_rowconfigure(1, weight=1)
        frame_video_container.grid_columnconfigure(0, weight=1)

        # Rótulo do painel de vídeo
        ctk.CTkLabel(frame_video_container, text="Área de Visualização", font=("Arial", 16)).grid(
            row=0, column=0, pady=(10,5)
        )

        # Instanciar VideoHandler, passando referência de HSV
        self.video_handler = VideoHandler(frame_video_container, self.hsv_frame)
        # Agora que temos VideoHandler, informar HSVSettingsFrame sobre ele
        self.hsv_frame.video_handler = self.video_handler

        # Rótulo "Modo de Controlo" posicionado abaixo do status
        ctk.CTkLabel(frame_video_container, text="Modo de Controlo").grid(row=3, column=0, pady=(5,2))
        # Botão de seleção de modo (Manual, Círculo, etc.)
        self.ctrl_menu = ctk.CTkOptionMenu(
            frame_video_container,
            values=["Manual", "Círculo", "Oito", "Quadrado", "Senoide"]
        )
        self.ctrl_menu.grid(row=4, column=0, pady=(0,10))

        # Botões Trajetória (embaixo do menu de modo)
        ctk.CTkButton(
            frame_video_container,
            text="Executar Trajetória",
            command=self._executar_trajetoria
        ).grid(row=5, column=0, pady=(0,5))
        ctk.CTkButton(
            frame_video_container,
            text="Parar Movimento",
            command=self._parar_trajetoria
        ).grid(row=6, column=0, pady=(0,10))

        # Frame PID (coluna 2), dentro de um scrollable frame para evitar perda de botões
        frame_pid_container = ctk.CTkScrollableFrame(self.app, corner_radius=10, width=250)
        frame_pid_container.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)
        frame_pid_container.grid_rowconfigure(0, weight=1)
        frame_pid_container.grid_columnconfigure(0, weight=1)

        # Instanciar PIDControllerFrame, passando SerialManager
        self.pid_frame = PIDControllerFrame(frame_pid_container, self.serial_manager)
        self.pid_frame.grid(row=0, column=0, sticky="nswe", padx=0, pady=0)

    # ------------------ MÉTODOS DE MENU ------------------

    def _salvar_config(self):
        """
        Salva configuração atual de HSV e PID em arquivo JSON.
        Abre diálogo de seleção de arquivo; se o usuário confirmar, grava JSON.
        """
        config = {
            "hsv": self.hsv_frame.save_hsv(),
            "pid": self.pid_frame.get_pid_config()
        }
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=CONFIG_FILE_TYPES)
        if not filepath:
            return  # usuário cancelou
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=4)
            messagebox.showinfo("Salvar Configuração", f"Configuração salva em:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Erro Salvar", f"Falha ao salvar configuração: {e}")

    def _carregar_config(self):
        """
        Carrega configuração de HSV e PID a partir de arquivo JSON.
        Abre diálogo para escolher JSON; se válido, atualiza sliders e campos.
        """
        filepath = filedialog.askopenfilename(filetypes=CONFIG_FILE_TYPES)
        if not filepath:
            return  # usuário cancelou
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                config = json.load(f)
            hsv_cfg = config.get("hsv", {})
            pid_cfg = config.get("pid", {})
            self.hsv_frame.load_hsv(hsv_cfg)
            self.pid_frame.load_pid_config(pid_cfg)
            messagebox.showinfo("Carregar Configuração", "Configuração carregada com sucesso.")
            # Atualizar vídeo imediatamente
            self.video_handler.update_once()
        except Exception as e:
            messagebox.showerror("Erro Carregar", f"Falha ao carregar configuração: {e}")

    def _resetar_valores(self):
        """
        Reseta valores de HSV e PID para padrões (0/255 ou 0.0).
        Desmarca checkbox de máscara.
        """
        # Resetar HSV para full-range
        default_hsv = {"h_min": 0, "h_max": 255, "s_min": 0, "s_max": 255, "v_min": 0, "v_max": 255}
        self.hsv_frame.load_hsv(default_hsv)

        # Resetar PID para zeros
        default_pid = {"kp": 0.0, "ki": 0.0, "kd": 0.0, "max_integral": 0, "offset_x": 0.0, "offset_y": 0.0}
        self.pid_frame.load_pid_config(default_pid)

        # Desmarcar checkbox de máscara
        self.hsv_frame.show_mask_var.set(False)
        self.video_handler.show_mask = False

        messagebox.showinfo("Resetar", "Valores de HSV e PID foram resetados para padrão.")
        # Atualizar vídeo imediatamente
        self.video_handler.update_once()

    def _mostrar_sobre(self):
        """
        Exibe caixa de mensagem 'Sobre' com informações do aplicativo.
        """
        messagebox.showinfo(
            "Sobre",
            "OpenBalance Dashboard\nAutor: João Pavão\nVersão: 1.0\nRefatorado com presets de cor e detecção de bola"
        )

    # ------------------ MÉTODOS DE TRAJETÓRIA ------------------

    def _executar_trajetoria(self):
        """
        Placeholder para execução de trajetória (não implementado nesta fase).
        """
        modo = self.ctrl_menu.get()
        messagebox.showinfo("Trajetória", f"Executar trajetória: {modo} (ainda não implementado)")

    def _parar_trajetoria(self):
        """
        Placeholder para parar movimento de trajetória.
        """
        messagebox.showinfo("Trajetória", "Parar trajetória (ainda não implementado)")

    # ------------------ MÉTODO DE ENCERRAMENTO ------------------

    def _on_close(self):
        """
        Quando o usuário fecha a janela, chamar métodos de cleanup:
        parar vídeo, liberar câmera, desconectar serial, e destruir janela.
        """
        # Parar loop de vídeo e liberar câmera
        if hasattr(self, "video_handler") and self.video_handler:
            self.video_handler.stop()

        # Desconectar serial (se estiver conectada)
        self.serial_manager.disconnect()

        # Encerra aplicação
        self.app.destroy()

    def run(self):
        """Inicia o loop principal da GUI."""
        self.app.mainloop()

# -----------------------------------------------------------------------------
# -------------------------- PONTO DE ENTRADA DO SCRIPT -----------------------
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    app = OpenBalanceApp()
    app.run()
