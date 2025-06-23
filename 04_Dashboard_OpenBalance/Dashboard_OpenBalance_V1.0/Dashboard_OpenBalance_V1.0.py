"""
===============================================================================
Projeto: OpenBalance – Dashboard Python com Visão e Controlo PID

Ficheiro: Dashboard_OpenBalance_V1.0.py

Descrição: Interface gráfica para controlo em tempo real de uma plataforma
           equilibradora de bola, com deteção por OpenCV e controlo PID via Arduino.
           
Autor: João Pavão

Data: 22/06/2025

Disciplina: Laboratório de Aplicações em Robótica e Aprendizagem (PRIA)
Instituição: Universidade dos Açores
Licença: MIT License
===============================================================================
Notas:
- Esta versão integra uma janela de gráfico para visualização dos erros (X/Y)
  em tempo real, ideal para tuning de PID.
- A comunicação com o Arduino é feita via serial (comandos: M1, M0, E,x,y).
- Código modular com classes bem definidas: vídeo, HSV, controlo, gráfico, UI.
===============================================================================

# Índice de Estrutura e Comentários do Código

Este índice resume a organização da aplicação `Dashboard_OpenBalance_V1.0.py`,
facilitando a navegação por classes e blocos principais.

| Secção | Bloco / Classe              | Finalidade                                                                  |
|--------|-----------------------------|-----------------------------------------------------------------------------|
| 1      | `IMPORTAÇÕES`               | Bibliotecas padrão, OpenCV, GUI (Tkinter + CustomTkinter), Matplotlib       |
| 2      | `CONSTANTES GLOBAIS`        | Dimensões da UI, comandos serial (`M1`, `M0`), presets HSV, cores de botões |
| 3.1    | `SerialManager`             | Comunicação com o Arduino via PySerial – lista, conecta, envia, fecha       |
| 3.2    | `RealTimePlotFrame`         | Classe com o gráfico Matplotlib de erro em tempo real (X/Y)                 |
| 3.2.1  | `PlotWindow`                | Janela popup que contém o gráfico                                           |
| 3.3    | `HSVSettingsFrame`          | Sliders interativos para calibrar HSV + presets e debounce                  |
| 3.4    | `SystemControlFrame`        | Botões de controlo (motores, seguimento, gráfico, ligação serial)           |
| 3.5    | `VideoHandler`              | Thread de vídeo + deteção de bola com OpenCV + envio de erros via serial    |
| 3.6    | `StatusBar`                 | Barra inferior de mensagens temporárias com feedback ao utilizador          |
| 4      | `OpenBalanceApp`            | Classe principal que monta a interface, gere eventos e ciclo da aplicação   |
| 5      | `if __name__ == "__main__"` | Ponto de entrada para correr a aplicação                                    |
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
# Valores pré-definidos para facilitar a calibração de cores comuns.
HSV_PRESETS = {
    "Vermelho": {"h_min": 0, "h_max": 10, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Laranja": {"h_min": 10, "h_max": 25, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Verde": {"h_min": 40, "h_max": 80, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 255},
    "Cinzento": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 50, "v_min": 50, "v_max": 200},
    "Azul": {"h_min": 90, "h_max": 130, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 255},
    "Branco": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 30, "v_min": 200, "v_max": 255}
}

# --- Estilos dos Botões de Presets ---
# Define a aparência de cada botão de preset para uma identificação visual rápida.
BUTTON_COLORS = {
    "Vermelho": {"fg": "#FF0000", "hover": "#CC0000", "text": "white"},
    "Laranja": {"fg": "#FFA500", "hover": "#CC8400", "text": "white"},
    "Verde": {"fg": "#00B200", "hover": "#008000", "text": "white"},
    "Cinzento": {"fg": "#808080", "hover": "#666666", "text": "white"},
    "Azul": {"fg": "#0000FF", "hover": "#0000CC", "text": "white"},
    "Branco": {"fg": "#FFFFFF", "hover": "#DDDDDD", "text": "black"}
}

# --- Paleta de Cores da UI ---
# Cores semânticas para botões de ação (sucesso, perigo, etc.)
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
    """
    Classe responsável por toda a comunicação serial com o hardware (Arduino).
    Abstrai as complexidades da biblioteca PySerial numa interface simples.
    """
    def __init__(self):
        """Inicializa o gestor sem nenhuma conexão ativa."""
        self.serial_conn = None

    def list_ports(self):
        """Retorna uma lista de todas as portas seriais COM disponíveis no sistema."""
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        """
        Tenta estabelecer uma conexão com a porta serial especificada.
        Fecha qualquer conexão anterior antes de abrir uma nova.
        Retorna True em caso de sucesso, False em caso de erro.
        """
        try:
            # Garante que qualquer conexão antiga seja fechada antes de abrir uma nova.
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            # Cria a nova instância de conexão serial. Timeout de 1s para não bloquear indefinidamente.
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            return True
        except Exception as e:
            # Em caso de falha (ex: porta em uso, dispositivo não responde), anula a conexão e reporta o erro.
            self.serial_conn = None
            print(f"Erro ao conectar na porta serial: {e}")
            return False

    def disconnect(self):
        """Fecha a conexão serial se estiver ativa."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.serial_conn = None

    def send(self, data_str):
        """
        Envia uma string de dados para o dispositivo serial, se a conexão estiver ativa.
        A string é codificada em 'utf-8' antes de ser enviada.
        """
        if not (self.serial_conn and self.serial_conn.is_open):
            return  # Não faz nada se não estiver conectado.
        try:
            self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException:
            # Se ocorrer um erro durante o envio (ex: dispositivo desligado), encerra a conexão.
            self.disconnect()


# -----------------------------------------------------------------------------
# 3.2. MÓDULO DE VISUALIZAÇÃO GRÁFICA (GRÁFICO DE ERRO)
# -----------------------------------------------------------------------------
class RealTimePlotFrame(ctk.CTkFrame):
    """
    Frame que contém o widget do gráfico Matplotlib para visualização de dados em tempo real.
    Esta classe gere a criação, estilização e atualização do gráfico.
    """
    def __init__(self, parent, max_points=200, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)

        # 'deque' é uma lista otimizada para adicionar/remover elementos das pontas.
        # 'maxlen' garante que a lista nunca exceda um tamanho máximo, descartando dados antigos.
        self.data_x = deque(maxlen=max_points)
        self.data_y = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)
        self.start_time = time.time()  # Ponto de referência para o eixo do tempo (0s).

        # Cria a figura e os eixos do Matplotlib.
        self.fig, self.ax = plt.subplots(figsize=(5, 3))

        # Estilização do gráfico para se integrar com o tema escuro do CustomTkinter.
        self.fig.patch.set_facecolor('#2B2B2B')
        self.ax.set_facecolor('#2B2B2B')
        self.ax.tick_params(axis='x', colors='white')
        self.ax.tick_params(axis='y', colors='white')
        [spine.set_color('white') for spine in self.ax.spines.values()]
        self.ax.yaxis.label.set_color('white')
        self.ax.xaxis.label.set_color('white')
        self.ax.title.set_color('white')

        # Cria as linhas do gráfico (inicialmente vazias) para os erros X e Y.
        self.line_x, = self.ax.plot([], [], label="Erro X", color='#E53935')  # Vermelho
        self.line_y, = self.ax.plot([], [], label="Erro Y", color='#1E88E5')  # Azul

        # Configura a legenda e os títulos dos eixos.
        legend = self.ax.legend(loc="upper right", facecolor='#333333', edgecolor='white')
        for text in legend.get_texts():
            text.set_color("white")
        
        self.ax.set_ylim(-350, 350)  # Limite vertical fixo para os erros em pixels.
        self.ax.set_xlim(0, 10)     # Janela de tempo inicial de 10 segundos.
        self.ax.set_title("Erro em Tempo Real (pixels)")
        self.ax.set_xlabel("Tempo (s)")
        self.ax.set_ylabel("Erro (pixels)")

        # A classe FigureCanvasTkAgg é a "ponte" que permite embutir um gráfico Matplotlib num widget Tkinter.
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(fill="both", expand=True)

        # Inicia o loop de atualização do gráfico.
        self._update_plot()

    def add_data_point(self, erro_x, erro_y):
        """Adiciona um novo ponto de dados (erro_x, erro_y) às listas (deques)."""
        now = time.time() - self.start_time
        self.timestamps.append(now)
        self.data_x.append(erro_x)
        self.data_y.append(erro_y)

    def _update_plot(self):
        """
        Função de atualização do gráfico, chamada periodicamente.
        Redesenha as linhas com os novos dados e ajusta os limites do eixo do tempo.
        """
        if self.timestamps:
            # Atualiza os dados das linhas do gráfico.
            self.line_x.set_data(self.timestamps, self.data_x)
            self.line_y.set_data(self.timestamps, self.data_y)
            # Cria um efeito de "janela deslizante" de 10 segundos no eixo do tempo.
            current_time = self.timestamps[-1]
            self.ax.set_xlim(max(0, current_time - 10), current_time + 1)
        
        # Recalcula os limites dos dados e redesenha o canvas.
        self.ax.relim()
        self.ax.autoscale_view(True, True, True)
        self.ax.set_ylim(-350, 350) # Garante que o eixo Y permaneça fixo.
        self.canvas.draw()
        
        # Agenda a próxima chamada a esta função para daqui a 100ms (10 FPS de atualização do gráfico).
        self.after(100, self._update_plot)


class PlotWindow(ctk.CTkToplevel):
    """
    Janela pop-up (Toplevel) que contém o frame do gráfico.
    Isto permite que o gráfico seja exibido ou escondido sem afetar a janela principal.
    """
    def __init__(self, master, *args, **kwargs):
        super().__init__(master, *args, **kwargs)
        self.title("Gráfico de Erro em Tempo Real")
        self.geometry("700x450")
        
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.plot_frame = RealTimePlotFrame(self)
        self.plot_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Otimização de performance: em vez de destruir a janela ao fechar, apenas a escondemos.
        # Isto evita ter de recriar o complexo objeto do gráfico cada vez que a janela é aberta.
        self.protocol("WM_DELETE_WINDOW", self.hide_window)

    def hide_window(self):
        """Esconde a janela em vez de a destruir."""
        self.withdraw()


# -----------------------------------------------------------------------------
# 3.3. MÓDULO DE INTERFACE: PAINEL DE CONFIGURAÇÕES HSV
# -----------------------------------------------------------------------------
class HSVSettingsFrame(ctk.CTkFrame):
    """Frame para o ajuste dos parâmetros de deteção de cor (HSV)."""
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
        
        # Variável para controlo do "debounce" dos sliders.
        self._debounce_job = None

    def _on_slider_change(self, _=None):
        """
        Chamado sempre que um slider é movido. Atualiza o valor na label e agenda a atualização real.
        """
        for slider in self.hsv_sliders: slider.val_var.set(f"{int(slider.get())}")
        self._schedule_hsv_update()

    def _schedule_hsv_update(self):
        """
        Implementa a técnica de "debounce": se o utilizador mover o slider rapidamente,
        a atualização só é enviada para o VideoHandler 150ms depois de ele parar.
        Isto evita sobrecarregar a thread de vídeo com atualizações desnecessárias.
        """
        if self._debounce_job: self.after_cancel(self._debounce_job) # Cancela a tarefa agendada anterior.
        self._debounce_job = self.after(150, self._perform_hsv_update) # Agenda a nova tarefa.

    def _perform_hsv_update(self):
        """Executa a atualização real dos valores HSV no VideoHandler."""
        self._debounce_job = None
        if self.video_handler: self.video_handler.update_hsv_from_frame()

    def _apply_preset(self, name):
        """Aplica um conjunto de valores HSV pré-definidos."""
        preset = HSV_PRESETS.get(name)
        if not preset: return
        vals = [preset[k] for k in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max"]]
        for i, val in enumerate(vals): self.hsv_sliders[i].set(val); self.hsv_values[i].set(str(val))
        if self.video_handler: self._perform_hsv_update(); self.video_handler.update_once()

    def _on_toggle_mask(self):
        """Informa o VideoHandler se deve mostrar a imagem normal ou a máscara binária."""
        if self.video_handler: self.video_handler.set_show_mask(self.show_mask_var.get())

    def get_hsv_bounds(self):
        """Retorna os limites HSV inferior e superior a partir dos valores dos sliders."""
        try: values = [int(v.get()) for v in self.hsv_values]
        except ValueError: values = [0, 255, 0, 255, 0, 255] # Valores de fallback
        return np.array([values[0], values[2], values[4]]), np.array([values[1], values[3], values[5]])

    def load_hsv(self, hsv_dict):
        """Carrega e aplica valores HSV a partir de um dicionário (ex: de um ficheiro de config)."""
        vals = [hsv_dict.get(k, 0) for k in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max"]]
        for i, val in enumerate(vals): self.hsv_sliders[i].set(val); self.hsv_values[i].set(str(val))

    def save_hsv(self):
        """Retorna um dicionário com os valores HSV atuais, para serem salvos em ficheiro."""
        lower, upper = self.get_hsv_bounds()
        return {"h_min": int(lower[0]), "h_max": int(upper[0]), "s_min": int(lower[1]), "s_max": int(upper[1]), "v_min": int(lower[2]), "v_max": int(upper[2])}


# -----------------------------------------------------------------------------
# 3.4. MÓDULO DE INTERFACE: PAINEL DE CONTROLO DO SISTEMA
# -----------------------------------------------------------------------------
class SystemControlFrame(ctk.CTkFrame):
    """Frame de controlo para motores, comunicação serial e visualizações."""
    def __init__(self, parent, serial_manager: SerialManager, app_ref, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.serial_manager, self.app = serial_manager, app_ref
        self.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(self, text="Controlo do Sistema", font=("Arial", 20, "bold")).grid(row=0, column=0, padx=10, pady=(10, 5))
        
        # Grupo de Botões de Controlo Principal
        motor_group = ctk.CTkFrame(self, fg_color="transparent"); motor_group.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        motor_group.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkButton(motor_group, text="Ligar Motores", command=self._ligar_motores, **COLOR_SUCCESS).grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Desligar Motores", command=self._desligar_motores, **COLOR_DANGER).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.btn_ligar_seguimento = ctk.CTkButton(motor_group, text="Ligar Seguimento", command=self._toggle_seguimento, **COLOR_SECONDARY)
        self.btn_ligar_seguimento.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Reset Centro", command=self._reset_centro_manual, fg_color="#AA00AA", hover_color="#880088").grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Ver Gráfico", command=self.app._show_plot_window, **COLOR_INFO).grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # Grupo de Controlo da Comunicação Serial
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
        """Ativa ou desativa o envio de dados de erro para o Arduino."""
        vh = self.app.video_handler
        if not vh: return
        # Usa os métodos seguros (thread-safe) para ler e escrever a variável de estado.
        new_state = not vh.tracking_enabled_safe
        vh.set_tracking_enabled(new_state)
        
        # Atualiza a aparência do botão e mostra uma mensagem na barra de status.
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

    def get_pid_config(self): return {} # Placeholder para futuras configurações de PID
    def load_pid_config(self, pid_dict): pass # Placeholder

# -----------------------------------------------------------------------------
# 3.5. MÓDULO PRINCIPAL: GESTOR DE VÍDEO E PROCESSAMENTO DE IMAGEM
# -----------------------------------------------------------------------------
class VideoHandler:
    """
    Classe central que gere a captura de vídeo, processamento de imagem e comunicação
    com outras partes da aplicação. Opera numa thread separada para não congelar a GUI.
    """
    def __init__(self, parent, hsv_frame_ref: HSVSettingsFrame, serial_manager: SerialManager, error_queue: queue.Queue, camera_index=0):
        self.parent, self.hsv_frame_ref, self.serial_manager, self.error_queue = parent, hsv_frame_ref, serial_manager, error_queue
        self.camera_index = camera_index; self.cap = None; self.thread = None; self.running = False
        
        # Queues para comunicação thread-safe:
        # - frame_queue: Envia frames processados da thread de vídeo para a thread da GUI.
        # - error_queue: Envia dados de erro da thread de vídeo para a thread da GUI (para o gráfico).
        self.frame_queue = queue.Queue(maxsize=1)
        
        # Coordenadas do centro funcional (pode ser alterado com o clique do rato).
        self.center_x, self.center_y = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2
        
        # --- Variáveis partilhadas entre threads ---
        # Lock para garantir acesso atómico (seguro) às variáveis partilhadas.
        self.shared_lock = threading.Lock()
        self.hsv_lower, self.hsv_upper = np.array([0,0,0]), np.array([255,255,255])
        self.show_mask = False
        self._tracking_enabled_internal = False # O '_' indica que esta variável não deve ser acedida diretamente de fora.
        
        # Debouncing do envio serial para não sobrecarregar o Arduino
        self.last_send_time = 0

        # Criação do canvas onde o vídeo será desenhado.
        self.canvas = ctk.CTkCanvas(parent, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, bg="gray20", highlightthickness=0)
        self.canvas.grid(row=2, column=0, padx=10, pady=10)
        self.canvas.bind("<Button-1>", self._on_mouse_click)
        
        # Inicializa a câmera e a thread de vídeo.
        self.update_hsv_from_frame()
        self._initialize_camera()

    # --- Métodos Seguros para Acesso a Variáveis Partilhadas ---
    # Estes métodos usam o 'lock' para garantir que as operações de leitura e escrita
    # em variáveis partilhadas sejam atómicas, prevenindo 'race conditions'.
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
        """Propriedade 'getter' segura para ler o estado do tracking."""
        with self.shared_lock: return self._tracking_enabled_internal

    # --- Métodos de Controlo do Centro ---
    def reset_center(self):
        """Redefine o centro funcional para o meio da imagem de forma segura."""
        with self.shared_lock: self.center_x, self.center_y = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2

    def _on_mouse_click(self, event):
        """Atualiza o centro de referência com base na posição do clique do rato."""
        with self.shared_lock: self.center_x, self.center_y = event.x, event.y

    # --- Métodos de Controlo da Thread e Câmera ---
    def _initialize_camera(self):
        """
        Inicializa ou reinicializa a captura de vídeo. Fecha a thread e a câmera antigas
        antes de criar novas.
        """
        if self.cap and self.cap.isOpened():
            self.running = False # Sinaliza à thread para parar.
            if self.thread and self.thread.is_alive(): self.thread.join(timeout=1) # Espera que a thread termine.
            self.cap.release()
        
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW) # CAP_DSHOW melhora a compatibilidade no Windows.
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        
        if not self.cap.isOpened():
            messagebox.showerror("Erro de Vídeo", f"Não foi possível aceder à webcam {self.camera_index}.")
        else:
            self.running = True
            # A thread é 'daemon' para que termine automaticamente quando a aplicação principal fechar.
            self.thread = threading.Thread(target=self._video_loop, daemon=True)
            self.thread.start()
            self._update_canvas_from_queue() # Inicia o loop de atualização do canvas na thread da GUI.

    def _video_loop(self):
        """
        O coração da thread de vídeo. Captura, processa e coloca frames na queue continuamente.
        """
        while self.running and self.cap and self.cap.isOpened():
            ret, frame_bgr = self.cap.read()
            if not ret: break # Fim do vídeo ou erro na câmera.
            
            try:
                # Processa o frame e tenta colocá-lo na queue de frames.
                # 'put_nowait' não bloqueia. Se a queue estiver cheia, lança uma exceção.
                self.frame_queue.put_nowait(self._process_frame_for_display(frame_bgr))
            except queue.Full:
                # Se a queue estiver cheia, significa que a GUI está atrasada.
                # Simplesmente ignoramos este frame para não bloquear a captura de vídeo.
                pass
        self.running = False

    def _process_frame_for_display(self, frame_bgr):
        """
        Pipeline de processamento de um único frame de vídeo.
        Aplica filtros, deteta a bola, calcula o erro e desenha informações na imagem.
        """
        height, width, _ = frame_bgr.shape
        # Acesso seguro às variáveis partilhadas no início do processamento.
        with self.shared_lock:
            center_x, center_y = self.center_x, self.center_y
            lower, upper, show_mask, tracking = self.hsv_lower, self.hsv_upper, self.show_mask, self._tracking_enabled_internal
        
        # 1. Conversão de cor e criação da máscara
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        # 2. Redução de ruído na máscara (remove pequenos pontos brancos)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        
        # Escolhe a imagem a ser exibida (original ou máscara).
        display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB) if show_mask else cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        
        # 3. Deteção de contornos e cálculo do erro
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        erro_x, erro_y = 0, 0
        if contours:
            c = max(contours, key=cv2.contourArea) # Assume que o maior contorno é a bola.
            if cv2.contourArea(c) > 500: # Ignora contornos muito pequenos.
                (x, y), radius = cv2.minEnclosingCircle(c)
                ball_center = (int(x), int(y))
                # Cálculo do erro: posição da bola - posição do centro. 'y' é invertido.
                erro_x, erro_y = ball_center[0] - center_x, -(ball_center[1] - center_y)

                # Se o seguimento estiver ativo, envia o erro para o Arduino (com debounce).
                if tracking and self.serial_manager.serial_conn:
                    now = time.time()
                    if now - self.last_send_time > 0.05:  # Limita o envio a 20Hz (50ms).
                        self.serial_manager.send(f"E,{erro_x},{erro_y}\n")
                        self.last_send_time = now

                # Desenha um círculo à volta da bola detetada.
                cv2.circle(display_img, ball_center, int(radius), (0, 255, 0), 2)
                cv2.circle(display_img, ball_center, 3, (255, 0, 0), -1)
                cv2.putText(display_img, f"Erro X: {erro_x:+d}, Y: {erro_y:+d}", (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        
        # 4. Envio de dados para a queue do gráfico (sempre, mesmo que o erro seja zero).
        try:
            self.error_queue.put_nowait((erro_x, erro_y))
        except queue.Full:
            pass # Ignora se a queue do gráfico estiver cheia.

        # 5. Desenho dos marcadores de centro (CORES ORIGINAIS RESTAURADAS)
        # Cruz vermelha e espessa no centro físico da imagem.
        cv2.line(display_img, (width//2 - 10, height//2), (width//2 + 10, height//2), (255, 0, 0), 4)
        cv2.line(display_img, (width//2, height//2 - 10), (width//2, height//2 + 10), (255, 0, 0), 4)
        # Círculo azul no centro funcional definido pelo clique.
        cv2.circle(display_img, (center_x, center_y), 5, (0, 0, 255), -1)

        return display_img

    def _update_canvas_from_queue(self):
        """
        Executado na thread da GUI. Retira um frame da queue e atualiza o canvas.
        """
        try:
            frame = self.frame_queue.get_nowait()
            img_pil = Image.fromarray(frame); img_tk = ImageTk.PhotoImage(image=img_pil)
            self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
            # Guarda uma referência à imagem para evitar que seja apagada pelo garbage collector do Python.
            self.canvas.image = img_tk
        except queue.Empty:
            pass # Normal se a thread de vídeo estiver um pouco mais lenta que a GUI.
        finally:
            # Agenda a próxima verificação, mantendo o loop de renderização da GUI.
            if self.running: self.canvas.after(33, self._update_canvas_from_queue) # ~30 FPS

    def change_camera(self, new_index):
        """Muda o índice da câmera e reinicia a captura."""
        try: self.camera_index = int(new_index)
        except ValueError: messagebox.showerror("Câmera", f"Índice inválido: {new_index}"); return
        self._initialize_camera()

    def update_once(self):
        """Força o processamento e a exibição de um único frame, útil após carregar configs."""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                display_img = self._process_frame_for_display(frame)
                img_pil = Image.fromarray(display_img); img_tk = ImageTk.PhotoImage(image=img_pil)
                self.canvas.create_image(0, 0, anchor="nw", image=img_tk); self.canvas.image = img_tk

    def stop(self):
        """Para a thread de vídeo e liberta os recursos da câmera de forma segura."""
        self.running = False
        if self.thread and self.thread.is_alive(): self.thread.join(timeout=1)
        if self.cap: self.cap.release()

# -----------------------------------------------------------------------------
# 3.6. MÓDULO DE INTERFACE: BARRA DE STATUS
# -----------------------------------------------------------------------------
class StatusBar(ctk.CTkFrame):
    """Uma barra de status simples na parte inferior da janela para exibir mensagens."""
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, corner_radius=0, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w", font=("Arial", 16)); self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None
    def show_message(self, message, duration_ms=5000, is_error=False):
        if self._job: self.after_cancel(self._job)         # Cancela mensagem anterior
        text_color = "#FF5555" if is_error else "#FFD700"  # amarelo ouro
        self.label.configure(text=message, text_color=text_color)
        self._job = self.after(duration_ms, lambda: self.label.configure(text="")) # Agenda para limpar a mensagem.

# =============================================================================
# 4. CLASSE PRINCIPAL DA APLICAÇÃO
# =============================================================================
class OpenBalanceApp:
    """
    Classe orquestradora principal. Monta todos os módulos da interface,
    gere os gestores (serial, vídeo) e o ciclo de vida da aplicação.
    """
    def __init__(self):
        # Configuração inicial da aparência da aplicação.
        ctk.set_appearance_mode("dark"); ctk.set_default_color_theme("dark-blue")
        self.app = ctk.CTk(); self.app.title("OpenBalance Dashboard"); self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.app.grid_rowconfigure(0, weight=1); self.app.grid_rowconfigure(1, weight=0); self.app.grid_columnconfigure(0, weight=1)
        self.app.grid_columnconfigure(1, weight=3); self.app.grid_columnconfigure(2, weight=1)
        
        # Instanciação dos gestores e variáveis de estado.
        self.serial_manager = SerialManager(); self.video_handler = None
        self.plot_window = None # Referência à janela do gráfico (criada sob demanda).
        self.error_queue = queue.Queue() # Queue para receber dados de erro do VideoHandler.

        # Construção da UI.
        self._create_menu(); self._create_frames(); self._create_status_bar()
        
        # Inicia o "consumidor" da queue de erros na thread da GUI.
        self._process_error_queue() 
        
        # Define o procedimento de encerramento limpo da aplicação.
        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _create_menu(self):
        menu = tk.Menu(self.app)

        # Menu Arquivo
        file_menu = tk.Menu(menu, tearoff=0)
        file_menu.add_command(label="Salvar Configuração", command=self._salvar_config)
        file_menu.add_command(label="Carregar Configuração", command=self._carregar_config)
        file_menu.add_separator()
        file_menu.add_command(label="Sair", command=self._on_close)
        menu.add_cascade(label="Arquivo", menu=file_menu)

        # Menu Ajuda
        ajuda_menu = tk.Menu(menu, tearoff=0)
        ajuda_menu.add_command(label="Sobre o Projeto", command=self._mostrar_sobre)
        ajuda_menu.add_command(label="Manual de Utilização", command=self._mostrar_manual)
        ajuda_menu.add_command(label="Créditos", command=self._mostrar_creditos)
        menu.add_cascade(label="ℹ️ Ajuda", menu=ajuda_menu)

        self.app.config(menu=menu)

    def _create_status_bar(self):
        """Cria e posiciona a barra de status."""
        self.status_bar = StatusBar(self.app); self.status_bar.grid(row=1, column=0, columnspan=3, sticky="sew")

    def _create_frames(self):
        """Cria e posiciona todos os frames principais da interface."""
        # --- Frame do Vídeo (centro) ---
        video_container = ctk.CTkFrame(self.app, corner_radius=10); video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        video_container.grid_rowconfigure(2, weight=1); video_container.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(video_container, text="Área de Visualização", font=("Arial", 22, "bold")).grid(row=0, column=0, pady=(10,5))
        cam_frame = ctk.CTkFrame(video_container, fg_color="transparent"); cam_frame.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(cam_frame, text="Câmera:").pack(side="left", padx=(0, 5))
        self.cam_var = tk.StringVar(value="0")
        ctk.CTkOptionMenu(cam_frame, values=["0", "1", "2", "3"], variable=self.cam_var, command=self._on_camera_change).pack(side="left")
        
        # --- Frames Laterais (configurações e controlo) ---
        self.hsv_frame = HSVSettingsFrame(self.app, video_handler=None); self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)
        self.control_frame = SystemControlFrame(self.app, self.serial_manager, app_ref=self); self.control_frame.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)
        
        # Instancia o VideoHandler, passando todas as dependências necessárias (incluindo a error_queue).
        self.video_handler = VideoHandler(video_container, self.hsv_frame, self.serial_manager, self.error_queue, int(self.cam_var.get()))
        self.hsv_frame.video_handler = self.video_handler # Completa a referência circular.

    def _on_camera_change(self, index):
        """Callback para quando uma nova câmera é selecionada no menu dropdown."""
        if self.video_handler: self.video_handler.change_camera(index)
    
    def _salvar_config(self):
        """Abre um diálogo para salvar as configurações HSV num ficheiro JSON."""
        config = {"hsv": self.hsv_frame.save_hsv(), "pid": self.control_frame.get_pid_config()}
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=CONFIG_FILE_TYPES)
        if not filepath: return
        try:
            with open(filepath, 'w', encoding='utf-8') as f: json.dump(config, f, indent=4)
            self.status_bar.show_message(f"Configuração salva em {filepath}")
        except Exception as e: self.status_bar.show_message(f"Erro ao salvar: {e}", is_error=True)

    def _carregar_config(self):
        """Abre um diálogo para carregar configurações de um ficheiro JSON."""
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
        messagebox.showinfo("Sobre o Projeto", "OpenBalance Dashboard\nVersão 1.0\nControlo PID com visão computacional.\nJoão Pavão – Universidade dos Açores")

    def _mostrar_manual(self):
        messagebox.showinfo("Manual de Utilização", "1. Ligue o Arduino e selecione a porta COM\n2. Clique em 'Ligar Motores'\n3. Ajuste os parâmetros HSV\n4. Clique em 'Ligar Seguimento' para iniciar o controlo\n5. Use 'Ver Gráfico' para análise de erros.")

    def _mostrar_creditos(self):
        messagebox.showinfo("Créditos", "Desenvolvido por João Pavão\nSupervisão: PRIA - Universidade dos Açores\nLicença: MIT License")


    def _show_plot_window(self):
        """
        Cria (se necessário) e exibe a janela do gráfico.
        Usa a técnica de "lazy instantiation" para performance.
        """
        # Cria a janela apenas na primeira vez que o botão é clicado.
        if self.plot_window is None:
            self.plot_window = PlotWindow(self.app)
            self.plot_window.withdraw() # Começa escondida para não piscar no ecrã.
        
        # Se a janela estiver escondida, mostra-a.
        if not self.plot_window.winfo_viewable():
             self.plot_window.deiconify()
        self.plot_window.lift() # Traz a janela para a frente de todas as outras.

    def _process_error_queue(self):
        """
        O "consumidor" do padrão Produtor-Consumidor. Executa na thread da GUI.
        Verifica a queue de erro periodicamente e, se houver dados, atualiza o gráfico.
        """
        try:
            # Esvazia a queue de todos os dados pendentes para manter o gráfico reativo.
            while not self.error_queue.empty():
                erro_x, erro_y = self.error_queue.get_nowait()
                # Só tenta atualizar o gráfico se a janela existir e estiver visível.
                if self.plot_window and self.plot_window.winfo_viewable():
                    self.plot_window.plot_frame.add_data_point(erro_x, erro_y)
        except queue.Empty:
            pass
        finally:
            # Agenda a próxima verificação para daqui a 100ms.
            self.app.after(100, self._process_error_queue)

    def _on_close(self):
        """
        Procedimento de encerramento limpo da aplicação.
        É crucial parar as threads e libertar recursos (câmera, serial) antes de fechar.
        """
        if self.video_handler: self.video_handler.stop()
        self.serial_manager.disconnect()
        self.app.destroy()

    def run(self):
        """Inicia o loop principal da aplicação GUI."""
        self.app.mainloop()

# =============================================================================
# 5. PONTO DE ENTRADA DO SCRIPT
# =============================================================================
if __name__ == "__main__":
    # Cria uma instância da aplicação principal e executa-a.
    app = OpenBalanceApp()
    app.run()