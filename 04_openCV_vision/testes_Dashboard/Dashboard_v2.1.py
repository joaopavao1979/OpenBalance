"""
OpenBalance Dashboard (Versão 3.1 - Edição Académica)
======================================================

Descrição:
    Dashboard para o controlo em tempo real de uma plataforma de equilíbrio de bola.
    Este sistema integra visão computacional (OpenCV) para deteção de objetos,
    um controlador PID para estabilização, e uma interface gráfica (GUI)
    moderna para interação do utilizador.

Autor:
    João Pavão (Conceito Original)
    Refatorado e Comentado por Professor AI (para fins didáticos)

Arquitetura e Conceitos-Chave:
    1.  **Separação de Responsabilidades (SoC):** O código está modularizado em
        classes, cada uma com uma responsabilidade única (Serial, Vídeo, UI).
        Isto segue princípios de design como o Single Responsibility Principle (SRP).
    2.  **Multithreading:** A captura de vídeo, uma operação de I/O bloqueante,
        é executada numa thread separada para não congelar a interface gráfica.
        Isto é essencial para qualquer aplicação responsiva que lida com tarefas demoradas.
    3.  **Programação Orientada a Eventos:** A GUI funciona num loop de eventos
        (o `mainloop` do CustomTkinter). Ações do utilizador (cliques, movimentos
        de slider) disparam "eventos", que por sua vez invocam funções ("callbacks").
    4.  **Gestão de Estado Explícita:** Variáveis como `tracking_enabled` controlam
        o comportamento do sistema de forma clara e deliberada, em vez de depender
        de estados implícitos.

Uso:
    Este código é de natureza educacional e pode ser livremente utilizado e
    modificado sob a licença MIT.
"""

# -----------------------------------------------------------------------------
# Módulo 1: Importações e Configurações Globais
# -----------------------------------------------------------------------------
# Uma boa prática é agrupar as importações por tipo: bibliotecas padrão,
# bibliotecas de terceiros, e módulos locais (se existirem).

# Bibliotecas Padrão do Python
import json         # Para serializar/desserializar configurações para/de ficheiros JSON.
import threading    # Fundamental para executar a captura de vídeo em paralelo com a GUI.
import csv          # Para exportar dados, como os parâmetros do PID, para um formato tabular.

# Bibliotecas de Terceiros (instaladas via 'pip')
import cv2                          # OpenCV: A nossa "visão" para processamento de imagem e vídeo.
import numpy as np                  # NumPy: Essencial para manipulação de arrays (imagens no OpenCV são arrays NumPy).
import customtkinter as ctk         # Uma versão moderna e personalizável da biblioteca Tkinter para a GUI.
import serial                       # PySerial: Para a comunicação via porta serial com o microcontrolador (ex: Arduino).
import serial.tools.list_ports      # Utilitário do PySerial para detetar as portas seriais disponíveis no sistema.
from PIL import Image, ImageTk      # Pillow (PIL Fork): Ponte entre o formato de imagem do OpenCV e o do Tkinter.
import tkinter as tk                # Tkinter: Usado para funcionalidades base da GUI como menus e diálogos de ficheiros.
from tkinter import filedialog, messagebox

# -----------------------------------------------------------------------------
# Módulo 2: Definição de Constantes
# -----------------------------------------------------------------------------
# O uso de constantes em vez de "números mágicos" ou "strings mágicas"
# no código é uma prática de engenharia de software crucial. Torna o código
# mais legível, fácil de manter e menos propenso a erros. Se uma dimensão
# ou um comando mudar, só precisamos de o alterar num único local.

# --- Configurações da Janela e Vídeo ---
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480

# --- Configurações de Comunicação ---
DEFAULT_BAUDRATE = 9600

# --- Tipos de Ficheiro para Diálogos ---
CONFIG_FILE_TYPES = [("JSON Files", "*.json")]
CSV_FILE_TYPES = [("CSV Files", "*.csv")]

# --- Constantes de Protocolo Serial ---
# Centralizamos os comandos enviados para o Arduino aqui.
CMD_MOTORS_ON = "M1\n"
CMD_MOTORS_OFF = "M0\n"
CMD_ERROR_PREFIX = "E"  # Prefixo para enviar coordenadas de erro (ex: "E,-10,25\n")
CMD_PID_PREFIX = "PID"  # Prefixo para enviar parâmetros PID

# --- Paleta de Cores da UI ---
# Definir cores como constantes permite uma identidade visual consistente
# e facilita a alteração do tema da aplicação.
COLOR_SUCCESS = {"fg_color": "#2E7D32", "hover_color": "#1B5E20"}   # Verde para ações positivas
COLOR_DANGER = {"fg_color": "#C62828", "hover_color": "#B71C1C"}    # Vermelho para ações negativas/perigosas
COLOR_NEUTRAL = {"fg_color": "#1E88E5", "hover_color": "#1565C0"}   # Azul para ações padrão
COLOR_SECONDARY = {"fg_color": "#616161", "hover_color": "#424242"} # Cinzento para ações secundárias

# --- Presets de Deteção de Cor (HSV) ---
# Usamos um dicionário para mapear nomes de cores legíveis a valores HSV.
# Isto torna a adição de novos presets trivial.
HSV_PRESETS = {
    "Vermelho": {"h_min": 0, "h_max": 10, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Laranja": {"h_min": 10, "h_max": 25, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Verde": {"h_min": 40, "h_max": 80, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "Cinzento": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 50, "v_min": 50, "v_max": 200},
    "Azul": {"h_min": 90, "h_max": 130, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 255},
    "Branco": {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 30, "v_min": 200, "v_max": 255}
}

# Cores correspondentes para os botões de preset.
BUTTON_COLORS = {
    "Vermelho": {"fg": "#FF0000", "hover": "#CC0000", "text": "white"},
    "Laranja": {"fg": "#FFA500", "hover": "#CC8400", "text": "white"},
    "Verde": {"fg": "#00B200", "hover": "#008000", "text": "white"},
    "Cinzento": {"fg": "#808080", "hover": "#666666", "text": "white"},
    "Azul": {"fg": "#0000FF", "hover": "#0000CC", "text": "white"},
    "Branco": {"fg": "#FFFFFF", "hover": "#DDDDDD", "text": "black"}
}


# -----------------------------------------------------------------------------
# Classe 1: SerialManager
# Função: Abstrair a complexidade da comunicação serial.
# -----------------------------------------------------------------------------
class SerialManager:
    """
    Esta classe encapsula toda a lógica de comunicação serial.
    O resto da aplicação não precisa de saber sobre 'pyserial', apenas
    interage com os métodos simples desta classe (ex: .connect(), .send()).
    Isto é um exemplo de "Abstração".
    """
    def __init__(self):
        """Construtor: Inicializa a conexão serial como Nula."""
        self.serial_conn = None

    def list_ports(self):
        """Retorna uma lista de todas as portas seriais COM disponíveis."""
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        """
        Tenta estabelecer uma conexão na porta e baudrate especificados.
        Retorna True em caso de sucesso e False em caso de falha.
        Notem que esta classe já não mostra pop-ups. A responsabilidade
        de notificar o utilizador foi movida para a camada de UI (a classe `StatusBar`).
        """
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            return True
        except Exception as e:
            # Em caso de erro, garantimos que a conexão é Nula e reportamos o erro no terminal.
            self.serial_conn = None
            print(f"Erro ao conectar na porta serial: {e}")
            return False

    def disconnect(self):
        """Encerra a conexão serial, se estiver ativa."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.serial_conn = None

    def send(self, data_str):
        """Envia uma string de dados para o Arduino."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return  # Não faz nada se não estiver conectado.

        try:
            # Os dados devem ser codificados em bytes (utf-8 é o padrão) antes de serem enviados.
            self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException:
            # Se a porta for fisicamente desconectada, um erro ocorrerá.
            # Tratamos disso limpando a nossa conexão.
            self.disconnect()


# -----------------------------------------------------------------------------
# Classe 2: HSVSettingsFrame
# Função: Fornecer uma interface para o utilizador ajustar os parâmetros de deteção de cor.
# -----------------------------------------------------------------------------
class HSVSettingsFrame(ctk.CTkFrame):
    """
    Este frame contém todos os widgets (sliders, botões) para a
    configuração da máscara HSV (Hue, Saturation, Value).
    """
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.video_handler = video_handler
        self.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10, 15), sticky="n")

        self.hsv_sliders = []
        self.hsv_values = []
        labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
        for idx, label in enumerate(labels):
            ctk.CTkLabel(self, text=label).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(value="0")
            self.hsv_values.append(val_var)

            slider = ctk.CTkSlider(self, from_=0, to=255, number_of_steps=255, command=lambda val, var=val_var: var.set(f"{int(val)}"))
            slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew")
            self.hsv_sliders.append(slider)

            ctk.CTkLabel(self, textvariable=val_var).grid(row=2 + 2*idx, column=0, padx=(270,10), sticky="e")

        self.show_mask_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(self, text="Mostrar Máscara", variable=self.show_mask_var, command=self._on_toggle_mask).grid(row=13, column=0, pady=(15,10), padx=10, sticky="w")

        # Os presets são criados dinamicamente a partir do nosso dicionário de constantes.
        # Isto torna o código mais "Data-Driven".
        preset_frame = ctk.CTkFrame(self, fg_color="transparent")
        preset_frame.grid(row=14, column=0, sticky="ew", padx=10)
        preset_frame.grid_columnconfigure((0, 1), weight=1)
        
        for i, (color_name, colors) in enumerate(BUTTON_COLORS.items()):
            row, col = divmod(i, 2)
            ctk.CTkButton(
                preset_frame,
                text=color_name,
                fg_color=colors["fg"],
                hover_color=colors["hover"],
                text_color=colors["text"],
                command=lambda cn=color_name: self._apply_preset(cn)
            ).grid(row=row, column=col, pady=3, padx=3, sticky="ew")

    def _apply_preset(self, color_name):
        """Callback: Aplica os valores HSV de um preset aos sliders."""
        preset = HSV_PRESETS.get(color_name)
        if not preset: return
        sliders_vals = [preset["h_min"], preset["h_max"], preset["s_min"], preset["s_max"], preset["v_min"], preset["v_max"]]
        for i, val in enumerate(sliders_vals):
            self.hsv_sliders[i].set(val)
            self.hsv_values[i].set(str(val))
        if self.video_handler: self.video_handler.update_once()

    def _on_toggle_mask(self):
        """Callback: Informa o VideoHandler para mostrar/ocultar a máscara."""
        if self.video_handler: self.video_handler.show_mask = self.show_mask_var.get()

    def get_hsv_bounds(self):
        """Retorna os limites HSV inferior e superior como arrays NumPy."""
        try:
            vals = [int(v.get()) for v in self.hsv_values]
        except ValueError: vals = [0, 255, 0, 255, 0, 255] # Valores de fallback
        return np.array([vals[0], vals[2], vals[4]]), np.array([vals[1], vals[3], vals[5]])

    def load_hsv(self, hsv_dict):
        """Carrega valores HSV de um dicionário (ex: de um ficheiro JSON)."""
        vals = [hsv_dict.get("h_min", 0), hsv_dict.get("h_max", 255), hsv_dict.get("s_min", 0), hsv_dict.get("s_max", 255), hsv_dict.get("v_min", 0), hsv_dict.get("v_max", 255)]
        for i, val in enumerate(vals):
            self.hsv_sliders[i].set(val)
            self.hsv_values[i].set(str(val))

    def save_hsv(self):
        """Retorna os valores HSV atuais como um dicionário para serem salvos."""
        lower, upper = self.get_hsv_bounds()
        return {"h_min": int(lower[0]), "h_max": int(upper[0]), "s_min": int(lower[1]), "s_max": int(upper[1]), "v_min": int(lower[2]), "v_max": int(upper[2])}


# -----------------------------------------------------------------------------
# Classe 3: PIDControllerFrame
# Função: Fornecer uma interface para o utilizador configurar o PID e controlar o sistema.
# -----------------------------------------------------------------------------
class PIDControllerFrame(ctk.CTkFrame):
    """
    Este frame foi significativamente refatorado para uma melhor experiência do utilizador (UX).
    Os controlos são agrupados logicamente em sub-frames, tornando a interface
    menos intimidante e mais intuitiva.
    """
    def __init__(self, parent, serial_manager: SerialManager, app_ref, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.serial_manager = serial_manager
        self.app = app_ref  # Uma referência à classe principal da aplicação.
        self.grid_columnconfigure(0, weight=1)

        # --- GRUPO 1: Ganhos do Controlador PID ---
        # A combinação de slider + entry oferece o melhor de dois mundos:
        # ajuste rápido e visual com o slider, e ajuste preciso com a entrada de texto.
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
            
            # Este é um ponto crucial: a sincronização dos widgets.
            # Usamos `lambda` para capturar o valor atual de `param` no loop. Sem isto,
            # todas as lambdas usariam o último valor de `param` ("Kd").
            def update_from_slider(value, p=param):
                entry = self.pid_params_widgets[p]['entry']
                entry.delete(0, 'end')
                entry.insert(0, f"{value:.2f}")

            def update_from_entry(event, p=param):
                try:
                    slider = self.pid_params_widgets[p]['slider']
                    entry = self.pid_params_widgets[p]['entry']
                    slider.set(float(entry.get()))
                except (ValueError, KeyError): pass
            
            slider.configure(command=update_from_slider)
            entry.bind("<Return>", update_from_entry)
            entry.bind("<FocusOut>", update_from_entry)

            self.pid_params_widgets[param] = {'slider': slider, 'entry': entry}
            self.pid_params_widgets[param]['entry'].insert(0, "0.00")

        # --- GRUPO 2: Controlo do Sistema Físico ---
        motor_group = ctk.CTkFrame(self, fg_color="transparent")
        motor_group.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        motor_group.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(motor_group, text="Controlo do Sistema", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(5, 10))

        ctk.CTkButton(motor_group, text="Ligar Motores", command=self._ligar_motores, **COLOR_SUCCESS).grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(motor_group, text="Desligar Motores", command=self._desligar_motores, **COLOR_DANGER).grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        # O botão de seguimento agora tem um estado e muda de cor/texto.
        self.btn_ligar_seguimento = ctk.CTkButton(motor_group, text="Ligar Seguimento", command=self._toggle_seguimento, **COLOR_SECONDARY)
        self.btn_ligar_seguimento.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        ctk.CTkButton(motor_group, text="Enviar PID", command=self._enviar_pid_arduino, **COLOR_NEUTRAL).grid(row=3, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # --- GRUPO 3: Comunicação Serial ---
        serial_group = ctk.CTkFrame(self, fg_color="transparent")
        serial_group.grid(row=2, column=0, padx=10, pady=10, sticky="ew")
        serial_group.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(serial_group, text="Comunicação Arduino", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=(5, 10))

        portas = self.serial_manager.list_ports()
        self.porta_var = tk.StringVar(value=portas[0] if portas else "Nenhuma porta")
        self.option_menu_porta = ctk.CTkOptionMenu(serial_group, values=portas if portas else ["Nenhuma porta"], variable=self.porta_var)
        self.option_menu_porta.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="🔄", width=30, command=self._atualizar_portas).grid(row=1, column=1, padx=5, pady=5)

        ctk.CTkButton(serial_group, text="Ligar", command=self._ligar_arduino, **COLOR_SUCCESS).grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        ctk.CTkButton(serial_group, text="Desligar", command=self._desligar_arduino, **COLOR_DANGER).grid(row=2, column=1, padx=5, pady=5, sticky="ew")

    def _toggle_seguimento(self):
        """Alterna o estado de seguimento da bola (tracking)."""
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
        self.app.status_bar.show_message("Lista de portas seriais atualizada.")

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
        """Lê os valores dos widgets e retorna-os num dicionário."""
        try:
            return {p: float(w['entry'].get()) for p, w in self.pid_params_widgets.items()}
        except (ValueError, KeyError):
            return {"kp": 0.0, "ki": 0.0, "kd": 0.0}

    def load_pid_config(self, pid_dict):
        """Carrega um dicionário de configuração para os widgets do PID."""
        for param, value in pid_dict.items():
            if param in self.pid_params_widgets:
                widgets = self.pid_params_widgets[param]
                widgets['slider'].set(value)
                widgets['entry'].delete(0, 'end')
                widgets['entry'].insert(0, f"{value:.2f}")

    def _enviar_pid_arduino(self):
        """Envia os parâmetros PID atuais para o Arduino."""
        pid_cfg = self.get_pid_config()
        cmd_str = f"{CMD_PID_PREFIX},{pid_cfg['kp']:.2f},{pid_cfg['ki']:.2f},{pid_cfg['kd']:.2f}\n"
        self.serial_manager.send(cmd_str)
        self.app.status_bar.show_message(f"Parâmetros PID enviados: Kp={pid_cfg['kp']:.2f}, Ki={pid_cfg['ki']:.2f}, Kd={pid_cfg['kd']:.2f}")


# -----------------------------------------------------------------------------
# Classe 4: StatusBar
# Função: Fornecer feedback não-disruptivo ao utilizador.
# -----------------------------------------------------------------------------
class StatusBar(ctk.CTkFrame):
    """
    Uma barra de status no rodapé da janela. É uma alternativa muito superior
    aos `messagebox` para mensagens informativas, pois não interrompe
    o fluxo de trabalho do utilizador.
    """
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w")
        self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None # Armazena o ID do evento agendado com 'after'.

    def show_message(self, message, duration_ms=4000, is_error=False):
        """Exibe uma mensagem que desaparece após 'duration_ms'."""
        if self._job:
            self.after_cancel(self._job) # Cancela a mensagem anterior, se houver.
        
        text_color = COLOR_DANGER["fg_color"] if is_error else "gray70"
        self.label.configure(text=message, text_color=text_color)
        
        # O método 'after' da Tkinter é usado para agendar a execução de uma
        # função no futuro, sem bloquear o loop da GUI.
        self._job = self.after(duration_ms, lambda: self.label.configure(text=""))


# -----------------------------------------------------------------------------
# Classe 5: VideoHandler
# Função: Gerir a câmara, processar os frames de vídeo e exibir o resultado.
# -----------------------------------------------------------------------------
class VideoHandler:
    """
    Esta é talvez a classe mais complexa. Ela executa o ciclo de vida do vídeo
    numa thread separada e realiza todo o processamento de imagem com OpenCV.
    """
    def __init__(self, parent, hsv_frame: HSVSettingsFrame, serial_manager: SerialManager, camera_index=0):
        self.parent = parent
        self.hsv_frame = hsv_frame
        self.serial_manager = serial_manager
        self.camera_index = camera_index
        
        # Estados internos da classe
        self.show_mask = False
        self.tracking_enabled = False # Flag que controla se os dados são enviados.
        self.running = False

        # Recursos que precisam de ser geridos (câmara e thread)
        self.cap = None
        self.thread = None

        # --- Widgets da UI que esta classe controla ---
        self.canvas = ctk.CTkCanvas(parent, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, bg="gray20", highlightthickness=0)
        self.canvas.grid(row=2, column=0, padx=10, pady=10, sticky="ew")

        self.status_frame = ctk.CTkFrame(parent, corner_radius=5)
        self.status_frame.grid(row=3, column=0, pady=(0, 10), padx=10, sticky="ew")
        self.status_frame.grid_columnconfigure((0, 1), weight=1)

        self.servo_x_var = ctk.StringVar(value="Servo X: 0")
        self.servo_y_var = ctk.StringVar(value="Servo Y: 0")
        self.pid_x_var = ctk.StringVar(value="PID X: 0.00")
        self.pid_y_var = ctk.StringVar(value="PID Y: 0.00")

        ctk.CTkLabel(self.status_frame, textvariable=self.servo_x_var).grid(row=0, column=0, padx=10, pady=2, sticky="w")
        ctk.CTkLabel(self.status_frame, textvariable=self.servo_y_var).grid(row=1, column=0, padx=10, pady=2, sticky="w")
        ctk.CTkLabel(self.status_frame, textvariable=self.pid_x_var).grid(row=0, column=1, padx=10, pady=2, sticky="w")
        ctk.CTkLabel(self.status_frame, textvariable=self.pid_y_var).grid(row=1, column=1, padx=10, pady=2, sticky="w")
        
        self._initialize_camera()

    def _initialize_camera(self):
        """Abre a câmara e inicia a thread de captura."""
        self.stop() # Garante que qualquer câmara ou thread anterior seja parada.

        # cv2.CAP_DSHOW é uma API específica do Windows que pode ser mais estável.
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        
        if not self.cap.isOpened():
            messagebox.showerror("Erro de Câmara", f"Não foi possível aceder à câmara (índice {self.camera_index}).")
            self.cap = None
        else:
            self.running = True
            # A thread é marcada como 'daemon' para que ela seja terminada
            # automaticamente quando o programa principal sair.
            self.thread = threading.Thread(target=self._video_loop, daemon=True)
            self.thread.start()

    def _video_loop(self):
        """O coração da thread de vídeo. Lê frames em loop contínuo."""
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                self.running = False
                break
            # O processamento e desenho é delegado a outro método.
            self._process_and_draw(frame)

    def _process_and_draw(self, frame_bgr):
        """Executa o pipeline de visão computacional num único frame."""
        # 1. Conversão de Espaço de Cor: BGR -> HSV
        # O espaço de cor HSV é menos sensível a variações de iluminação do que o BGR/RGB.
        hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        
        # 2. Thresholding/Mascaramento: Isola os pixels que estão dentro da gama de cores definida.
        lower, upper = self.hsv_frame.get_hsv_bounds()
        mask = cv2.inRange(hsv_frame, lower, upper)
        
        # 3. Filtragem Morfológica: Remove ruído da máscara binária.
        # MORPH_OPEN remove pequenos pontos de ruído.
        # MORPH_CLOSE preenche pequenos buracos no objeto detetado.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 4. Encontrar Contornos e o Objeto Principal
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        ball_detected = False
        if contours:
            c = max(contours, key=cv2.contourArea) # Assume que o maior contorno é a bola.
            if cv2.contourArea(c) > 500: # Ignora contornos muito pequenos.
                ball_detected = True
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                
                # 5. Cálculo do Erro: A diferença entre a posição da bola e o centro do vídeo.
                erro_x = int(x) - (VIDEO_WIDTH // 2)
                # O eixo Y nas imagens é invertido (cresce para baixo). Invertemos o erro para
                # seguir a convenção matemática padrão (Y cresce para cima).
                erro_y = - (int(y) - (VIDEO_HEIGHT // 2))

                self.servo_x_var.set(f"Servo X: {erro_x}")
                self.servo_y_var.set(f"Servo Y: {erro_y}")

                # 6. Envio de Dados: Apenas se o seguimento estiver ativo.
                if self.tracking_enabled and self.serial_manager.serial_conn:
                    self.serial_manager.send(f"{CMD_ERROR_PREFIX},{erro_x},{erro_y}\n")

                # 7. Anotação Visual (Desenho sobre a imagem)
                display_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB) if not self.show_mask else cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                cv2.circle(display_img, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(display_img, (int(x), int(y)), 3, (255, 0, 0), -1)

        if not ball_detected:
            # Se não houver bola, limpa os labels e mostra a imagem original ou a máscara.
            self.servo_x_var.set("Servo X: 0"); self.servo_y_var.set("Servo Y: 0")
            self.pid_x_var.set("PID X: 0.00"); self.pid_y_var.set("PID Y: 0.00")
            display_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB) if not self.show_mask else cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

        # Desenha a cruz de referência no centro do vídeo.
        cx, cy = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2
        cv2.line(display_img, (cx - 10, cy), (cx + 10, cy), (255, 255, 0), 1)
        cv2.line(display_img, (cx, cy - 10), (cx, cy + 10), (255, 255, 0), 1)

        # 8. Exibição na GUI
        # O OpenCV usa arrays NumPy (formato BGR), mas o Tkinter precisa de um objeto de imagem específico.
        # A biblioteca Pillow (PIL) faz essa conversão.
        img_pil = Image.fromarray(display_img)
        img_tk = ImageTk.PhotoImage(image=img_pil)
        
        # O frame é desenhado no canvas.
        self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
        # Este é um ponto CRUCIAL em Tkinter: precisamos de manter uma referência à imagem
        # para que o garbage collector do Python não a apague.
        self.canvas.image = img_tk

    def change_camera(self, new_index):
        """Muda o índice da câmara e reinicia o vídeo."""
        try:
            self.camera_index = int(new_index)
            self._initialize_camera()
        except ValueError:
            messagebox.showerror("Erro", f"Índice de câmara inválido: {new_index}")

    def update_once(self):
        """Captura e processa um único frame. Útil após alterar configurações."""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret: self._process_and_draw(frame)

    def stop(self):
        """Para a thread de vídeo e liberta a câmara de forma segura."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1) # Espera a thread terminar.
        if self.cap:
            self.cap.release() # Liberta o recurso da câmara.
        self.cap = None
        self.thread = None


# -----------------------------------------------------------------------------
# Classe 6: OpenBalanceApp
# Função: A classe principal que constrói a janela e orquestra todas as outras classes.
# -----------------------------------------------------------------------------
class OpenBalanceApp:
    """
    Esta classe atua como o "maestro" da nossa orquestra de software.
    Ela instancia todos os componentes (SerialManager, VideoHandler, os Frames da UI)
    e organiza-os na janela principal.
    """
    def __init__(self):
        # Configuração inicial da aparência da aplicação.
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.app = ctk.CTk()
        self.app.title("OpenBalance Dashboard 3.1")
        self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        
        # Configuração do layout de grid da janela principal.
        self.app.grid_rowconfigure(0, weight=1) # A linha principal (0) expande-se verticalmente.
        self.app.grid_rowconfigure(1, weight=0) # A linha da status bar (1) não expande.
        self.app.grid_columnconfigure(0, weight=1, minsize=300) # Painel HSV
        self.app.grid_columnconfigure(1, weight=3, minsize=660) # Painel Vídeo
        self.app.grid_columnconfigure(2, weight=1, minsize=300) # Painel PID

        # Instanciação dos nossos componentes de lógica.
        self.serial_manager = SerialManager()
        self.video_handler = None # Será inicializado depois.

        # Construção da UI
        self._create_menu()
        self._create_frames()
        self._create_status_bar()

        # O protocolo WM_DELETE_WINDOW permite-nos intercetar o clique no 'X' da janela
        # para garantir que os recursos (câmara, serial) são libertados corretamente.
        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _create_menu(self):
        """Cria a barra de menu no topo da janela."""
        menu_bar = tk.Menu(self.app)
        arquivo_menu = tk.Menu(menu_bar, tearoff=0)
        arquivo_menu.add_command(label="Salvar Configuração", command=self._salvar_config)
        arquivo_menu.add_command(label="Carregar Configuração", command=self._carregar_config)
        arquivo_menu.add_separator()
        arquivo_menu.add_command(label="Sair", command=self._on_close)
        menu_bar.add_cascade(label="Arquivo", menu=arquivo_menu)
        self.app.config(menu=menu_bar)

    def _create_status_bar(self):
        """Instancia e posiciona a barra de status no rodapé."""
        self.status_bar = StatusBar(self.app)
        self.status_bar.grid(row=1, column=0, columnspan=3, sticky="sew")

    def _create_frames(self):
        """Cria e posiciona os três painéis principais da UI."""
        # Painel HSV (Esquerda)
        self.hsv_frame = HSVSettingsFrame(self.app, None, corner_radius=10)
        self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)

        # Painel de Vídeo (Centro)
        frame_video_container = ctk.CTkFrame(self.app, corner_radius=10)
        frame_video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        frame_video_container.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(frame_video_container, text="Área de Visualização", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10,5))
        
        cam_select_frame = ctk.CTkFrame(frame_video_container, fg_color="transparent")
        cam_select_frame.grid(row=1, column=0, padx=10, pady=5, sticky="w")
        ctk.CTkLabel(cam_select_frame, text="Câmera:").pack(side="left", padx=(0, 5))
        self.cam_index_var = tk.StringVar(value="0")
        self.option_menu_camera = ctk.CTkOptionMenu(cam_select_frame, values=["0", "1", "2", "3"], variable=self.cam_index_var, command=self._on_camera_change)
        self.option_menu_camera.pack(side="left")

        # Agora que os frames estão criados, podemos instanciar o VideoHandler
        # e passar as referências necessárias.
        self.video_handler = VideoHandler(frame_video_container, self.hsv_frame, self.serial_manager, int(self.cam_index_var.get()))
        self.hsv_frame.video_handler = self.video_handler # Injeção de dependência.

        trajectory_controls_frame = ctk.CTkFrame(frame_video_container, fg_color="transparent")
        trajectory_controls_frame.grid(row=4, column=0, pady=10, padx=10, sticky="s")
        ctk.CTkLabel(trajectory_controls_frame, text="Modo de Controlo").pack(pady=(0, 5))
        self.ctrl_menu = ctk.CTkOptionMenu(trajectory_controls_frame, values=["Manual", "Círculo", "Oito", "Quadrado"])
        self.ctrl_menu.pack(pady=5, fill="x")
        ctk.CTkButton(trajectory_controls_frame, text="Executar Trajetória", command=self._executar_trajetoria, **COLOR_NEUTRAL).pack(pady=5, fill="x")
        ctk.CTkButton(trajectory_controls_frame, text="Parar Movimento", command=self._parar_trajetoria, **COLOR_SECONDARY).pack(pady=5, fill="x")

        # Painel PID (Direita)
        self.pid_frame = PIDControllerFrame(self.app, self.serial_manager, self, corner_radius=10)
        self.pid_frame.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)
    
    # --- Funções de Callback ---
    def _on_camera_change(self, new_index):
        if self.video_handler: self.video_handler.change_camera(new_index)
    
    def _salvar_config(self):
        config = {"hsv": self.hsv_frame.save_hsv(), "pid": self.pid_frame.get_pid_config()}
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=CONFIG_FILE_TYPES)
        if not filepath: return
        try:
            with open(filepath, 'w', encoding='utf-8') as f: json.dump(config, f, indent=4)
            self.status_bar.show_message(f"Configuração salva em {filepath.split('/')[-1]}")
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

    def _executar_trajetoria(self):
        modo = self.ctrl_menu.get()
        self.status_bar.show_message(f"Função 'Executar Trajetória: {modo}' ainda não implementada.")

    def _parar_trajetoria(self):
        self.status_bar.show_message("Função 'Parar Movimento' ainda não implementada.")

    def _on_close(self):
        """Função de limpeza chamada antes de fechar a aplicação."""
        if self.video_handler: self.video_handler.stop()
        self.serial_manager.disconnect()
        self.app.destroy()

    def run(self):
        """Inicia o loop principal da aplicação."""
        self.app.mainloop()


# -----------------------------------------------------------------------------
# Ponto de Entrada do Script
# -----------------------------------------------------------------------------
# O bloco `if __name__ == "__main__"` é uma convenção em Python.
# O código dentro deste bloco só é executado quando o ficheiro é corrido
# diretamente (e não quando é importado como um módulo noutro script).
if __name__ == "__main__":
    # Criamos uma instância da nossa aplicação...
    app = OpenBalanceApp()
    # ...e iniciamos o seu ciclo de vida.
    app.run()