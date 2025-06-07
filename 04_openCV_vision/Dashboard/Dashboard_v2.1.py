"""
OpenBalance Dashboard (Versão com Coordenadas Centralizadas e Cruz Amarela)
Descrição: Dashboard para controlo de bola numa plataforma inclinável,
utilizando visão computacional (HSV) e PID.
Autor: João Pavão (Refatorado por ChatGPT)
Finalidade: Interface gráfica para ajuste de parâmetros HSV, controlo PID,
visualização de vídeo com preview de máscara em tempo real, botões de presets de cor
(com cores correspondentes), desenho de círculo ao redor da bola detectada,
envio de coordenadas centralizadas (centro = 0,0) via serial, e exibição de uma cruz amarela
no ponto (0,0) da imagem para referência. Agora com seleção de câmera e atualização de portas.
Uso: Código aberto, disponível livremente sob licença MIT.
"""

import json
import threading
import cv2
import numpy as np
import customtkinter as ctk         # GUI estilizada com CustomTkinter
import serial                        # Comunicação serial com Arduino
import serial.tools.list_ports       # Para listar portas seriais
from PIL import Image, ImageTk       # Converter frames OpenCV em imagens Tkinter
import tkinter as tk                 # GUI nativa para menus e diálogos
from tkinter import filedialog, messagebox
import csv                           # Biblioteca para manipulação de CSV

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
    },
    "Cinzento": {
        "h_min": 0,   "h_max": 180,
        "s_min": 0,   "s_max": 50,
        "v_min": 50,  "v_max": 200
    },
    "Azul": {
        "h_min": 90,  "h_max": 130,
        "s_min": 100, "s_max": 255,
        "v_min": 50,  "v_max": 255
    },
    "Branco": {
        "h_min": 0,   "h_max": 180,
        "s_min": 0,   "s_max": 30,
        "v_min": 200, "v_max": 255
    }
}

# Mapeamento de nome para cor de botão (hex)
BUTTON_COLORS = {
    "Vermelho":   {"fg": "#FF0000", "hover": "#CC0000", "text": "white"},
    "Laranja":    {"fg": "#FFA500", "hover": "#CC8400", "text": "white"},
    "Verde":      {"fg": "#00B200", "hover": "#008000", "text": "white"},
    "Cinzento":   {"fg": "#808080", "hover": "#666666", "text": "white"},
    "Azul":       {"fg": "#0000FF", "hover": "#0000CC", "text": "white"},
    "Branco":     {"fg": "#FFFFFF", "hover": "#DDDDDD", "text": "black"}
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
        :param data_str: string formatada com comandos, ex: "E,10,-5\n" ou "PID,...\n"
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        try:
            self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException:
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
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.video_handler = video_handler
        self.grid_columnconfigure(0, weight=1)

        # Título do frame
        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 16)).grid(
            row=0, column=0, pady=(10, 5), sticky="n"
        )

        self.hsv_sliders = []
        self.hsv_values = []

        # Criação dos sliders: H Min, H Max, S Min, S Max, V Min, V Max
        labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
        for idx, label in enumerate(labels):
            ctk.CTkLabel(self, text=label).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(master=self, value="0")
            self.hsv_values.append(val_var)

            slider = ctk.CTkSlider(
                self,
                from_=0,
                to=255,
                number_of_steps=255,
                command=lambda val, var=val_var: var.set(f"{int(float(val))}")
            )
            slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew")
            self.hsv_sliders.append(slider)

            ctk.CTkLabel(self, textvariable=val_var).grid(
                row=2 + 2*idx, column=0, padx=(270,10), sticky="e"
            )

        # Checkbox para mostrar/ocultar a máscara binária
        self.show_mask_var = ctk.BooleanVar(master=self, value=False)
        ctk.CTkCheckBox(
            self,
            text="Mostrar Máscara",
            variable=self.show_mask_var,
            command=self._on_toggle_mask
        ).grid(row=13, column=0, pady=(10,5), padx=10, sticky="w")

        # Botões de preset de cor (Vermelho, Laranja, Verde, Cinzento, Azul, Branco)
        row_base = 14
        for i, color_name in enumerate(HSV_PRESETS.keys()):
            colors = BUTTON_COLORS[color_name]
            ctk.CTkButton(
                self,
                text=color_name,
                fg_color=colors["fg"],
                hover_color=colors["hover"],
                text_color=colors["text"],
                command=lambda cn=color_name: self._apply_preset(cn)
            ).grid(row=row_base + i, column=0, pady=(2,2), padx=10, sticky="ew")

    def _apply_preset(self, color_name):
        """
        Aplica os valores de HSV predefinidos de acordo com o preset selecionado.
        Atualiza sliders e labels correspondentes, e dispara uma atualização única na visão.
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
        if self.video_handler:
            self.video_handler.update_once()

    def _on_toggle_mask(self):
        """
        Liga ou desliga a visualização da máscara binária.
        Informa o VideoHandler para exibir ou não a máscara.
        """
        if self.video_handler:
            self.video_handler.show_mask = self.show_mask_var.get()

    def get_hsv_bounds(self):
        """
        Retorna os limites HSV atuais (lower e upper) em forma de arrays numpy.
        """
        try:
            h_min = int(self.hsv_values[0].get())
            h_max = int(self.hsv_values[1].get())
            s_min = int(self.hsv_values[2].get())
            s_max = int(self.hsv_values[3].get())
            v_min = int(self.hsv_values[4].get())
            v_max = int(self.hsv_values[5].get())
        except ValueError:
            h_min, h_max, s_min, s_max, v_min, v_max = 0, 255, 0, 255, 0, 255

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        return lower, upper

    def load_hsv(self, hsv_dict):
        """
        Carrega um dicionário de configurações HSV (por ex., vindo de JSON) e atualiza sliders.
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
        Retorna um dicionário com os valores atuais de HSV.
        Pode ser usado para salvar em JSON.
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
    comunicação Arduino e atualização de portas seriais.
    """

    def __init__(self, parent, serial_manager: SerialManager, *args, **kwargs):
        super().__init__(parent, corner_radius=10, *args, **kwargs)
        self.serial_manager = serial_manager
        self.grid_columnconfigure(0, weight=1)

        # ===== Título =====
        ctk.CTkLabel(self, text="Controlador PID", font=("Arial", 16)).grid(
            row=0, column=0, pady=(10,5), sticky="n"
        )

        # ===== Sliders Kp, Ki, Kd =====
        self.pid_values = []
        for idx, param in enumerate(["Kp", "Ki", "Kd"]):
            ctk.CTkLabel(self, text=param).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(master=self, value="0.00")
            self.pid_values.append(val_var)

            slider = ctk.CTkSlider(
                self,
                from_=0,
                to=10,               # valor máximo = 10
                number_of_steps=100,
                command=lambda val, var=val_var: var.set(f"{float(val):.2f}")
            )
            slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew")
            ctk.CTkLabel(self, textvariable=val_var).grid(
                row=2 + 2*idx, column=0, padx=(270,10), sticky="e"
            )

        # ===== Slider Max Integral =====
        ctk.CTkLabel(self, text="Max Integral").grid(row=7, column=0, padx=10, sticky="w")
        self.max_integral_var = ctk.StringVar(master=self, value="0")
        max_int_slider = ctk.CTkSlider(
            self,
            from_=0,
            to=500,            # valor máximo = 500
            number_of_steps=100,
            command=lambda val: self.max_integral_var.set(f"{int(float(val))}")
        )
        max_int_slider.grid(row=8, column=0, padx=10, pady=(0,5), sticky="ew")
        ctk.CTkLabel(self, textvariable=self.max_integral_var).grid(
            row=8, column=0, padx=(270,10), sticky="e"
        )

        # ===== Entradas Offset X / Y =====
        ctk.CTkLabel(self, text="Offset X / Y").grid(row=9, column=0, pady=(10,2), padx=10, sticky="w")
        self.offset_x_entry = ctk.CTkEntry(self, placeholder_text="Offset X")
        self.offset_x_entry.grid(row=10, column=0, pady=2, padx=10, sticky="ew")
        self.offset_y_entry = ctk.CTkEntry(self, placeholder_text="Offset Y")
        self.offset_y_entry.grid(row=11, column=0, pady=2, padx=10, sticky="ew")

        # ===== Botão Auto Calibrar (inativo / greyed-out) =====
        # - Este botão aparece mas está desativado pois a rotina de auto-calibração não será usada por enquanto.
        self.btn_auto_calibrar = ctk.CTkButton(
            self,
            text="Auto Calibrar",
            command=self._auto_calibrar,
            fg_color="grey",
            hover_color="grey40",
            state="disabled"    # botão desativado
        )
        self.btn_auto_calibrar.grid(row=12, column=0, pady=(10,5), padx=10, sticky="ew")

        # ==== SEÇÃO DE CONTROLE DE MOTORES / SEGUIMENTO ====
        ctk.CTkLabel(self, text="Controlo de Motores", font=("Arial", 14)).grid(
            row=13, column=0, pady=(10,5), sticky="w", padx=10
        )

        # Botão “Ligar Motores” (leva servos a 90º para plataforma horizontal)
        ctk.CTkButton(
            self,
            text="Ligar Motores",
            command=self._ligar_motores,
            fg_color="#007ACC",
            hover_color="#005C9E"
        ).grid(row=14, column=0, pady=5, padx=10, sticky="ew")

        # Botão “Desligar Motores”
        ctk.CTkButton(
            self,
            text="Desligar Motores",
            command=self._desligar_motores,
            fg_color="#007ACC",
            hover_color="#005C9E"
        ).grid(row=15, column=0, pady=(0,10), padx=10, sticky="ew")

        # Botão “Ligar Seguimento” — DESATIVADO por enquanto
        self.btn_ligar_seguimento = ctk.CTkButton(
            self,
            text="Ligar Seguimento",
            command=self._iniciar_seguimento,
            fg_color="grey",
            hover_color="grey40",
            state="disabled"
        )
        self.btn_ligar_seguimento.grid(row=16, column=0, pady=(0,10), padx=10, sticky="ew")

        # Botão “Enviar PID para Arduino”
        ctk.CTkButton(
            self,
            text="Enviar PID para Arduino",
            command=self._enviar_pid_arduino,
            fg_color="#007ACC",
            hover_color="#005C9E"
        ).grid(row=17, column=0, pady=(0,10), padx=10, sticky="ew")

        # ==== SEÇÃO SEPARADA: Comunicação Arduino ====
        ctk.CTkLabel(self, text="Comunicação Arduino", font=("Arial", 14)).grid(
            row=18, column=0, pady=(10,5), padx=10, sticky="w"
        )

        # ----------- NOVO: Botão “Atualizar Portas” -----------
        # Sempre que o usuário clicar, teremos um refresh da lista de portas disponíveis
        ctk.CTkButton(
            self,
            text="Atualizar Portas",
            command=self._atualizar_portas,  # novo método para recarregar as portas
            fg_color="#888888",              # cor cinza clara
            hover_color="#666666"
        ).grid(row=19, column=0, pady=(0,5), padx=10, sticky="ew")
        # -------------------------------------------------------

        # Lista inicial de portas
        portas = self.serial_manager.list_ports()
        # variável que armazena a porta selecionada
        self.porta_var = tk.StringVar(master=self, value=portas[0] if portas else "")

        # Menu dropdown para selecionar a porta serial
        self.option_menu_porta = ctk.CTkOptionMenu(
            self,
            values=portas,
            variable=self.porta_var,
            fg_color="green",
            button_color="green",
            button_hover_color="darkgreen"
        )
        self.option_menu_porta.grid(row=20, column=0, pady=(0,5), padx=10, sticky="ew")

        # Botão “Ligar Arduino” abre a porta serial selecionada
        ctk.CTkButton(
            self,
            text="Ligar Arduino",
            command=self._ligar_arduino,
            fg_color="green",
            hover_color="darkgreen"
        ).grid(row=21, column=0, pady=5, padx=10, sticky="ew")

        # Botão “Desligar Arduino” fecha a conexão serial
        ctk.CTkButton(
            self,
            text="Desligar Arduino",
            command=self._desligar_arduino,
            fg_color="green",
            hover_color="darkgreen"
        ).grid(row=22, column=0, pady=(0,10), padx=10, sticky="ew")

        # Botão “Guardar PID em CSV” permanece disponível para exportar parâmetros
        ctk.CTkButton(
            self,
            text="Guardar PID em CSV",
            command=self._save_pid_csv,
            fg_color="orange",
            hover_color="darkorange"
        ).grid(row=23, column=0, pady=(0,10), padx=10, sticky="ew")

        # Ajustar pesos das linhas para permitir scroll caso necessário
        for r in range(0, 24):
            self.grid_rowconfigure(r, weight=0)
        self.grid_rowconfigure(24, weight=1)

    # =======================================================================
    #    MÉTODOS “placeholder” OU AINDA NÃO IMPLEMENTADOS (podem ficar vazios)
    # =======================================================================
    def _auto_calibrar(self):
        # Botão está desativado; se for reativado no futuro, aqui ficaria a lógica
        pass

    def _iniciar_seguimento(self):
        # Botão “Ligar Seguimento” está desativado; 
        # se reativado, aqui viria a lógica para habilitar envio de erros.
        messagebox.showinfo("Seguimento", "Funcionalidade de Seguimento ainda não implementada.")

    def _ligar_motores(self):
        """
        Envia comando "M1\n" para o Arduino, que deverá posicionar os servos a 90° (horizontal)
        e habilitar a flag de motores no firmware.
        """
        if self.serial_manager.serial_conn and self.serial_manager.serial_conn.is_open:
            cmd = "M1\n"
            self.serial_manager.send(cmd)
            messagebox.showinfo("Motores", "Enviado comando para Ligar Motores (90°).")
        else:
            messagebox.showwarning("Motores", "Arduino não conectado.")

    def _desligar_motores(self):
        """
        Envia comando "M0\n" para o Arduino, que deverá desativar os motores (parar PWM).
        """
        if self.serial_manager.serial_conn and self.serial_manager.serial_conn.is_open:
            cmd = "M0\n"
            self.serial_manager.send(cmd)
            messagebox.showinfo("Motores", "Enviado comando para Desligar Motores.")
        else:
            messagebox.showwarning("Motores", "Arduino não conectado.")

    # =======================================================================
    #           NOVO MÉTODO: ATUALIZAR LISTA DE PORTAS SERIAIS
    # =======================================================================
    def _atualizar_portas(self):
        """
        Recarrega a lista de portas seriais disponíveis e atualiza o OptionMenu.
        Deve ser chamado quando o usuário clicar no botão 'Atualizar Portas'.
        """
        # Obtém a lista atualizada
        novas_portas = self.serial_manager.list_ports()
        # Atualiza as opções do dropdown (CTkOptionMenu)
        self.option_menu_porta.configure(values=novas_portas)
        # Se houver ao menos uma porta, seleciona a primeira por padrão
        if novas_portas:
            self.porta_var.set(novas_portas[0])
        else:
            self.porta_var.set("")
        messagebox.showinfo("Portas Atualizadas", f"Portas disponíveis: {', '.join(novas_portas)}")
    # =======================================================================

    def _ligar_arduino(self):
        """
        Abre a porta serial selecionada pelo usuário.
        """
        porta = self.porta_var.get()
        if porta:
            self.serial_manager.connect(porta)
        else:
            messagebox.showwarning("Porta", "Nenhuma porta selecionada.")

    def _desligar_arduino(self):
        """
        Fecha a conexão serial com o Arduino.
        """
        self.serial_manager.disconnect()

    def _save_pid_csv(self):
        """
        Salva os valores atuais de Kp, Ki, Kd, MaxIntegral, OffsetX, OffsetY em um arquivo CSV.
        """
        pid_cfg = self.get_pid_config()
        filepath = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=CSV_FILE_TYPES)
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

    # =======================================================================
    #        RETORNA UM DICIONÁRIO COM OS VALORES ATUAIS DO PID/INT/OFFSET
    # =======================================================================
    def get_pid_config(self):
        """
        Lê valores atuais dos sliders (Kp, Ki, Kd, MaxIntegral) e entradas de offset.
        Retorna um dicionário contendo:
            { "kp", "ki", "kd", "max_integral", "offset_x", "offset_y" }
        """
        try:
            kp = float(self.pid_values[0].get())
            ki = float(self.pid_values[1].get())
            kd = float(self.pid_values[2].get())
            max_int = int(self.max_integral_var.get())
            offset_x = float(self.offset_x_entry.get()) if self.offset_x_entry.get() else 0.0
            offset_y = float(self.offset_y_entry.get()) if self.offset_y_entry.get() else 0.0
        except ValueError:
            kp, ki, kd, max_int, offset_x, offset_y = 0.0, 0.0, 0.0, 0, 0.0, 0.0
        return {
            "kp": kp, "ki": ki, "kd": kd,
            "max_integral": max_int,
            "offset_x": offset_x, "offset_y": offset_y
        }

    # =======================================================================
    #   CARREGA OS VALORES DE UM DICIONÁRIO (ex.: carregados de JSON)
    # =======================================================================
    def load_pid_config(self, pid_dict):
        """
        Recebe um dicionário com os valores de PID e carrega nos sliders e entradas.
        Estrutura esperada: {"kp":.., "ki":.., "kd":.., "max_integral":.., "offset_x":.., "offset_y":..}
        """
        try:
            sliders_vals = [pid_dict.get("kp", 0.0), pid_dict.get("ki", 0.0), pid_dict.get("kd", 0.0)]
            for i, val in enumerate(sliders_vals):
                # Encontra o slider correspondente via grid_slaves
                slider_widget = [w for w in self.grid_slaves(row=2 + 2*i, column=0)
                                 if isinstance(w, ctk.CTkSlider)]
                if slider_widget:
                    slider_widget[0].set(val)
                self.pid_values[i].set(f"{val:.2f}")

            max_int = pid_dict.get("max_integral", 0)
            max_int_slider = [w for w in self.grid_slaves(row=8, column=0)
                              if isinstance(w, ctk.CTkSlider)]
            if max_int_slider:
                max_int_slider[0].set(max_int)
            self.max_integral_var.set(str(int(max_int)))

            self.offset_x_entry.delete(0, tk.END)
            self.offset_x_entry.insert(0, str(pid_dict.get("offset_x", 0.0)))
            self.offset_y_entry.delete(0, tk.END)
            self.offset_y_entry.insert(0, str(pid_dict.get("offset_y", 0.0)))
        except Exception as e:
            messagebox.showerror("Erro PID", f"Falha ao carregar PID: {e}")

    # =======================================================================
    #   MÉTODO PARA ENVIAR OS PARÂMETROS DO PID AO ARDUINO (via serial)
    # =======================================================================
    def _enviar_pid_arduino(self):
        """
        Lê os parâmetros atuais de PID (Kp, Ki, Kd, MaxIntegral, OffsetX, OffsetY),
        monta a string no formato esperado pelo Arduino:
            "PID,<Kp>,<Ki>,<Kd>,<MaxIntegral>,<OffsetX>,<OffsetY>\n"
        e envia via serial. Em seguida, exibe um messagebox de confirmação.
        """
        pid_cfg = self.get_pid_config()
        kp = pid_cfg["kp"]
        ki = pid_cfg["ki"]
        kd = pid_cfg["kd"]
        max_int = pid_cfg["max_integral"]
        offset_x = pid_cfg["offset_x"]
        offset_y = pid_cfg["offset_y"]

        # Monta a string com duas casas decimais para floats e sem casas para MaxIntegral
        cmd_str = f"PID,{kp:.2f},{ki:.2f},{kd:.2f},{max_int},{offset_x:.2f},{offset_y:.2f}\n"
        self.serial_manager.send(cmd_str)

        messagebox.showinfo(
            "Enviar PID",
            f"Parâmetros PID enviados ao Arduino:\n"
            f"Kp = {kp:.2f}, Ki = {ki:.2f}, Kd = {kd:.2f}\n"
            f"MaxIntegral = {max_int}, OffsetX = {offset_x:.2f}, OffsetY = {offset_y:.2f}"
        )

# ----------------------------------------------------------------------------- 
# -------------------------- GERENCIADOR DE VÍDEO ----------------------------- 
# ----------------------------------------------------------------------------- 
class VideoHandler:
    """
    Classe que cria um canvas para exibir feed de vídeo (ou máscara) e gerencia um thread
    de captura baseado em OpenCV. Desenha uma cruz amarela no centro (0,0),
    converte coordenadas de pixel para erro centralizado (centro = 0,0), desenha
    um círculo ao redor da bola detectada e envia coordenadas centralizadas via serial.
    Agora permite seleção dinâmica do índice da câmera (0, 1, 2, ...).
    """
    def __init__(self, parent, hsv_frame: HSVSettingsFrame, serial_manager: SerialManager, camera_index=0):
        """
        :param parent: frame pai onde ficará o canvas
        :param hsv_frame: referência ao HSVSettingsFrame para obter limites HSV
        :param serial_manager: referência ao SerialManager para enviar dados
        :param camera_index: índice da câmera a usar (0, 1, 2, ...)
        """
        self.parent = parent
        self.hsv_frame = hsv_frame
        self.serial_manager = serial_manager
        self.show_mask = False
        self.camera_index = camera_index  # ARMAZENA O ÍNDICE DA CÂMERA SELECIONADA
        self.cap = None
        self.thread = None
        self.running = False

        # Canvas para exibir vídeo ou máscara
        self.canvas = ctk.CTkCanvas(
            parent,
            width=VIDEO_WIDTH,
            height=VIDEO_HEIGHT,
            bg="gray20",
            highlightthickness=0
        )
        self.canvas.grid(row=2, column=0, padx=10, pady=10)

        # Frame para exibir valores de Servo X/Y e PID X/Y
        self.status_frame = ctk.CTkFrame(parent, corner_radius=5)
        self.status_frame.grid(row=3, column=0, pady=5)
        self.servo_x_var = ctk.StringVar(master=parent, value="Servo X: 0")
        self.servo_y_var = ctk.StringVar(master=parent, value="Servo Y: 0")
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

        # Inicializa a câmera com o índice fornecido
        self._initialize_camera()

    def _initialize_camera(self):
        """
        Abre a webcam no índice especificado por self.camera_index.
        Configura resolução e FPS. Se falhar, exibe erro. Se bem-sucedido,
        inicia uma thread de captura contínua (_video_loop).
        """
        # Se já houver captura aberta, libera primeiro
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.running = False
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=1)

        # Abre a nova captura
        self.cap = cv2.VideoCapture(self.camera_index)
        # Forçar resolução 640x480 para processamento
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        if not self.cap.isOpened():
            messagebox.showerror("Erro de Vídeo", f"Não foi possível aceder à webcam (índice {self.camera_index}). Vídeo desabilitado.")
            self.cap = None
        else:
            self.running = True
            self.thread = threading.Thread(target=self._video_loop, daemon=True)
            self.thread.start()

    def change_camera(self, new_index):
        """
        Permite alterar o índice da câmera em tempo de execução.
        Faz shutdown da câmera atual e inicializa a nova com new_index.
        :param new_index: inteiro indicando a câmera (0, 1, 2, ...)
        """
        try:
            idx = int(new_index)
        except ValueError:
            messagebox.showerror("Câmera", f"Índice de câmera inválido: {new_index}")
            return

        # Atualiza atributo e reinicializa
        self.camera_index = idx
        self._initialize_camera()

    def update_once(self):
        """
        Captura um único frame da webcam, processa e desenha no canvas.
        Útil para atualizar a visualização após trocar de câmera ou carregar presets.
        """
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                return
            frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT), interpolation=cv2.INTER_AREA)
            self._process_and_draw(frame)

    def _video_loop(self):
        """
        Loop contínuo que lê frames da webcam enquanto `self.running` for True.
        Chamado em thread separado para não travar a GUI.
        """
        while self.running and self.cap:
            ret, frame = self.cap.read()
            if not ret:
                self.running = False
                messagebox.showerror("Erro de Vídeo", "Falha ao ler frame da webcam.")
                break

            frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT), interpolation=cv2.INTER_AREA)
            self._process_and_draw(frame)
            cv2.waitKey(10)

    def _process_and_draw(self, frame_bgr):
        """
        Processa o frame em BGR:
        1) Converte para HSV e aplica máscara conforme limites definidos.
        2) Filtragem morfológica para reduzir ruído.
        3) Encontra contornos para detectar a bola.
        4) Se bola encontrada, calcula erro centralizado (pixels) e envia via serial:
           "E,erroX,erroY\n". Também atualiza labels de Servo e PID.
        5) Desenha cruz amarela no centro (cx=320, cy=240).
        6) Desenha círculo ao redor da bola (se houver).
        7) Converte para PIL ImageTk e exibe no canvas.
        """
        # 1) Converte para HSV e aplica máscara
        hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower, upper = self.hsv_frame.get_hsv_bounds()
        mask = cv2.inRange(hsv_frame, lower, upper)

        # 2) Filtragem morfológica para reduzir ruído
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 3) Encontrar contornos na máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ball_center = None
        ball_radius = None

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 500:
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                ball_center = (int(x), int(y))
                ball_radius = int(radius)

                # 4) Converter pixels para erro centralizado (0 no centro)
                erro_x = ball_center[0] - (VIDEO_WIDTH // 2)
                erro_y = ball_center[1] - (VIDEO_HEIGHT // 2)

                # Enviar coordenadas centralizadas via serial: "E,erroX,erroY\n"
                if self.serial_manager and self.serial_manager.serial_conn and self.serial_manager.serial_conn.is_open:
                    cmd_str = f"E,{erro_x},{erro_y}\n"
                    self.serial_manager.send(cmd_str)

                # Atualizar labels de status com erro centralizado
                self.servo_x_var.set(f"Servo X: {erro_x}")
                self.servo_y_var.set(f"Servo Y: {erro_y}")

        # 5) Preparar imagem para exibir
        if self.show_mask:
            display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        else:
            display_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # 6) Desenhar cruz amarela no centro (cx=320, cy=240)
        cx = VIDEO_WIDTH // 2
        cy = VIDEO_HEIGHT // 2
        cruz_tamanho = 10  # comprimento de 10 px para cada ponta da cruz
        cor_amarela = (255, 255, 0)  # em RGB
        # Linha horizontal da cruz
        cv2.line(display_img,
                 (cx - cruz_tamanho, cy),
                 (cx + cruz_tamanho, cy),
                 cor_amarela, 1)
        # Linha vertical da cruz
        cv2.line(display_img,
                 (cx, cy - cruz_tamanho),
                 (cx, cy + cruz_tamanho),
                 cor_amarela, 1)

        # 7) Desenhar círculo ao redor da bola (se encontrada)
        if ball_center and ball_radius:
            cv2.circle(display_img, ball_center, ball_radius, (0, 255, 0), 2)
            cv2.circle(display_img, ball_center, 3, (255, 0, 0), -1)

        # 8) Converter para PIL ImageTk e exibir no canvas
        img_pil = Image.fromarray(display_img)
        img_tk = ImageTk.PhotoImage(image=img_pil)
        self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
        self.canvas.image = img_tk

    def stop(self):
        """
        Para a thread de captura de vídeo, libera a câmera e espera o término do thread.
        Chamado ao fechar a aplicação.
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
    Agora com seleção de câmera disponível na interface e botão para atualizar portas.
    """
    def __init__(self):
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        self.app = ctk.CTk()
        self.app.title("OpenBalance")
        self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.app.grid_rowconfigure(0, weight=1)
        self.app.grid_columnconfigure(0, weight=1)
        self.app.grid_columnconfigure(1, weight=3)
        self.app.grid_columnconfigure(2, weight=1)

        self.serial_manager = SerialManager()

        self._create_menu()
        self._create_frames()

        # Override do fechamento para encerrar thread de vídeo e desconectar Arduino
        self.app.protocol("WM_DELETE_WINDOW", self._on_close)

    def _create_menu(self):
        """
        Cria a barra de menus no topo com opções:
         - Arquivo: Salvar/Carregar Configuração (JSON), Sair
         - Configurações: Resetar Valores
         - Ajuda: Sobre
        """
        menu_bar = tk.Menu(self.app)

        arquivo_menu = tk.Menu(menu_bar, tearoff=0)
        arquivo_menu.add_command(label="Salvar Configuração", command=self._salvar_config)
        arquivo_menu.add_command(label="Carregar Configuração", command=self._carregar_config)
        arquivo_menu.add_separator()
        arquivo_menu.add_command(label="Sair", command=self.app.quit)
        menu_bar.add_cascade(label="Arquivo", menu=arquivo_menu)

        config_menu = tk.Menu(menu_bar, tearoff=0)
        config_menu.add_command(label="Resetar Valores", command=self._resetar_valores)
        menu_bar.add_cascade(label="Configurações", menu=config_menu)

        ajuda_menu = tk.Menu(menu_bar, tearoff=0)
        ajuda_menu.add_command(label="Sobre", command=self._mostrar_sobre)
        menu_bar.add_cascade(label="Ajuda", menu=ajuda_menu)

        self.app.config(menu=menu_bar)

    def _create_frames(self):
        """
        Cria e posiciona os três frames principais na janela:
          - Coluna 0: HSVSettingsFrame
          - Coluna 1: Área de Visualização (com seleção de câmera, VideoHandler e controls)
          - Coluna 2: PIDControllerFrame
        """
        # ----- Frame de configuração HSV (coluna 0) -----
        self.hsv_frame = HSVSettingsFrame(self.app, video_handler=None)
        self.hsv_frame.grid(row=0, column=0, sticky="nswe", padx=10, pady=10)

        # ----- Frame de vídeo + seleção de câmera + controles de trajetória (coluna 1) -----
        frame_video_container = ctk.CTkFrame(self.app, corner_radius=10)
        frame_video_container.grid(row=0, column=1, sticky="nswe", padx=10, pady=10)
        frame_video_container.grid_rowconfigure(2, weight=1)
        frame_video_container.grid_columnconfigure(0, weight=1)

        # Título da área de visualização
        ctk.CTkLabel(frame_video_container, text="Área de Visualização", font=("Arial", 16)).grid(
            row=0, column=0, pady=(10,5)
        )

        # ==== Opção para selecionar o índice da câmera (nova funcionalidade) ====
        ctk.CTkLabel(frame_video_container, text="Selecione Câmera (índice):").grid(
            row=1, column=0, pady=(5,2), padx=10, sticky="w"
        )
        # Varável que armazenará o índice da câmera como string
        self.cam_index_var = tk.StringVar(master=frame_video_container, value="0")
        # OptionMenu contém valores ["0", "1", "2", "3"] como exemplos de índices possíveis
        self.option_menu_camera = ctk.CTkOptionMenu(
            frame_video_container,
            values=["0", "1", "2", "3"],
            variable=self.cam_index_var,
            command=self._on_camera_change   # callback ao mudar de valor
        )
        self.option_menu_camera.grid(row=2, column=0, pady=(0,10), padx=10, sticky="w")

        # Instancia o VideoHandler usando o índice inicial de câmera (0)
        initial_cam_index = int(self.cam_index_var.get())
        self.video_handler = VideoHandler(frame_video_container, self.hsv_frame, self.serial_manager, camera_index=initial_cam_index)
        # Informa HSVSettingsFrame sobre o VideoHandler para permitir presets atualizarem a imagem
        self.hsv_frame.video_handler = self.video_handler

        # ----- Controles de trajetória abaixo do vídeo -----
        ctk.CTkLabel(frame_video_container, text="Modo de Controlo").grid(row=4, column=0, pady=(5,2))
        self.ctrl_menu = ctk.CTkOptionMenu(
            frame_video_container,
            values=["Manual", "Círculo", "Oito", "Quadrado", "Senoide"]
        )
        self.ctrl_menu.grid(row=5, column=0, pady=(0,10))

        ctk.CTkButton(
            frame_video_container,
            text="Executar Trajetória",
            command=self._executar_trajetoria
        ).grid(row=6, column=0, pady=(0,5))
        ctk.CTkButton(
            frame_video_container,
            text="Parar Movimento",
            command=self._parar_trajetoria
        ).grid(row=7, column=0, pady=(0,10))

        # ----- Frame PID (coluna 2) -----
        frame_pid_container = ctk.CTkScrollableFrame(self.app, corner_radius=10, width=250)
        frame_pid_container.grid(row=0, column=2, sticky="nswe", padx=10, pady=10)
        frame_pid_container.grid_rowconfigure(0, weight=1)
        frame_pid_container.grid_columnconfigure(0, weight=1)

        self.pid_frame = PIDControllerFrame(frame_pid_container, self.serial_manager)
        self.pid_frame.grid(row=0, column=0, sticky="nswe", padx=0, pady=0)

    def _on_camera_change(self, new_index):
        """
        Callback chamado quando o usuário escolhe outro índice de câmera.
        Converte new_index para int e chama VideoHandler.change_camera().
        """
        self.video_handler.change_camera(new_index)

    def _salvar_config(self):
        """
        Salva as configurações atuais de HSV e PID em um arquivo JSON.
        """
        config = {
            "hsv": self.hsv_frame.save_hsv(),
            "pid": self.pid_frame.get_pid_config()
        }
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=CONFIG_FILE_TYPES)
        if not filepath:
            return
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=4)
            messagebox.showinfo("Salvar Configuração", f"Configuração salva em:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Erro Salvar", f"Falha ao salvar configuração: {e}")

    def _carregar_config(self):
        """
        Carrega configurações de HSV e PID a partir de um arquivo JSON.
        """
        filepath = filedialog.askopenfilename(filetypes=CONFIG_FILE_TYPES)
        if not filepath:
            return
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                config = json.load(f)
            hsv_cfg = config.get("hsv", {})
            pid_cfg = config.get("pid", {})
            self.hsv_frame.load_hsv(hsv_cfg)
            self.pid_frame.load_pid_config(pid_cfg)
            messagebox.showinfo("Carregar Configuração", "Configuração carregada com sucesso.")
            self.video_handler.update_once()
        except Exception as e:
            messagebox.showerror("Erro Carregar", f"Falha ao carregar configuração: {e}")

    def _resetar_valores(self):
        """
        Reseta sliders de HSV e PID para valores padrões:
         - HSV: todos 0–255
         - PID: Kp=0, Ki=0, Kd=0, MaxIntegral=0, OffsetX=0, OffsetY=0
        Também desativa a máscara e atualiza a imagem.
        """
        default_hsv = {"h_min": 0, "h_max": 255, "s_min": 0, "s_max": 255, "v_min": 0, "v_max": 255}
        self.hsv_frame.load_hsv(default_hsv)

        default_pid = {"kp": 0.0, "ki": 0.0, "kd": 0.0, "max_integral": 0, "offset_x": 0.0, "offset_y": 0.0}
        self.pid_frame.load_pid_config(default_pid)

        self.hsv_frame.show_mask_var.set(False)
        self.video_handler.show_mask = False

        messagebox.showinfo("Resetar", "Valores de HSV e PID foram resetados para padrão.")
        self.video_handler.update_once()

    def _mostrar_sobre(self):
        """
        Exibe a janela 'Sobre' com informações de autoria e versão.
        """
        messagebox.showinfo(
            "Sobre",
            "OpenBalance Dashboard\nAutor: João Pavão\nVersão: 1.0\nRefatorado com cruz amarela no centro e coordenadas centralizadas"
        )

    def _executar_trajetoria(self):
        """
        Placeholder para executar uma trajetória predefinida.
        Em versões futuras, será implementado o movimento em Círculo, Oito, etc.
        """
        modo = self.ctrl_menu.get()
        messagebox.showinfo("Trajetória", f"Executar trajetória: {modo} (ainda não implementado)")

    def _parar_trajetoria(self):
        """
        Placeholder para parar o movimento de trajetória.
        """
        messagebox.showinfo("Trajetória", "Parar trajetória (ainda não implementado)")

    def _on_close(self):
        """
        Chamado ao fechar a janela principal. Garante que o vídeo e a serial sejam finalizados.
        """
        if hasattr(self, "video_handler") and self.video_handler:
            self.video_handler.stop()
        self.serial_manager.disconnect()
        self.app.destroy()

    def run(self):
        """
        Inicia o loop principal da aplicação.
        """
        self.app.mainloop()

# ----------------------------------------------------------------------------- 
# -------------------------- PONTO DE ENTRADA DO SCRIPT ----------------------- 
# ----------------------------------------------------------------------------- 
if __name__ == "__main__":
    app = OpenBalanceApp()
    app.run()
