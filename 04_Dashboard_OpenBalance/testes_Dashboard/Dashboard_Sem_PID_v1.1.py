# -*- coding: utf-8 -*-
"""
================================================================================
OpenBalance Dashboard (Versão 6.0 - PID no Arduino via PCA9685)
================================================================================

Descrição:
    Dashboard para controlar uma plataforma de equilíbrio com base na posição
    de uma bola detetada por visão computacional. O PID é executado no Arduino,
    que comanda os servos via driver PCA9685. O PC apenas envia o erro (X, Y).

Principais funcionalidades:
    - Detecção da bola via OpenCV e HSV.
    - Envio do erro (em píxeis) para o Arduino via Serial.
    - Interface gráfica simples com ligação serial e vídeo em tempo real.
    - Suavização opcional com filtro de Kalman.
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
from tkinter import messagebox

# -----------------------------------------------------------------------------
# CONSTANTES GERAIS DE JANELA E SERIAL
# -----------------------------------------------------------------------------
WINDOW_WIDTH = 1500
WINDOW_HEIGHT = 850
VIDEO_WIDTH = 800
VIDEO_HEIGHT = 600
DEFAULT_BAUDRATE = 9600
CMD_ERROR_PREFIX = "E"

# Cores para botões
COLOR_SUCCESS = {"fg_color": "#2E7D32", "hover_color": "#1B5E20"}

# -----------------------------------------------------------------------------
# CLASSE SerialManager: gere a conexão com o Arduino
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
        except Exception:
            self.serial_conn = None
            return False

    def send(self, data_str):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(data_str.encode('utf-8'))

# -----------------------------------------------------------------------------
# CLASSE KalmanFilter: suaviza a posição da bola
# -----------------------------------------------------------------------------
class KalmanFilter:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,0,1,0]], np.float32)
        dt = 0.1
        self.kalman.transitionMatrix = np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5

    def predict(self):
        p = self.kalman.predict()
        return (p[0,0], p[2,0])

    def correct(self, meas):
        vec = np.array([[np.float32(meas[0])],[np.float32(meas[1])]])
        c = self.kalman.correct(vec)
        return (c[0,0], c[2,0])

# -----------------------------------------------------------------------------
# CLASSE PRINCIPAL: Interface do utilizador com vídeo e comunicação serial
# -----------------------------------------------------------------------------
class OpenBalanceApp:
    def __init__(self):
        ctk.set_appearance_mode("dark")
        self.root = ctk.CTk()
        self.root.title("OpenBalance Dashboard 6.0 - PCA9685")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")

        # Estado da app
        self.serial = SerialManager()
        self.port_var = tk.StringVar(value="COM3")  # Porta default (editar conforme o PC)

        # Interface para escolher e ligar à porta serial
        frame = ctk.CTkFrame(self.root)
        frame.pack(padx=20, pady=20)

        ctk.CTkLabel(frame, text="Porta Serial:").pack(side="left")
        ctk.CTkEntry(frame, textvariable=self.port_var, width=120).pack(side="left", padx=5)
        ctk.CTkButton(frame, text="Ligar", command=self._connect_serial, **COLOR_SUCCESS).pack(side="left")

        # Canvas para vídeo
        self.video = cv2.VideoCapture(0)
        self.canvas = ctk.CTkCanvas(self.root, width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
        self.canvas.pack(padx=10, pady=10)

        # Ativa o filtro de Kalman
        self.kalman = KalmanFilter()

        # Inicia a thread de vídeo
        self.running = True
        threading.Thread(target=self._video_loop, daemon=True).start()

    # Ligar à porta serial escolhida
    def _connect_serial(self):
        port = self.port_var.get()
        if self.serial.connect(port):
            messagebox.showinfo("Ligado", f"Ligado a {port}")
        else:
            messagebox.showerror("Erro", f"Falha na ligação a {port}")

    # Loop de vídeo: detecção da bola + envio do erro
    def _video_loop(self):
        while self.running:
            ret, frame = self.video.read()
            if not ret:
                continue

            # Conversão para HSV e aplicação de filtro
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array([40,100,100])  # Limites para detectar verde (exemplo)
            upper = np.array([80,255,255])
            mask = cv2.inRange(hsv, lower, upper)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if cnts:
                c = max(cnts, key=cv2.contourArea)
                (x, y), r = cv2.minEnclosingCircle(c)
                cx, cy = int(x), int(y)

                # Cálculo do erro (distância ao centro)
                err_x = VIDEO_WIDTH//2 - cx
                err_y = VIDEO_HEIGHT//2 - cy

                # Envia erro via Serial para o Arduino
                self.serial.send(f"{CMD_ERROR_PREFIX},{err_x},{err_y}\n")

                # Visualização da bola
                cv2.circle(img, (cx, cy), int(r), (0,255,0), 2)

            # Mostra imagem no Canvas
            img_tk = ImageTk.PhotoImage(Image.fromarray(img))
            self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
            self.canvas.image = img_tk

    def run(self):
        self.root.mainloop()
        self.running = False
        if self.video:
            self.video.release()

# -----------------------------------------------------------------------------
# INICIAR A APLICAÇÃO
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    OpenBalanceApp().run()
