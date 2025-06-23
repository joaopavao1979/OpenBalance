import cv2
import numpy as np
import serial
import tkinter as tk
from PIL import Image, ImageTk
import threading
import time

# Definição HSV para laranja (ajusta se necessário)
HSV_LARANJA_MIN = np.array([10, 100, 100])
HSV_LARANJA_MAX = np.array([25, 255, 255])

SERIAL_PORT = 'COM3'  # Altera para a porta correta no teu sistema
BAUDRATE = 115200

VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480

class OpenBalanceSimple:
    def __init__(self, master):
        self.master = master
        self.master.title("OpenBalance Simples - Bola Laranja")
        self.canvas = tk.Canvas(master, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, bg="black")
        self.canvas.pack()

        self.btn_motors_on = tk.Button(master, text="Ligar Motores", command=self.motors_on)
        self.btn_motors_on.pack(side=tk.LEFT, padx=10, pady=5)
        self.btn_motors_off = tk.Button(master, text="Desligar Motores", command=self.motors_off)
        self.btn_motors_off.pack(side=tk.LEFT, padx=10, pady=5)
        self.btn_seguir = tk.Button(master, text="Ligar Seguimento", command=self.toggle_seguimento)
        self.btn_seguir.pack(side=tk.LEFT, padx=10, pady=5)
        self.btn_stop = tk.Button(master, text="Desligar Seguimento", command=self.stop_tracking)
        self.btn_stop.pack(side=tk.LEFT, padx=10, pady=5)

        self.tracking = False
        self.serial_conn = None

        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        except Exception as e:
            print(f"Erro ao abrir porta serial: {e}")

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)

        self.running = True
        self.thread = threading.Thread(target=self.video_loop)
        self.thread.start()

    def motors_on(self):
        if self.serial_conn:
            try:
                self.serial_conn.write(b"M1\n")
                print("MOTORES LIGADOS (M1 enviado)")
            except Exception as e:
                print(f"Erro ao enviar M1: {e}")

    def motors_off(self):
        if self.serial_conn:
            try:
                self.serial_conn.write(b"M0\n")
                print("MOTORES DESLIGADOS (M0 enviado)")
            except Exception as e:
                print(f"Erro ao enviar M0: {e}")

    def toggle_seguimento(self):
        self.tracking = True
        print("Seguimento ativado")

    def stop_tracking(self):
        self.tracking = False
        print("Seguimento desativado")

    def video_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(frame_hsv, HSV_LARANJA_MIN, HSV_LARANJA_MAX)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            ball_center = None
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 400:
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    ball_center = (int(x), int(y))
                    cv2.circle(frame, ball_center, int(radius), (0,255,0), 2)
                    erro_x = ball_center[0] - (VIDEO_WIDTH // 2)
                    erro_y = -(ball_center[1] - (VIDEO_HEIGHT // 2))
                    txt = f"X:{erro_x}  Y:{erro_y}"
                    cv2.putText(frame, txt, (10, VIDEO_HEIGHT-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

                    # Envia erro via serial
                    if self.tracking and self.serial_conn:
                        try:
                            cmd = f"E,{erro_x},{erro_y}\n"
                            self.serial_conn.write(cmd.encode('utf-8'))
                        except Exception as e:
                            print(f"Erro ao enviar serial: {e}")

            # Converte imagem e atualiza canvas
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_pil = Image.fromarray(img_rgb)
            img_tk = ImageTk.PhotoImage(img_pil)
            self.canvas.create_image(0, 0, anchor="nw", image=img_tk)
            self.canvas.image = img_tk
            time.sleep(0.03)

    def on_close(self):
        self.running = False
        if self.cap: self.cap.release()
        if self.serial_conn: self.serial_conn.close()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = OpenBalanceSimple(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
