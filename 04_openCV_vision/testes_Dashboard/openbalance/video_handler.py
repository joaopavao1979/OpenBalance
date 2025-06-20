# --------------------------------------------------
# File: video_handler.py
# --------------------------------------------------
import cv2
import threading
from PIL import Image, ImageTk
import customtkinter as ctk

from pid_controller import PIDController
from utils import map_value

class VideoHandler:
    def __init__(self, parent, hsv_frame, serial_manager, app_ref):
        # TODO: mover lógica OpenCV, detecção de bola, PID, desenho
        pass

    def start(self):
        pass

    def stop(self):
        pass