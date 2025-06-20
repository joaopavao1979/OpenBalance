# --------------------------------------------------
# File: app.py
# --------------------------------------------------
import customtkinter as ctk
import tkinter as tk

from serial_manager import SerialManager
from video_handler import VideoHandler
from config_manager import load_config, save_config
from frames.hsv_frame import HSVSettingsFrame
from frames.pid_frame import PIDFrame
from frames.control_frame import ControlFrame
from frames.status_bar import StatusBar

class OpenBalanceApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        # TODO: orquestrar inicialização e layout dos frames

    def run(self):
        self.mainloop()
