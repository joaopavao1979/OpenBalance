# --------------------------------------------------
# File: frames/pid_frame.py
# --------------------------------------------------
import customtkinter as ctk
from pid_controller import PIDController

class PIDFrame(ctk.CTkFrame):
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        # TODO: implementar sliders e entries para Kp, Ki, Kd
        self.video_handler = video_handler

    def apply_gains(self):
        pass