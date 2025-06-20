# --------------------------------------------------
# File: frames/hsv_frame.py
# --------------------------------------------------
import customtkinter as ctk
import numpy as np

class HSVSettingsFrame(ctk.CTkFrame):
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        # TODO: copiar aqui a lógica dos sliders HSV e presets
        self.video_handler = video_handler

    def get_hsv_bounds(self):
        # Retorna tupla (lower, upper) como arrays NumPy
        pass

    def load_hsv(self, hsv_config: dict):
        pass

    def save_hsv(self) -> dict:
        return {}
