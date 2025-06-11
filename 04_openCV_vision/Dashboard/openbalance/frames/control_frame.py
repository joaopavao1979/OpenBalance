# --------------------------------------------------
# File: frames/control_frame.py
# --------------------------------------------------
import customtkinter as ctk
import tkinter as tk
from serial_manager import SerialManager
from constants import CMD_MOTORS_ON, CMD_MOTORS_OFF, CMD_CALIBRATION_PREFIX

class ControlFrame(ctk.CTkFrame):
    def __init__(self, parent, serial_manager: SerialManager, app_ref, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        # TODO: mover aqui toda a lógica dos tabs Operação e Calibração