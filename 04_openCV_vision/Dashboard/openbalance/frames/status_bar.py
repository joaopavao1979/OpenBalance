# --------------------------------------------------
# File: frames/status_bar.py
# --------------------------------------------------
import customtkinter as ctk
from constants import COLOR_DANGER

class StatusBar(ctk.CTkFrame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w")
        self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None

    def show_message(self, message: str, duration_ms: int = 4000, is_error: bool = False):
        if self._job:
            self.after_cancel(self._job)
        color = COLOR_DANGER if is_error else None
        self.label.configure(text=message, text_color=color)
        self._job = self.after(duration_ms, lambda: self.label.configure(text=""))
