"""
OpenBalance Dashboard (Versão 4.2 - Edição de Calibração Definitiva)
====================================================================

Descrição:
    Dashboard para controlo de uma plataforma de equilíbrio de bola. Esta versão
    finaliza a implementação da arquitetura de controlo centralizado, com foco
    na usabilidade e robustez do processo de calibração.

    - Lógica do PID reside no Python.
    - Separador "Calibração" com ferramentas de Nivelamento e Amplitude.
    - Botões de teste dedicados para cada limite de movimento, permitindo
      uma calibração física iterativa e precisa.
    - Correção do layout dos botões de preset.
    - Código extensivamente comentado para fins didáticos.

Autor:
    João Pavão (Conceito Original)
    Refatorado e Comentado por Professor AI (para fins didáticos)
"""

# (Importações e Constantes Globais - Sem alterações)
import json; import threading; import cv2; import numpy as np; import customtkinter as ctk; import serial; import serial.tools.list_ports; from PIL import Image, ImageTk; import tkinter as tk; from tkinter import filedialog, messagebox
WINDOW_WIDTH = 1500; WINDOW_HEIGHT = 850; VIDEO_WIDTH = 800; VIDEO_HEIGHT = 600
DEFAULT_BAUDRATE = 9600; CONFIG_FILE_TYPES = [("JSON Files", "*.json")]
CMD_MOTORS_ON = "M1\n"; CMD_MOTORS_OFF = "M0\n"; CMD_CALIBRATION_PREFIX = "C"
COLOR_SUCCESS = {"fg_color": "#2E7D32", "hover_color": "#1B5E20"}; COLOR_DANGER = {"fg_color": "#C62828", "hover_color": "#B71C1C"}; COLOR_NEUTRAL = {"fg_color": "#1E88E5", "hover_color": "#1565C0"}; COLOR_SECONDARY = {"fg_color": "#616161", "hover_color": "#424242"}
HSV_PRESETS = {"Vermelho":{"h_min":0,"h_max":10,"s_min":100,"s_max":255,"v_min":100,"v_max":255},"Laranja":{"h_min":10,"h_max":25,"s_min":100,"s_max":255,"v_min":100,"v_max":255},"Verde":{"h_min":40,"h_max":80,"s_min":100,"s_max":255,"v_min":100,"v_max":255},"Cinzento":{"h_min":0,"h_max":180,"s_min":0,"s_max":50,"v_min":50,"v_max":200},"Azul":{"h_min":90,"h_max":130,"s_min":100,"s_max":255,"v_min":50,"v_max":255},"Branco":{"h_min":0,"h_max":180,"s_min":0,"s_max":30,"v_min":200,"v_max":255}}
BUTTON_COLORS = {"Vermelho":{"fg":"#FF0000","hover":"#CC0000","text":"white"},"Laranja":{"fg":"#FFA500","hover":"#CC8400","text":"white"},"Verde":{"fg":"#00B200","hover":"#008000","text":"white"},"Cinzento":{"fg":"#808080","hover":"#666666","text":"white"},"Azul":{"fg":"#0000FF","hover":"#0000CC","text":"white"},"Branco":{"fg":"#FFFFFF","hover":"#DDDDDD","text":"black"}}


# (Classes SerialManager, StatusBar, PIDController - Sem alterações)
class SerialManager:
    def __init__(self): self.serial_conn = None
    def list_ports(self): return [port.device for port in serial.tools.list_ports.comports()]
    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        try:
            if self.serial_conn and self.serial_conn.is_open: self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=1); return True
        except Exception: self.serial_conn = None; return False
    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open: self.serial_conn.close()
        self.serial_conn = None
    def send(self, data_str):
        if not self.serial_conn or not self.serial_conn.is_open: return
        try: self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException: self.disconnect()
class StatusBar(ctk.CTkFrame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.label = ctk.CTkLabel(self, text="", anchor="w"); self.label.pack(side="left", fill="x", expand=True, padx=10, pady=2)
        self._job = None
    def show_message(self, message, duration_ms=4000, is_error=False):
        if self._job: self.after_cancel(self._job)
        text_color = COLOR_DANGER["fg_color"] if is_error else "gray70"
        self.label.configure(text=message, text_color=text_color)
        self._job = self.after(duration_ms, lambda: self.label.configure(text=""))
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0): self.Kp, self.Ki, self.Kd = Kp, Ki, Kd; self.setpoint = setpoint; self._last_error, self._integral = 0, 0
    def update(self, process_variable):
        error = self.setpoint - process_variable; self._integral += error; derivative = error - self._last_error
        self._last_error = error; return self.Kp * error + self.Ki * self._integral + self.Kd * derivative
    def set_gains(self, Kp, Ki, Kd): self.Kp, self.Ki, self.Kd = Kp, Ki, Kd; self._integral = 0

# -----------------------------------------------------------------------------
# Classe 3: HSVSettingsFrame (CORRIGIDA)
# -----------------------------------------------------------------------------
class HSVSettingsFrame(ctk.CTkFrame):
    def __init__(self, parent, video_handler, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.video_handler = video_handler; self.grid_columnconfigure(0, weight=1); self.filter_enabled = True
        ctk.CTkLabel(self, text="Deteção de Cor (HSV)", font=("Arial", 16, "bold")).grid(row=0, column=0, pady=(10, 15), sticky="n")
        self.hsv_sliders = []; self.hsv_values = []
        labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
        for idx, label in enumerate(labels):
            ctk.CTkLabel(self, text=label).grid(row=1 + 2*idx, column=0, padx=10, sticky="w")
            val_var = ctk.StringVar(value="0"); self.hsv_values.append(val_var)
            slider = ctk.CTkSlider(self, from_=0, to=255, command=lambda val, var=val_var: var.set(f"{int(val)}")); slider.grid(row=2 + 2*idx, column=0, padx=10, pady=(0,5), sticky="ew"); self.hsv_sliders.append(slider)
            ctk.CTkLabel(self, textvariable=val_var).grid(row=2 + 2*idx, column=0, padx=(270,10), sticky="e")
        self.show_mask_var = ctk.BooleanVar(value=False)
        self.checkbox_mask = ctk.CTkCheckBox(self, text="Mostrar Máscara", variable=self.show_mask_var, command=self._on_toggle_mask)
        self.checkbox_mask.grid(row=13, column=0, pady=(15,10), padx=10, sticky="w")
        
        preset_frame = ctk.CTkFrame(self, fg_color="transparent"); preset_frame.grid(row=14, column=0, sticky="ew", padx=10)
        preset_frame.grid_columnconfigure((0, 1, 2), weight=1) 
        ctk.CTkButton(preset_frame, text="Sem Filtro", command=self._disable_filter, **COLOR_SECONDARY).grid(row=0, column=0, pady=3, padx=3, sticky="ew")
        
        # ## CORREÇÃO v4.2: Lógica de layout dos botões de cor ##
        color_items = list(BUTTON_COLORS.items())
        for i, (name, colors) in enumerate(color_items):
            row = (i + 1) // 3
            col = (i + 1) % 3
            ctk.CTkButton(preset_frame, text=name, fg_color=colors["fg"], hover_color=colors["hover"], text_color=colors["text"], command=lambda cn=name: self._apply_preset(cn)).grid(row=row, column=col, pady=3, padx=3, sticky="ew")

    def _toggle_filter_controls(self, enabled):
        state = "normal" if enabled else "disabled"
        for widget in self.hsv_sliders + [self.checkbox_mask]: widget.configure(state=state)
        self.filter_enabled = enabled
    def _disable_filter(self):
        self._toggle_filter_controls(False)
        if self.video_handler: self.video_handler.update_once()
    def _apply_preset(self, color_name):
        self._toggle_filter_controls(True)
        preset = HSV_PRESETS.get(color_name)
        if not preset: return
        vals = [preset[k] for k in ["h_min", "h_max", "s_min", "s_max", "v_min", "v_max"]]
        for i, val in enumerate(vals): self.hsv_sliders[i].set(val); self.hsv_values[i].set(str(val))
        if self.video_handler: self.video_handler.update_once()
    def _on_toggle_mask(self):
        if self.video_handler: self.video_handler.show_mask = self.show_mask_var.get()
    def get_hsv_bounds(self):
        try: vals = [int(v.get()) for v in self.hsv_values]
        except (ValueError, tk.TclError): vals = [0, 255, 0, 255, 0, 255]
        return np.array([vals[0], vals[2], vals[4]]), np.array([vals[1], vals[3], vals[5]])
    def load_hsv(self, hsv_dict):
        self._toggle_filter_controls(True)
        vals = [hsv_dict.get(k, d) for k, d in [("h_min",0),("h_max",255),("s_min",0),("s_max",255),("v_min",0),("v_max",255)]]
        for i, val in enumerate(vals): self.hsv_sliders[i].set(val); self.hsv_values[i].set(str(val))
    def save_hsv(self):
        l, u = self.get_hsv_bounds(); return {"h_min": int(l[0]),"h_max": int(u[0]),"s_min": int(l[1]),"s_max": int(u[1]),"v_min": int(l[2]),"v_max": int(u[2])}

# (VideoHandler - Sem alterações)
class VideoHandler:
    def __init__(self, parent, hsv_frame, serial_manager, app_ref):
        self.parent = parent; self.hsv_frame = hsv_frame; self.serial_manager = serial_manager; self.app = app_ref
        self.camera_index = 0; self.show_mask = False; self.tracking_enabled = False; self.running = False; self.cap = None; self.thread = None
        self.pid_x = PIDController(0, 0, 0); self.pid_y = PIDController(0, 0, 0)
        self.canvas = ctk.CTkCanvas(parent, width=VIDEO_WIDTH, height=VIDEO_HEIGHT, bg="gray20", highlightthickness=0); self.canvas.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        self.status_frame = ctk.CTkFrame(parent, corner_radius=5); self.status_frame.grid(row=3, column=0, pady=(0, 10), padx=10, sticky="ew"); self.status_frame.grid_columnconfigure((0, 1), weight=1)
        self.servo_x_var = ctk.StringVar(value="Erro X: 0"); self.servo_y_var = ctk.StringVar(value="Erro Y: 0"); self.pid_x_var = ctk.StringVar(value="PID X: 0.00"); self.pid_y_var = ctk.StringVar(value="PID Y: 0.00")
        ctk.CTkLabel(self.status_frame, textvariable=self.servo_x_var).grid(row=0, column=0, padx=10, pady=2, sticky="w"); ctk.CTkLabel(self.status_frame, textvariable=self.servo_y_var).grid(row=1, column=0, padx=10, pady=2, sticky="w")
        ctk.CTkLabel(self.status_frame, textvariable=self.pid_x_var).grid(row=0, column=1, padx=10, pady=2, sticky="w"); ctk.CTkLabel(self.status_frame, textvariable=self.pid_y_var).grid(row=1, column=1, padx=10, pady=2, sticky="w")
        self._initialize_camera()
    def _process_and_draw(self, frame_bgr):
        display_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        if self.hsv_frame.filter_enabled:
            hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV); lower, upper = self.hsv_frame.get_hsv_bounds(); mask = cv2.inRange(hsv_frame, lower, upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)); mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel); mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            if self.show_mask: display_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            ball_detected = False
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 500:
                    ball_detected = True; ((x, y), radius) = cv2.minEnclosingCircle(c)
                    erro_x = int(x) - (VIDEO_WIDTH // 2); erro_y = -(int(y) - (VIDEO_HEIGHT // 2)); self.servo_x_var.set(f"Erro X: {erro_x}"); self.servo_y_var.set(f"Erro Y: {erro_y}")
                    if self.tracking_enabled: pid_output_x = self.pid_x.update(erro_x); pid_output_y = self.pid_y.update(erro_y); self.pid_x_var.set(f"PID X: {pid_output_x:.2f}"); self.pid_y_var.set(f"PID Y: {pid_output_y:.2f}"); self.send_servo_command(pid_output_x, pid_output_y)
                    cv2.circle(display_img, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            if not ball_detected: self.servo_x_var.set("Erro X: 0"); self.servo_y_var.set("Erro Y: 0"); self.pid_x_var.set("PID X: 0.00"); self.pid_y_var.set("PID Y: 0.00")
        else: self.servo_x_var.set("Erro X: (sem filtro)"); self.servo_y_var.set("Erro Y: (sem filtro)")
        cx, cy = VIDEO_WIDTH // 2, VIDEO_HEIGHT // 2; cv2.line(display_img, (cx-10, cy), (cx+10, cy), (255,255,0), 1); cv2.line(display_img, (cx, cy-10), (cx, cy+10), (255,255,0), 1)
        img_pil = Image.fromarray(display_img); img_tk = ImageTk.PhotoImage(image=img_pil); self.canvas.create_image(0, 0, anchor="nw", image=img_tk); self.canvas.image = img_tk
    def send_servo_command(self, pid_output_x, pid_output_y):
        calib = self.app.calibration_data; pid_min, pid_max = -1000, 1000
        target_x_us = self.map_value(pid_output_x, pid_min, pid_max, calib['x_min_us'], calib['x_max_us']) + calib['x_center_offset_us']
        target_y_us = self.map_value(pid_output_y, pid_min, pid_max, calib['y_min_us'], calib['y_max_us']) + calib['y_center_offset_us']
        cmd_str = f"{CMD_CALIBRATION_PREFIX},{int(target_x_us)},{int(target_y_us)}\n"; self.serial_manager.send(cmd_str)
    @staticmethod
    def map_value(v, f_min, f_max, t_min, t_max): return (v-f_min)*(t_max-t_min)/(f_max-f_min)+t_min
    def _initialize_camera(self):
        self.stop(); self.camera_index = int(self.app.cam_index_var.get())
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW); self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
        if not self.cap.isOpened(): messagebox.showerror("Erro de Câmara", f"Não foi possível aceder à câmara (índice {self.camera_index})."); self.cap = None
        else: self.running = True; self.thread = threading.Thread(target=self._video_loop, daemon=True); self.thread.start()
    def _video_loop(self):
        while self.running and self.cap and self.cap.isOpened(): ret, frame = self.cap.read(); _ = not ret and self.running and setattr(self, 'running', False) or self._process_and_draw(frame)
    def stop(self): self.running = False; _ = self.thread and self.thread.is_alive() and self.thread.join(timeout=1); _ = self.cap and self.cap.release(); self.cap = None; self.thread = None
    def update_once(self): _ = self.cap and self.cap.isOpened() and (lambda: (ret, frame) if (ret := self.cap.read()[0]) else None and self._process_and_draw(frame))()
    def change_camera(self, new_index): _ = self.running and self._initialize_camera()

# -----------------------------------------------------------------------------
# Classe 6: ControlFrame (MELHORADA)
# -----------------------------------------------------------------------------
class ControlFrame(ctk.CTkFrame):
    def __init__(self, parent, serial_manager, app_ref, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.serial_manager = serial_manager; self.app = app_ref
        self.tab_view = ctk.CTkTabview(self); self.tab_view.pack(expand=True, fill="both")
        self._create_operation_tab(self.tab_view.add("Operação")); self._create_calibration_tab(self.tab_view.add("Calibração"))
    def _create_operation_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)
        pid_group = ctk.CTkFrame(tab, fg_color="transparent"); pid_group.grid(row=0, column=0, padx=10, pady=10, sticky="ew"); pid_group.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(pid_group, text="Ganhos do Controlador", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=3, pady=5)
        self.pid_params_widgets = {}
        for i, param in enumerate(["Kp", "Ki", "Kd"]):
            ctk.CTkLabel(pid_group, text=param).grid(row=i+1, column=0, padx=(5,10), pady=5, sticky="w"); slider = ctk.CTkSlider(pid_group, from_=0, to=1, number_of_steps=1000); slider.grid(row=i+1, column=1, padx=5, pady=5, sticky="ew"); entry = ctk.CTkEntry(pid_group, width=60); entry.grid(row=i+1, column=2, padx=(5,10), pady=5, sticky="e")
            def update_from_slider(v,p=param): w=self.pid_params_widgets[p]['entry']; w.delete(0,'end'); w.insert(0,f"{v:.3f}")
            def update_from_entry(e,p=param):
                try: self.pid_params_widgets[p]['slider'].set(float(self.pid_params_widgets[p]['entry'].get()))
                except(ValueError,KeyError,tk.TclError): pass
            slider.configure(command=update_from_slider); entry.bind("<Return>",update_from_entry); entry.bind("<FocusOut>",update_from_entry)
            self.pid_params_widgets[param] = {'slider':slider,'entry':entry}; self.pid_params_widgets[param]['entry'].insert(0,"0.000")
        ctk.CTkButton(pid_group,text="Aplicar Ganhos PID",command=self._apply_pid_gains,**COLOR_NEUTRAL).grid(row=4,column=0,columnspan=3,pady=10,sticky="ew")
        motor_group = ctk.CTkFrame(tab, fg_color="transparent"); motor_group.grid(row=1, column=0, padx=10, pady=10, sticky="ew"); motor_group.grid_columnconfigure((0,1), weight=1)
        ctk.CTkLabel(motor_group,text="Controlo do Sistema",font=("Arial",14,"bold")).grid(row=0,column=0,columnspan=2,pady=5)
        ctk.CTkButton(motor_group,text="Ligar Motores",command=lambda:self.serial_manager.send(CMD_MOTORS_ON),**COLOR_SUCCESS).grid(row=1,column=0,padx=5,pady=5,sticky="ew")
        ctk.CTkButton(motor_group,text="Desligar Motores",command=lambda:self.serial_manager.send(CMD_MOTORS_OFF),**COLOR_DANGER).grid(row=1,column=1,padx=5,pady=5,sticky="ew")
        self.btn_ligar_seguimento = ctk.CTkButton(motor_group,text="Ligar Seguimento",command=self._toggle_seguimento,**COLOR_SECONDARY); self.btn_ligar_seguimento.grid(row=2,column=0,columnspan=2,padx=5,pady=5,sticky="ew")
        serial_group = ctk.CTkFrame(tab, fg_color="transparent"); serial_group.grid(row=2, column=0, padx=10, pady=10, sticky="ew"); serial_group.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(serial_group,text="Comunicação Arduino",font=("Arial",14,"bold")).grid(row=0,column=0,columnspan=2,pady=5)
        portas = self.serial_manager.list_ports(); self.porta_var = tk.StringVar(value=portas[0] if portas else "Nenhuma porta"); self.option_menu_porta = ctk.CTkOptionMenu(serial_group,values=portas if portas else ["Nenhuma porta"],variable=self.porta_var); self.option_menu_porta.grid(row=1,column=0,padx=5,pady=5,sticky="ew")
        ctk.CTkButton(serial_group,text="🔄",width=30,command=self._atualizar_portas).grid(row=1,column=1,padx=5,pady=5)
        ctk.CTkButton(serial_group,text="Ligar",command=self._ligar_arduino,**COLOR_SUCCESS).grid(row=2,column=0,padx=5,pady=5,sticky="ew"); ctk.CTkButton(serial_group,text="Desligar",command=self._desligar_arduino,**COLOR_DANGER).grid(row=2,column=1,padx=5,pady=5,sticky="ew")
    def _create_calibration_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)
        center_group = ctk.CTkFrame(tab, fg_color="transparent"); center_group.grid(row=0, column=0, sticky="ew", padx=10, pady=10); center_group.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(center_group, text="Nivelamento Central (Offset μs)", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=2, pady=5)
        self.calib_vars = {}
        for i, name in enumerate(['x_center_offset_us', 'y_center_offset_us']):
            var = tk.IntVar(value=self.app.calibration_data.get(name, 0)); self.calib_vars[name] = var; label = "X" if "x_" in name else "Y"
            ctk.CTkLabel(center_group, text=f"Offset {label}:").grid(row=i+1, column=0, sticky="w", padx=5)
            entry = ctk.CTkEntry(center_group, textvariable=var, width=60); entry.grid(row=i+1, column=1, sticky="e", padx=5); entry.bind("<Return>", self._send_center_calib_command); entry.bind("<FocusOut>", self._send_center_calib_command)
        ctk.CTkButton(center_group, text="Testar Posição Central", command=self._send_center_calib_command, **COLOR_NEUTRAL).grid(row=3, column=0, columnspan=2, sticky="ew", pady=10, padx=5)
        
        # ## MELHORIA v4.2: UI de Limites com Botões de Teste ##
        limits_group = ctk.CTkFrame(tab, fg_color="transparent"); limits_group.grid(row=1, column=0, sticky="ew", padx=10, pady=10); limits_group.grid_columnconfigure((1,3), weight=1)
        ctk.CTkLabel(limits_group, text="Limites de Movimento (μs)", font=("Arial", 14, "bold")).grid(row=0, column=0, columnspan=5, pady=5)
        for name in ['x_min_us', 'x_max_us', 'y_min_us', 'y_max_us']:
            var = tk.IntVar(value=self.app.calibration_data.get(name, 1500)); self.calib_vars[name] = var
            axis, limit, row = ("X","Mín",1) if name=='x_min_us' else (("X","Máx",1) if name=='x_max_us' else (("Y","Mín",2) if name=='y_min_us' else ("Y","Máx",2)))
            col_offset = 0 if "min" in name else 2
            ctk.CTkLabel(limits_group, text=f"{axis} {limit}:").grid(row=row, column=col_offset, sticky="w", padx=5)
            ctk.CTkEntry(limits_group, textvariable=var, width=70).grid(row=row, column=col_offset+1, sticky="ew", padx=5)
            ctk.CTkButton(limits_group, text="T", width=25, command=lambda n=name: self._send_limit_test_command(n)).grid(row=row, column=col_offset+2, padx=(0,5))
    def _apply_pid_gains(self):
        try:
            kp=float(self.pid_params_widgets['Kp']['entry'].get()); ki=float(self.pid_params_widgets['Ki']['entry'].get()); kd=float(self.pid_params_widgets['Kd']['entry'].get())
            self.app.video_handler.pid_x.set_gains(kp,ki,kd); self.app.video_handler.pid_y.set_gains(kp,ki,kd); self.app.status_bar.show_message("Ganhos PID aplicados.")
        except(ValueError,AttributeError,tk.TclError): self.app.status_bar.show_message("Erro: Ganhos PID inválidos.",is_error=True)
    def _send_center_calib_command(self, _=None):
        x_us = 1500 + self.calib_vars['x_center_offset_us'].get(); y_us = 1500 + self.calib_vars['y_center_offset_us'].get()
        self.serial_manager.send(f"{CMD_CALIBRATION_PREFIX},{x_us},{y_us}\n")
    def _send_limit_test_command(self, limit_name):
        x_us = 1500 + self.calib_vars['x_center_offset_us'].get(); y_us = 1500 + self.calib_vars['y_center_offset_us'].get()
        if "x" in limit_name: x_us = self.calib_vars[limit_name].get()
        if "y" in limit_name: y_us = self.calib_vars[limit_name].get()
        self.serial_manager.send(f"{CMD_CALIBRATION_PREFIX},{x_us},{y_us}\n")
    def _toggle_seguimento(self):
        vh = self.app.video_handler; vh.tracking_enabled = not vh.tracking_enabled
        if vh.tracking_enabled: self.btn_ligar_seguimento.configure(text="Parar Seguimento",**COLOR_DANGER); self.app.status_bar.show_message("Seguimento da bola ativado.")
        else: self.btn_ligar_seguimento.configure(text="Ligar Seguimento",**COLOR_SECONDARY); self.app.status_bar.show_message("Seguimento da bola desativado.")
    def _atualizar_portas(self):
        novas_portas=self.serial_manager.list_ports(); self.option_menu_porta.configure(values=novas_portas if novas_portas else ["Nenhuma porta"]); self.porta_var.set(novas_portas[0] if novas_portas else "Nenhuma porta")
    def _ligar_arduino(self):
        porta = self.porta_var.get()
        if porta and porta != "Nenhuma porta":
            if self.serial_manager.connect(porta): self.app.status_bar.show_message(f"Conectado a {porta}.", 5000)
            else: self.app.status_bar.show_message(f"Falha ao conectar a {porta}.",5000,is_error=True)
    def _desligar_arduino(self): self.serial_manager.disconnect(); self.app.status_bar.show_message("Conexão serial encerrada.")
    def get_pid_config(self):
        try: return {p: float(w['entry'].get()) for p, w in self.pid_params_widgets.items()}
        except(ValueError,KeyError,tk.TclError): return {"kp":0.0,"ki":0.0,"kd":0.0}
    def load_pid_config(self, pid_dict):
        for param, value in pid_dict.items():
            if param in self.pid_params_widgets: w=self.pid_params_widgets[param]; w['slider'].set(value); w['entry'].delete(0,'end'); w['entry'].insert(0,f"{value:.3f}")
        self._apply_pid_gains()

# -----------------------------------------------------------------------------
# Classe 7: OpenBalanceApp
# -----------------------------------------------------------------------------
class OpenBalanceApp:
    def __init__(self):
        ctk.set_appearance_mode("dark"); ctk.set_default_color_theme("blue")
        self.app = ctk.CTk(); self.app.title("OpenBalance Dashboard 4.2"); self.app.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.app.grid_rowconfigure(0, weight=1); self.app.grid_rowconfigure(1, weight=0); self.app.grid_columnconfigure(0, weight=1, minsize=320); self.app.grid_columnconfigure(1, weight=4, minsize=VIDEO_WIDTH + 40); self.app.grid_columnconfigure(2, weight=1, minsize=320)
        self.serial_manager = SerialManager(); self.video_handler = None
        self._create_status_bar(); self._load_app_config(); self._create_menu(); self._create_frames(); self.app.protocol("WM_DELETE_WINDOW", self._on_close)
    def _load_app_config(self):
        self.calibration_data = {"x_center_offset_us":0,"y_center_offset_us":0,"x_min_us":1000,"x_max_us":2000,"y_min_us":1000,"y_max_us":2000}
        try:
            with open("config.json",'r',encoding='utf-8') as f: config=json.load(f)
            self.hsv_config=config.get("hsv",{}); self.pid_config=config.get("pid",{}); self.calibration_data.update(config.get("calibration",{}))
        except(FileNotFoundError,json.JSONDecodeError): self.hsv_config={}; self.pid_config={}; self.status_bar.show_message("config.json não encontrado.",is_error=True)
    def _save_app_config(self):
        if hasattr(self,'control_frame') and self.control_frame:
            for name,var in self.control_frame.calib_vars.items(): self.calibration_data[name]=var.get()
        config = {"hsv":self.hsv_frame.save_hsv() if hasattr(self,'hsv_frame') else {},"pid":self.control_frame.get_pid_config() if hasattr(self,'control_frame') else {},"calibration":self.calibration_data}
        try:
            with open("config.json",'w',encoding='utf-8') as f: json.dump(config,f,indent=4)
            self.status_bar.show_message("Configuração salva.")
        except Exception as e: self.status_bar.show_message(f"Erro ao salvar: {e}",is_error=True)
    def _create_frames(self):
        self.hsv_frame = HSVSettingsFrame(self.app,None,corner_radius=10); self.hsv_frame.grid(row=0,column=0,sticky="nswe",padx=10,pady=10)
        frame_video_container = ctk.CTkFrame(self.app,corner_radius=10); frame_video_container.grid(row=0,column=1,sticky="nswe",padx=10,pady=10); frame_video_container.grid_columnconfigure(0,weight=1); frame_video_container.grid_rowconfigure(2,weight=1)
        ctk.CTkLabel(frame_video_container,text="Área de Visualização",font=("Arial",16,"bold")).grid(row=0,column=0,pady=(10,5))
        cam_select_frame = ctk.CTkFrame(frame_video_container,fg_color="transparent"); cam_select_frame.grid(row=1,column=0,padx=10,pady=5,sticky="w"); ctk.CTkLabel(cam_select_frame,text="Câmera:").pack(side="left",padx=(0,5)); self.cam_index_var=tk.StringVar(value="0")
        self.option_menu_camera = ctk.CTkOptionMenu(cam_select_frame,values=["0","1","2","3"],variable=self.cam_index_var,command=lambda x:self.video_handler.change_camera(x)); self.option_menu_camera.pack(side="left")
        self.video_handler = VideoHandler(frame_video_container,self.hsv_frame,self.serial_manager,self); self.hsv_frame.video_handler=self.video_handler
        self.control_frame = ControlFrame(self.app,self.serial_manager,self,corner_radius=10); self.control_frame.grid(row=0,column=2,sticky="nswe",padx=10,pady=10)
        self.hsv_frame.load_hsv(self.hsv_config); self.control_frame.load_pid_config(self.pid_config)
    def _on_close(self): self._save_app_config(); _ = self.video_handler and self.video_handler.stop(); self.serial_manager.disconnect(); self.app.destroy()
    def _create_menu(self):
        menu_bar = tk.Menu(self.app); arquivo_menu = tk.Menu(menu_bar,tearoff=0); arquivo_menu.add_command(label="Salvar Configuração Agora",command=self._save_app_config); arquivo_menu.add_separator(); arquivo_menu.add_command(label="Sair",command=self._on_close)
        menu_bar.add_cascade(label="Arquivo",menu=arquivo_menu); self.app.config(menu=menu_bar)
    def _create_status_bar(self): self.status_bar = StatusBar(self.app); self.status_bar.grid(row=1,column=0,columnspan=3,sticky="sew")
    def run(self): self.app.mainloop()

if __name__ == "__main__":
    app = OpenBalanceApp()
    app.run()