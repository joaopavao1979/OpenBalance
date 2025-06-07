import customtkinter as ctk

# Inicializar o modo escuro
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("dark-blue")

# Criar a janela principal
app = ctk.CTk()
app.title("OpenBalance Dashboard")
app.geometry("1200x700")

# ----------------- Estrutura de Configurações Feedback -----------------
# Variáveis para exibir valores de status (servos e PID)
servo_x_var = ctk.StringVar(value="Servo X: 0.0°")
servo_y_var = ctk.StringVar(value="Servo Y: 0.0°")
pid_x_var = ctk.StringVar(value="PID X: 0.00")
pid_y_var = ctk.StringVar(value="PID Y: 0.00")

# ======================= FRAME ESQUERDO - HSV =======================
frame_hsv = ctk.CTkFrame(app, corner_radius=10)
frame_hsv.pack(side="left", fill="y", padx=10, pady=10)

ctk.CTkLabel(frame_hsv, text="Deteção de Cor (HSV)", font=("Arial", 16)).pack(pady=10)

hsv_sliders = []
labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
for label in labels:
    ctk.CTkLabel(frame_hsv, text=label).pack()
    slider = ctk.CTkSlider(frame_hsv, from_=0, to=255, number_of_steps=255)
    slider.pack(fill="x", pady=5)
    hsv_sliders.append(slider)

# Botões de cores em português
for cor in ["Vermelho", "Laranja", "Verde"]:
    ctk.CTkButton(frame_hsv, text=cor).pack(pady=2)

ctk.CTkCheckBox(frame_hsv, text="Mostrar máscara").pack(pady=10)

# ======================= FRAME CENTRO - VÍDEO =======================
frame_video = ctk.CTkFrame(app, corner_radius=10)
frame_video.pack(side="left", fill="both", expand=True, padx=10, pady=10)

ctk.CTkLabel(frame_video, text="Área de Visualização", font=("Arial", 16)).pack(pady=10)

# Placeholder para vídeo (será substituído por imagem da webcam posteriormente)
video_canvas = ctk.CTkLabel(frame_video, text="[Vídeo Aqui]", width=640, height=480, fg_color="gray20")
video_canvas.pack(pady=10)

# Exibição de valores de status abaixo da área de vídeo
status_frame = ctk.CTkFrame(frame_video, corner_radius=5)
status_frame.pack(pady=5)
ctk.CTkLabel(status_frame, textvariable=servo_x_var).grid(row=0, column=0, padx=10, pady=(5,2), sticky="w")
ctk.CTkLabel(status_frame, textvariable=servo_y_var).grid(row=1, column=0, padx=10, pady=(2,5), sticky="w")
ctk.CTkLabel(status_frame, textvariable=pid_x_var).grid(row=0, column=1, padx=10, pady=(5,2), sticky="w")
ctk.CTkLabel(status_frame, textvariable=pid_y_var).grid(row=1, column=1, padx=10, pady=(2,5), sticky="w")

ctk.CTkLabel(frame_video, text="Modo de Controlo").pack(pady=5)
ctk.CTkOptionMenu(frame_video, values=["Manual", "Círculo", "Oito", "Quadrado", "Senoide"]).pack()

ctk.CTkButton(frame_video, text="Executar Trajetória").pack(pady=10)
ctk.CTkButton(frame_video, text="Parar Movimento").pack(pady=5)

# ======================= FRAME DIREITO - PID =======================
frame_pid = ctk.CTkFrame(app, corner_radius=10)
frame_pid.pack(side="left", fill="y", padx=10, pady=10)

ctk.CTkLabel(frame_pid, text="Controlador PID", font=("Arial", 16)).pack(pady=10)
for param in ["Kp", "Ki", "Kd"]:
    ctk.CTkLabel(frame_pid, text=param).pack()
    ctk.CTkSlider(frame_pid, from_=0, to=10, number_of_steps=100).pack(fill="x", pady=5)

ctk.CTkLabel(frame_pid, text="Max Integral").pack()
ctk.CTkSlider(frame_pid, from_=0, to=500, number_of_steps=100).pack(fill="x", pady=5)

ctk.CTkLabel(frame_pid, text="Offset X / Y").pack(pady=10)
ctk.CTkEntry(frame_pid, placeholder_text="Offset X").pack(pady=2)
ctk.CTkEntry(frame_pid, placeholder_text="Offset Y").pack(pady=2)
ctk.CTkButton(frame_pid, text="Auto Calibrar").pack(pady=5)

# Botões traduzidos para português
ctk.CTkButton(frame_pid, text="Iniciar Seguimento").pack(pady=5)
ctk.CTkButton(frame_pid, text="Parar Seguimento").pack(pady=5)
ctk.CTkButton(frame_pid, text="Ligar Motores").pack(pady=5)
ctk.CTkButton(frame_pid, text="Desligar Motores").pack(pady=5)

# ======================= INICIAR APP =======================
app.mainloop()
