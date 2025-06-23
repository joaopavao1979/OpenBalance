# OpenBalance Dashboard
# Descrição: Dashboard para o projeto OpenBalance, sistema de controlo de bola equilibrada numa plataforma inclinável usando visão computacional e PID.
# Autor: João Pavão
# Finalidade: Fornecer interface gráfica para ajuste de parâmetros HSV, controlo PID, monitorização de vídeo em tempo real (opcional), calibração e envio de comandos ao Arduino.
# Uso: Código aberto, disponibilizado livremente no GitHub sob licença MIT.

import customtkinter as ctk
import serial
import serial.tools.list_ports
import csv
from tkinter import filedialog, messagebox

# Inicializar o modo escuro
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("dark-blue")

# Criar a janela principal
app = ctk.CTk()
app.title("OpenBalance Dashboard")
app.geometry("1200x700")

# ----------------- Estrutura de Configurações Feedback -----------------
# Variáveis para exibir valores de status (servos e PID)
servo_x_var = ctk.StringVar(master=app, value="Servo X: 0.0°")
servo_y_var = ctk.StringVar(master=app, value="Servo Y: 0.0°")
pid_x_var = ctk.StringVar(master=app, value="PID X: 0.00")
pid_y_var = ctk.StringVar(master=app, value="PID Y: 0.00")

# ----------------- Configuração da Porta Serial (após criar a janela) -----------------
# Listar portas disponíveis e criar StringVar associado à janela
portas_disponiveis = [porta.device for porta in serial.tools.list_ports.comports()]
porta_selecionada = ctk.StringVar(master=app, value=portas_disponiveis[0] if portas_disponiveis else "")
serial_conn = None

def ligar_serial():
    """Abre conexão serial com a porta selecionada."""
    global serial_conn
    try:
        serial_conn = serial.Serial(porta_selecionada.get(), 9600, timeout=1)
        messagebox.showinfo("Conexão", f"Ligado à porta {porta_selecionada.get()}")
    except Exception as e:
        messagebox.showerror("Erro", f"Falha ao ligar: {e}")

def desligar_serial():
    """Fecha a conexão serial caso esteja aberta."""
    global serial_conn
    if serial_conn and serial_conn.is_open:
        serial_conn.close()
        messagebox.showinfo("Conexão", "Conexão serial terminada.")

# ----------------- Função para salvar PID em CSV -----------------
def salvar_csv_pid():
    """Salva os valores atuais de PID X e Y num arquivo CSV."""
    # Obter valores numéricos ou texto completo
    valor_x = pid_x_var.get()
    valor_y = pid_y_var.get()
    filepath = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV Files", "*.csv")])
    if filepath:
        try:
            with open(filepath, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["PID_X", "PID_Y"])
                writer.writerow([valor_x, valor_y])
            messagebox.showinfo("Salvo", f"Dados salvos em: {filepath}")
        except Exception as e:
            messagebox.showerror("Erro", f"Falha ao salvar CSV: {e}")

# ======================= FRAME ESQUERDO - HSV =======================
frame_hsv = ctk.CTkFrame(app, corner_radius=10)
frame_hsv.pack(side="left", fill="y", padx=10, pady=10)

ctk.CTkLabel(frame_hsv, text="Deteção de Cor (HSV)", font=("Arial", 16)).pack(pady=10)

hsv_sliders = []  # Sliders para ajustar limites HSV
hsv_values = []   # StringVars para mostrar valor numérico de cada slider
labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
for label in labels:
    ctk.CTkLabel(frame_hsv, text=label).pack()
    val_var = ctk.StringVar(master=app, value="0")
    # Slider ajusta o valor e atualiza o StringVar correspondente
    slider = ctk.CTkSlider(frame_hsv, from_=0, to=255, number_of_steps=255,
                            command=lambda val, var=val_var: var.set(f"{int(float(val))}"))
    slider.pack(fill="x", pady=5)
    value_label = ctk.CTkLabel(frame_hsv, textvariable=val_var)
    value_label.pack()
    hsv_sliders.append(slider)
    hsv_values.append(val_var)

# Botões para selecionar cor alvo (ex: Vermelho, Laranja, Verde)
for cor in ["Vermelho", "Laranja", "Verde"]:
    ctk.CTkButton(frame_hsv, text=cor).pack(pady=2)

ctk.CTkCheckBox(frame_hsv, text="Mostrar máscara").pack(pady=10)

# ======================= FRAME CENTRO - VÍDEO =======================
frame_video = ctk.CTkFrame(app, corner_radius=10)
frame_video.pack(side="left", fill="both", expand=True, padx=10, pady=10)

ctk.CTkLabel(frame_video, text="Área de Visualização", font=("Arial", 16)).pack(pady=10)

# Placeholder para exibir quadro de vídeo da webcam (será substituído por imagem gerada via OpenCV)
video_canvas = ctk.CTkLabel(frame_video, text="[Vídeo Aqui]", width=640, height=480, fg_color="gray20")
video_canvas.pack(pady=10)

# Frame para mostrar status de servos e saída PID
status_frame = ctk.CTkFrame(frame_video, corner_radius=5)
status_frame.pack(pady=5)
ctk.CTkLabel(status_frame, textvariable=servo_x_var).grid(row=0, column=0, padx=10, pady=(5,2), sticky="w")
ctk.CTkLabel(status_frame, textvariable=servo_y_var).grid(row=1, column=0, padx=10, pady=(2,5), sticky="w")
ctk.CTkLabel(status_frame, textvariable=pid_x_var).grid(row=0, column=1, padx=10, pady=(5,2), sticky="w")
ctk.CTkLabel(status_frame, textvariable=pid_y_var).grid(row=1, column=1, padx=10, pady=(2,5), sticky="w")

ctk.CTkLabel(frame_video, text="Modo de Controlo").pack(pady=5)
ctrl_menu = ctk.CTkOptionMenu(frame_video, values=["Manual", "Círculo", "Oito", "Quadrado", "Senoide"])  # Usa estilo padrão
ctrl_menu.pack(pady=5)

ctk.CTkButton(frame_video, text="Executar Trajetória").pack(pady=10)
ctk.CTkButton(frame_video, text="Parar Movimento").pack(pady=5)

# ======================= FRAME DIREITO - PID (AGORA ROLARÁ EM CASO DE REDUÇÃO) =======================
# Criar um frame de rolagem para o painel direito, evitando ocultar botões quando a janela for reduzida
frame_pid_container = ctk.CTkScrollableFrame(app, corner_radius=10, width=250)
frame_pid_container.pack(side="left", fill="both", padx=10, pady=10, expand=False)

# Dentro do frame scrollable, criar o conteúdo do PID
frame_pid = ctk.CTkFrame(frame_pid_container, corner_radius=10)
frame_pid.pack(fill="both", expand=True, padx=0, pady=0)

ctk.CTkLabel(frame_pid, text="Controlador PID", font=("Arial", 16)).pack(pady=10)

pid_values = []  # StringVars para exibir valores atuais do PID
for param in ["Kp", "Ki", "Kd"]:
    ctk.CTkLabel(frame_pid, text=param).pack()
    val_var = ctk.StringVar(master=app, value="0.00")
    # Slider ajusta valor e atualiza StringVar formatado com 2 casas decimais
    slider = ctk.CTkSlider(frame_pid, from_=0, to=10, number_of_steps=100,
                            command=lambda val, var=val_var: var.set(f"{float(val):.2f}"))
    slider.pack(fill="x", pady=5)
    value_label = ctk.CTkLabel(frame_pid, textvariable=val_var)
    value_label.pack()
    pid_values.append(val_var)

ctk.CTkLabel(frame_pid, text="Max Integral").pack()
ctk.CTkSlider(frame_pid, from_=0, to=500, number_of_steps=100).pack(fill="x", pady=5)

ctk.CTkLabel(frame_pid, text="Offset X / Y").pack(pady=10)
ctk.CTkEntry(frame_pid, placeholder_text="Offset X").pack(pady=2)
ctk.CTkEntry(frame_pid, placeholder_text="Offset Y").pack(pady=2)
ctk.CTkButton(frame_pid, text="Auto Calibrar").pack(pady=5)

# Botões de controlo
ctk.CTkButton(frame_pid, text="Iniciar Seguimento").pack(pady=5)
ctk.CTkButton(frame_pid, text="Parar Seguimento").pack(pady=5)
ctk.CTkButton(frame_pid, text="Ligar Motores").pack(pady=5)
ctk.CTkButton(frame_pid, text="Desligar Motores").pack(pady=5)

# ======================= SEPARADOR VISUAL PARA CONEXÃO ARDUINO =======================
ctk.CTkLabel(frame_pid, text="Conexão Arduino", font=("Arial", 14)).pack(pady=(20, 5))

# Dropdown de portas disponíveis
option_menu_porta = ctk.CTkOptionMenu(frame_pid, values=portas_disponiveis, variable=porta_selecionada, fg_color="green", button_color="green", button_hover_color="darkgreen")
option_menu_porta.pack(pady=5)
# Botões verdes para ligar e desligar
ctk.CTkButton(frame_pid, text="Ligar Arduino", command=ligar_serial, fg_color="green", hover_color="darkgreen").pack(pady=5)
ctk.CTkButton(frame_pid, text="Desligar Arduino", command=desligar_serial, fg_color="green", hover_color="darkgreen").pack(pady=5)

# ======================= BOTÃO PARA SALVAR PID EM CSV =======================
ctk.CTkLabel(frame_pid, text="", height=10).pack()  # Espaçamento extra
ctk.CTkButton(frame_pid, text="Guardar PID em CSV", command=salvar_csv_pid, fg_color="orange", hover_color="darkorange").pack(pady=5)

# ======================= INICIAR APP =======================
app.mainloop()
