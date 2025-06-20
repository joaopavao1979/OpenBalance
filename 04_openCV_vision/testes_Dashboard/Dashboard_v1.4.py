# OpenBalance Dashboard
# Descrição: Dashboard para o projeto OpenBalance, sistema de controlo de bola equilibrada numa plataforma inclinável usando visão computacional e PID.
# Autor: João Pavão
# Finalidade: Fornecer interface gráfica para ajuste de parâmetros HSV, controlo PID, monitorização de vídeo em tempo real (opcional), calibração e envio de comandos ao Arduino.
# Uso: Código aberto, disponibilizado livremente no GitHub sob licença MIT.

import customtkinter as ctk  # Biblioteca CustomTkinter para GUI estilizada
import serial  # Biblioteca para comunicação serial com Arduino
import serial.tools.list_ports  # Ferramenta para listar portas seriais disponíveis
import csv  # Biblioteca para manipulação de arquivos CSV
from tkinter import filedialog, messagebox  # Diálogos de arquivo e caixas de mensagem
import tkinter as tk  # GUI nativa do Tkinter para menus nativos

# Adicionar ao início do código, com as outras importações
import cv2
from PIL import Image, ImageTk

# Configuração de tema e modo do CustomTkinter
ctk.set_appearance_mode("dark")  # Modo escuro
ctk.set_default_color_theme("dark-blue")  # Tema de cores padrão azul escuro

# Criação da janela principal
app = ctk.CTk()
app.title("OpenBalance")  # Título da janela
app.geometry("1200x700")  # Tamanho inicial da janela

# ----------------- MENU SUPERIOR -----------------
menu_bar = tk.Menu(app)
# Menu Arquivo
arquivo_menu = tk.Menu(menu_bar, tearoff=0)
arquivo_menu.add_command(label="Salvar Configuração", command=lambda: messagebox.showinfo("Salvar", "Funcionalidade Salvar Configuração ainda não implementada"))
arquivo_menu.add_command(label="Carregar Configuração", command=lambda: messagebox.showinfo("Carregar", "Funcionalidade Carregar Configuração ainda não implementada"))
arquivo_menu.add_separator()
arquivo_menu.add_command(label="Sair", command=app.quit)
menu_bar.add_cascade(label="Arquivo", menu=arquivo_menu)
# Menu Configurações
config_menu = tk.Menu(menu_bar, tearoff=0)
config_menu.add_command(label="Resetar Valores", command=lambda: messagebox.showinfo("Reset", "Resetar valores de HSV e PID ainda não implementado"))
menu_bar.add_cascade(label="Configurações", menu=config_menu)
# Menu Ajuda
ajuda_menu = tk.Menu(menu_bar, tearoff=0)
ajuda_menu.add_command(label="Sobre", command=lambda: messagebox.showinfo(
    "Sobre",  # Título da caixa de mensagem
    "OpenBalance Dashboard\nAutor: João Pavão\nVersão: 1.0"  # Texto da mensagem com quebras de linha
))
menu_bar.add_cascade(label="Ajuda", menu=ajuda_menu)
app.config(menu=menu_bar)  # Associar a barra de menus à janela

# ----------------- VARIÁVEIS DE FEEDBACK -----------------
# Variáveis StringVar para exibir valores de status dos servos e PID
servo_x_var = ctk.StringVar(master=app, value="Servo X: 0.0°")
servo_y_var = ctk.StringVar(master=app, value="Servo Y: 0.0°")
pid_x_var = ctk.StringVar(master=app, value="PID X: 0.00")
pid_y_var = ctk.StringVar(master=app, value="PID Y: 0.00")

# ----------------- PORTAS SERIAIS E FUNÇÕES DE CONEXÃO -----------------
# Obter lista de portas seriais disponíveis
portas_disponiveis = [porta.device for porta in serial.tools.list_ports.comports()]
porta_selecionada = ctk.StringVar(master=app, value=portas_disponiveis[0] if portas_disponiveis else "")
serial_conn = None  # Inicialmente sem conexão

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

# ----------------- FUNÇÃO PARA SALVAR DADOS PID EM CSV -----------------
def salvar_csv_pid():
    """Salva os valores atuais de PID X e Y num arquivo CSV."""
    valor_x = pid_x_var.get()  # Recuperar valor atual de PID X
    valor_y = pid_y_var.get()  # Recuperar valor atual de PID Y
    filepath = filedialog.asksaveasfilename(
        defaultextension=".csv", filetypes=[("CSV Files", "*.csv")]
    )
    if filepath:
        try:
            with open(filepath, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["PID_X", "PID_Y"])  # Cabeçalho
                writer.writerow([valor_x, valor_y])  # Dados
            messagebox.showinfo("Salvo", f"Dados salvos em: {filepath}")
        except Exception as e:
            messagebox.showerror("Erro", f"Falha ao salvar CSV: {e}")

# ======================= FRAME ESQUERDO - HSV =======================
frame_hsv = ctk.CTkFrame(app, corner_radius=10)
frame_hsv.pack(side="left", fill="y", padx=10, pady=10)
ctk.CTkLabel(frame_hsv, text="Deteção de Cor (HSV)", font=("Arial", 16)).pack(pady=10)
# Listas para armazenar sliders e valores
hsv_sliders = []  # Lista de widgets Slider para HSV
hsv_values = []  # Lista de StringVars para exibir valor numérico do slider
labels = ["H Min", "H Max", "S Min", "S Max", "V Min", "V Max"]
for label in labels:
    ctk.CTkLabel(frame_hsv, text=label).pack()
    val_var = ctk.StringVar(master=app, value="0")  # Valor inicial 0
    # Slider: de 0 a 255, atualiza o StringVar com valor inteiro
    slider = ctk.CTkSlider(
        frame_hsv,
        from_=0,
        to=255,
        number_of_steps=255,
        command=lambda val, var=val_var: var.set(f"{int(float(val))}")
    )
    slider.pack(fill="x", pady=5)
    value_label = ctk.CTkLabel(frame_hsv, textvariable=val_var)
    value_label.pack()
    hsv_sliders.append(slider)
    hsv_values.append(val_var)
# Botões para escolher cor alvo (Vermelho, Laranja, Verde)
for cor in ["Vermelho", "Laranja", "Verde"]:
    ctk.CTkButton(frame_hsv, text=cor).pack(pady=2)
ctk.CTkCheckBox(frame_hsv, text="Mostrar máscara").pack(pady=10)

# ======================= FRAME CENTRO - VÍDEO =======================
frame_video = ctk.CTkFrame(app, corner_radius=10)
frame_video.pack(side="left", fill="both", expand=True, padx=10, pady=10)
ctk.CTkLabel(frame_video, text="Área de Visualização", font=("Arial", 16)).pack(pady=10)

# Substituir o placeholder por um canvas para o vídeo
video_canvas = ctk.CTkCanvas(frame_video, width=640, height=480, bg="gray20", highlightthickness=0)
video_canvas.pack(pady=10)

# Inicializar a webcam
cap = cv2.VideoCapture(1)  # 0 para a webcam padrão; pode ser 1, 2, etc., para outras câmaras
if not cap.isOpened():
    messagebox.showerror("Erro", "Não foi possível aceder à webcam. Verifique a ligação.")
    cap = None

def atualizar_video():
    """Atualiza o feed de vídeo no canvas."""
    if cap and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            # Converter frame BGR (OpenCV) para RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Redimensionar para o tamanho do canvas
            img = Image.fromarray(frame_rgb)
            img = img.resize((640, 480), Image.Resampling.LANCZOS)
            img_tk = ImageTk.PhotoImage(img)
            # Atualizar o canvas
            video_canvas.create_image(0, 0, anchor="nw", image=img_tk)
            video_canvas.image = img_tk  # Evitar garbage collection
        # Agendar próxima atualização
        app.after(10, atualizar_video)
    else:
        messagebox.showerror("Erro", "Webcam não está disponível.")

# Iniciar o feed de vídeo
if cap:
    atualizar_video()
    
# Frame interno para status de servos e PID
status_frame = ctk.CTkFrame(frame_video, corner_radius=5)
status_frame.pack(pady=5)
ctk.CTkLabel(status_frame, textvariable=servo_x_var).grid(row=0, column=0, padx=10, pady=(5,2), sticky="w")
ctk.CTkLabel(status_frame, textvariable=servo_y_var).grid(row=1, column=0, padx=10, pady=(2,5), sticky="w")
ctk.CTkLabel(status_frame, textvariable=pid_x_var).grid(row=0, column=1, padx=10, pady=(5,2), sticky="w")
ctk.CTkLabel(status_frame, textvariable=pid_y_var).grid(row=1, column=1, padx=10, pady=(2,5), sticky="w")
# Rótulo e menu para selecionar modo de controlo
ctk.CTkLabel(frame_video, text="Modo de Controlo").pack(pady=5)
ctrl_menu = ctk.CTkOptionMenu(frame_video, values=["Manual", "Círculo", "Oito", "Quadrado", "Senoide"])  # Menu padrão
ctrl_menu.pack(pady=5)
# Botões de execução/parada de trajetória
ctk.CTkButton(frame_video, text="Executar Trajetória").pack(pady=10)
ctk.CTkButton(frame_video, text="Parar Movimento").pack(pady=5)

# ======================= FRAME DIREITO - PID/SCROLL =======================
# Frame scrollable para evitar perda de botões em janelas pequenas
frame_pid_container = ctk.CTkScrollableFrame(app, corner_radius=10, width=250)
frame_pid_container.pack(side="left", fill="both", padx=10, pady=10, expand=False)
# Frame interno com conteúdos do PID
frame_pid = ctk.CTkFrame(frame_pid_container, corner_radius=10)
frame_pid.pack(fill="both", expand=True, padx=0, pady=0)
ctk.CTkLabel(frame_pid, text="Controlador PID", font=("Arial", 16)).pack(pady=10)
pid_values = []  # StringVars para exibir valores atuais do PID
for param in ["Kp", "Ki", "Kd"]:
    ctk.CTkLabel(frame_pid, text=param).pack()
    val_var = ctk.StringVar(master=app, value="0.00")  # Valor inicial 0.00
    # Slider PID: de 0 a 10, atualiza StringVar com 2 casas decimais
    slider = ctk.CTkSlider(
        frame_pid,
        from_=0,
        to=10,
        number_of_steps=100,
        command=lambda val, var=val_var: var.set(f"{float(val):.2f}")
    )
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
# Botões de controlo principais
ctk.CTkButton(frame_pid, text="Iniciar Seguimento").pack(pady=5)
ctk.CTkButton(frame_pid, text="Parar Seguimento").pack(pady=5)
ctk.CTkButton(frame_pid, text="Ligar Motores").pack(pady=5)
ctk.CTkButton(frame_pid, text="Desligar Motores").pack(pady=5)

# ======================= SEÇÃO CONEXÃO ARDUINO =======================
ctk.CTkLabel(frame_pid, text="Conexão Arduino", font=("Arial", 14)).pack(pady=(20, 5))
option_menu_porta = ctk.CTkOptionMenu(
    frame_pid,
    values=portas_disponiveis,
    variable=porta_selecionada,
    fg_color="green",  # Cor de fundo verde
    button_color="green",  # Cor do botão verde
    button_hover_color="darkgreen"  # Cor ao passar o mouse
)
option_menu_porta.pack(pady=5)
ctk.CTkButton(frame_pid, text="Ligar Arduino", command=ligar_serial, fg_color="green", hover_color="darkgreen").pack(pady=5)
ctk.CTkButton(frame_pid, text="Desligar Arduino", command=desligar_serial, fg_color="green", hover_color="darkgreen").pack(pady=5)

# ----------------- BOTÃO PARA SALVAR PID EM CSV -----------------
ctk.CTkLabel(frame_pid, text="", height=10).pack()  # Espaçamento extra
ctk.CTkButton(frame_pid, text="Guardar PID em CSV", command=salvar_csv_pid, fg_color="orange", hover_color="darkorange").pack(pady=5)

# Adicionar no final do código, antes de app.mainloop()
def fechar_app():
    """Fecha a aplicação e liberta a webcam."""
    if cap and cap.isOpened():
        cap.release()
    app.destroy()

app.protocol("WM_DELETE_WINDOW", fechar_app)  # Chamar ao fechar a janela
# ======================= INICIAR APP =======================
app.mainloop()
