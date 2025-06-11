### Estrutura Modular do Projeto OpenBalance (v5.0)
# Cada seção abaixo corresponde a um ficheiro Python no diretório `openbalance/`

# --------------------------------------------------
# File: serial_manager.py
# --------------------------------------------------
import serial
import serial.tools.list_ports

DEFAULT_BAUDRATE = 9600

class SerialManager:
    def __init__(self):
        self.serial_conn = None

    def list_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port, baudrate=DEFAULT_BAUDRATE):
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(port, baudrate, timeout=1)
            return True
        except Exception:
            self.serial_conn = None
            return False

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.serial_conn = None

    def send(self, data_str: str):
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        try:
            self.serial_conn.write(data_str.encode('utf-8'))
        except serial.SerialException:
            self.disconnect()
