# --------------------------------------------------
# File: pid_controller.py
# --------------------------------------------------
class PIDController:
    def __init__(self, Kp: float, Ki: float, Kd: float, setpoint: float = 0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._last_error = 0
        self._integral = 0

    def update(self, process_variable: float) -> float:
        error = self.setpoint - process_variable
        self._integral += error
        derivative = error - self._last_error
        self._last_error = error
        return self.Kp * error + self.Ki * self._integral + self.Kd * derivative

    def set_gains(self, Kp: float, Ki: float, Kd: float):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self._integral = 0
        self._last_error = 0
