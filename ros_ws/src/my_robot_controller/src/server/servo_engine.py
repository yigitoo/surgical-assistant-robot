class ServoEngine:
    def __init__(self):
        self.servos: list[dict] = [{}]
        self.servo_names = []
        self.last_servo_values = {}

    def setup(self):
        self.servo_1