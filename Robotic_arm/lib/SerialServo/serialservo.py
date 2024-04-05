import pigpio



class serialservo:
    def __init__(self):
        self._ID = ID
        
        self.servo = pigpio.pi()
        if not self.servo.connected:
            exit()
        self.servo.serial_open("/dev/ttyAMA0", 1000000)# not working as the BR is not supported
