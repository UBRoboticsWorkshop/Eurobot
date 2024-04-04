import pigpio
# run this in bash: sudo pigpiod



class servo:
    def __init__(self, pin=12, upper_limit=1900, lower_limit=1100, angle_limit=180):
        self._lower_limit = lower_limit
        self._x = float(upper_limit - lower_limit) / float(angle_limit)
        self._angle_limit = angle_limit
        self._pin = pin

        self.pwm = pigpio.pi()
        if not self.pwm.connected:
            exit()
        self.pwm.set_PWM_frequency(self._pin, 50)
        #self.pwm.set_PWM_range(self._pin, 255)
        self.set_angle()

    def left(self,add_angle=5):
        self.angle += add_angle
        self.set_angle(self.angle)
    
    def right(self,sub_angle=5):
        self.angle -= sub_angle
        self.set_angle(self.angle)

    def stop(self):
        self.pwm.set_PWM_dutycycle(self._pin, 0)

    def set_angle(self,angle=90):
        self.angle = angle
        if (self.angle < 0):
            self.angle = 0
        elif (self.angle > self._angle_limit):
            self.angle = self._angle_limit

        print('current angle:', self.angle) # debug
        print('PWM:', self.get_pwm(self.angle))

        #self.pwm.set_PWM_dutycycle(self._pin, self.get_pwm(self.angle))
        self.pwm.set_servo_pulsewidth(self._pin, self.get_pwm(self.angle))

    def get_pwm(self,angle): # Remap the angele to pwm
        return int(float(self._lower_limit) + float(angle) * self._x)
