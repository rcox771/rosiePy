import RPi.GPIO as GPIO
import time
import numpy as np
from sensors import *
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
from random import choice

A_FWD = 10
A_BCK = 9
B_FWD = 8
B_BCK = 7

STOP = 0



class Pin:
    def __init__(self, num, init_mode=GPIO.OUT, freq=40):
        self.num = num
        self.freq = freq #times/second
        GPIO.setup(self.num, init_mode)
        self.signal = None
        self.set_freq(self.freq)
        self.signal.start(0)
        self.duty_cycle = 0
        self.last_cmd = [0, 0]

    def set_freq(self, freq):
        self.signal = GPIO.PWM(self.num, freq)
        self.freq = freq

    def set_duty_cycle(self, cycles):
        self.signal.ChangeDutyCycle(cycles)
        self.duty_cycle = cycles



class Rosie:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.pins = {
            'motors' : {
                'A': {
                    '+': Pin(10),
                    '-': Pin(9)
                },
                'B': {
                    '+': Pin(8),
                    '-': Pin(7)
                }
            }

        }
        self.sensor = Sensor()


    def interp_cmd(self, A, B, t=None):
        abs_a = np.abs(A)
        abs_b = np.abs(B)
        if A > 0:
            self.pins['motors']['A']['-'].set_duty_cycle(STOP)
            self.pins['motors']['A']['+'].set_duty_cycle(abs_a)
        elif A < 0:
            self.pins['motors']['A']['-'].set_duty_cycle(abs_a)
            self.pins['motors']['A']['+'].set_duty_cycle(STOP)
        else:
            self.pins['motors']['A']['-'].set_duty_cycle(STOP)
            self.pins['motors']['A']['+'].set_duty_cycle(STOP)

        if B > 0:
            self.pins['motors']['B']['-'].set_duty_cycle(STOP)
            self.pins['motors']['B']['+'].set_duty_cycle(abs_b)
        elif B < 0:
            self.pins['motors']['B']['-'].set_duty_cycle(abs_b)
            self.pins['motors']['B']['+'].set_duty_cycle(STOP)
        else:
            self.pins['motors']['B']['-'].set_duty_cycle(STOP)
            self.pins['motors']['B']['+'].set_duty_cycle(STOP)

        if t:
            time.sleep(t)
            self.brake()

    def backward(self, A=-40, B=40, t=None):
        self.interp_cmd(A, B)
        if t:
            time.sleep(t)
            self.brake()

    def forward(self, A=40, B=-40, t=None):
        self.interp_cmd(A, B)
        if t:
            time.sleep(t)
            self.brake()

    def rand_direction(self, t=.6):
        chosen = choice([self.turn_left, self.turn_right])
        chosen(t=2.)
        time.sleep(t)
        self.brake()


    def explore(self):
        try:
            while True:
                dist = self.sensor.get_distance()
                if dist <= 30:
                    self.brake()
                    self.backward(t=1)

                    self.rand_direction()
                else:
                    self.forward()


        except KeyboardInterrupt:
            self.brake()

    def turn_right(self, A=40, B=40, t=None):
        self.interp_cmd(A, B)
        if t:
            time.sleep(t)
            self.brake()

    def turn_left(self, A=-40, B=-40, t=None):
        self.interp_cmd(A, B)
        if t:
            time.sleep(t)
            self.brake()

    def brake(self):
        self.interp_cmd(0, 0)








if __name__ == '__main__':
    r = Rosie()

    GPIO.cleanup()



