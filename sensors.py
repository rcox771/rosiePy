import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


TRIGGER_PIN = 17
ECHO_PIN = 18


class Sensor:
    def __init__(self, trigger_pin=17, echo_pin=18):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.setup()

    def setup(self):
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    def set_trigger(self, mode='low'):
        mode = bool(mode=='high')
        GPIO.output(self.trigger_pin, mode)
        time.sleep(0.5)

    def pulse_trigger(self, t=0.00001):
        GPIO.output(self.trigger_pin, True)
        time.sleep(t)
        GPIO.output(self.trigger_pin, False)

    def get_distance(self):
        self.set_trigger()
        self.pulse_trigger()
        start = time.time()
        while GPIO.input(self.echo_pin)==0:
            start = time.time()
        while GPIO.input(self.echo_pin)==1:
            stop = time.time()
            if stop-start >= .04:
                print('too close! aghhhhhhhhh')
                stop = start
                break
        elapsed = stop - start
        dist = elapsed * 34326
        dist = dist/2.
        print('distance: %.1f cm' % dist)
        return dist


if __name__ == '__main__':
    sensor = Sensor()
    try:
        while True:
            dist = sensor.get_distance()
            time.sleep(.5)
    except KeyboardInterrupt:
        GPIO.cleanup()


