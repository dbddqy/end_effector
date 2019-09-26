import RPi.GPIO as GPIO
import time


def GPIO_init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)


class Motor:
    def __init__(self, ports):
        self.ports = ports
    def step(self, step, clockwise):
        if clockwise:
            arr = [0,1,2,3]
        else:
            arr = [3,2,1,0]

        for p in self.ports:
            GPIO.setup(p,GPIO.OUT)

        for x in range(step):
            for j in arr:
                time.sleep(0.004)
                for i in range(4):
                    if i == j:
                        GPIO.output(self.ports[i],True)
                    else:
                        GPIO.output(self.ports[i],False)
