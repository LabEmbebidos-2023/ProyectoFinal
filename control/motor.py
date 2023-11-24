import math
import RPi.GPIO as GPIO
import numpy as np

CODES_PER_REV = 54
CMS_PER_REV = 2.0 * math.pi * 3.25  # cms
SPEED_ROLLING_AVERAGE_WINDOW = 10


class Motor:
    def __init__(self, forward_pin, backward_pin, encoder_pin, update_time_period):
        GPIO.setmode(GPIO.BCM)
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin
        self.encoder_pin = encoder_pin
        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.backward_pin, GPIO.OUT)
        self.forward_pwm = GPIO.PWM(self.forward_pin, 1000)
        self.backward_pwm = GPIO.PWM(self.backward_pin, 1000)
        self.forward_pwm.start(0)
        self.backward_pwm.start(0)
        self.encoder_count = 0
        self.last_set = 0
        self.speed = 0
        self.last_raw_speeds = []
        self.last_distance = 0
        self.update_time_period = update_time_period

        GPIO.setup(self.encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self.encoder_interrupt)

    def update(self):
        current_distance = self.get_distance()
        raw_speed = (current_distance - self.last_distance) / self.update_time_period

        if len(self.last_raw_speeds) == SPEED_ROLLING_AVERAGE_WINDOW:
            del self.last_raw_speeds[0]

        self.last_raw_speeds.append(raw_speed)

        self.speed = np.mean(self.last_raw_speeds)

        self.last_distance = current_distance

    def encoder_interrupt(self, channel):
        #self.encoder_count += math.copysign(1, self.last_set)
        if self.last_set > 0:
            self.encoder_count += 1
        else:
            self.encoder_count -= 1
        print(self.last_set)

    def get_distance(self):
        revs = self.encoder_count / CODES_PER_REV

        return revs * CMS_PER_REV

    def set(self, percent_out):
        motor_out = abs(max(-1.0, min(percent_out, 1.0)))
        self.last_set = percent_out

        if percent_out > 0.0:
            self.forward_pwm.ChangeDutyCycle(motor_out * 100.0)
            self.backward_pwm.ChangeDutyCycle(0)
        elif percent_out < 0.0:
            self.forward_pwm.ChangeDutyCycle(0)
            self.backward_pwm.ChangeDutyCycle(motor_out * 100.0)
        else:
            self.forward_pwm.ChangeDutyCycle(0)
            self.backward_pwm.ChangeDutyCycle(0)
