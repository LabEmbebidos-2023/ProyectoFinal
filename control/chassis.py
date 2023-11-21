import math

import board
import adafruit_mpu6050
import numpy as np
import time
from .motor import Motor

from control.madgwick import MadgwickAHRS


class Chassis:
    motorL1Pin = 13
    motorL2Pin = 19
    motorR1Pin = 12
    motorR2Pin = 16

    gyroCalibration = [0, 0, 0]

    def __init__(self, imu_sample_period, odometry_update_period):
        self.left_motor = Motor(self.motorR1Pin, self.motorR2Pin, 17, odometry_update_period)
        self.right_motor = Motor(self.motorL1Pin, self.motorL2Pin, 27, odometry_update_period)
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.mpu = adafruit_mpu6050.MPU6050(i2c)
        self.ahrs = MadgwickAHRS(sampleperiod=imu_sample_period)
        self.odometry_update_period = odometry_update_period
        self.imu_sample_period = imu_sample_period
        self.pose = [0, 0, 0]

        calibrationStartTime = time.time()
        calibrationTimeElapsed = 0
        calibrationTime = 5
        calibrationDataCount = 0
        print("Calibrating IMU")
        time.sleep(1)
        while calibrationTimeElapsed < calibrationTime:
            gyro = self.mpu.gyro
            self.gyroCalibration[0] += gyro[0]
            self.gyroCalibration[1] += gyro[1]
            self.gyroCalibration[2] += gyro[2]
            calibrationDataCount += 1
            print("Calibrating IMU %d" % calibrationTimeElapsed)
            time.sleep(0.1)
            calibrationTimeElapsed = time.time() - calibrationStartTime
        print("Done calibrating")
        print("Done asdasdsafa")

        self.gyroCalibration[0] /= calibrationDataCount
        self.gyroCalibration[1] /= calibrationDataCount
        self.gyroCalibration[2] /= calibrationDataCount

    def update_imu(self):
        gyro = np.subtract(self.mpu.gyro, self.gyroCalibration)
        accel = self.mpu.acceleration
        self.ahrs.update_imu(gyro, accel)

    def update_odometry(self):
        self.right_motor.update()
        self.left_motor.update()

        q = self.ahrs.quaternion

        roll = math.atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)

        v = (self.right_motor.speed + self.left_motor.speed) / 2.0
        w = (self.right_motor.speed - self.left_motor.speed) / 10.0

        local_speeds = np.array([[v], [0]])
        m = np.array([[math.cos(roll), -math.sin(roll)], [math.sin(roll), math.cos(roll)]])

        global_speeds = np.dot(m, local_speeds)

        self.pose[0] += global_speeds[0] * self.odometry_update_period
        self.pose[1] += global_speeds[1] * self.odometry_update_period
        self.pose[2] = roll  # Heading is roll based on how the 6050 ys mounted

    def set_speeds(self, vx, w):
        self.left_motor.set(vx - w)
        self.right_motor.set(vx + w)
