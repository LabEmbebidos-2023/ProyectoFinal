# -*- coding: utf-8 -*-
"""
    Copyright (c) 2015 Jonas Böer, jonas.boeer@student.kit.edu

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import warnings
import numpy as np
from numpy.linalg import norm
import quaternion as quat


class MadgwickAHRS:
    samplePeriod = 1 / 256
    quaternion = quat.quaternion(1, 0, 0, 0)
    beta = 1
    zeta = 0

    def __init__(self, sampleperiod=None, quaternion=None, beta=None, zeta=None):
        """
        Initialize the class with the given parameters.
        :param sampleperiod: The sample period
        :param quaternion: Initial quaternion
        :param beta: Algorithm gain beta
        :param beta: Algorithm gain zeta
        :return:
        """
        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta
        if zeta is not None:
            self.zeta = zeta

    def update_imu(self, gyroscope, accelerometer):
        """
        Perform one update step with data from a IMU sensor array
        :param gyroscope: A three-element array containing the gyroscope data in radians per second.
        :param accelerometer: A three-element array containing the accelerometer data. Can be any unit since a normalized value is used.
        """
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()

        # Normalise accelerometer measurement
        if norm(accelerometer) == 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Gradient descent algorithm corrective step
        f = np.array([
            2 * (q.x * q.z - q.w * q.y) - accelerometer[0],
            2 * (q.w * q.x + q.y * q.z) - accelerometer[1],
            2 * (0.5 - q.x ** 2 - q.y ** 2) - accelerometer[2]
        ])
        j = np.array([
            [-2 * q.y, 2 * q.z, -2 * q.w, 2 * q.x],
            [2 * q.x, 2 * q.w, 2 * q.z, 2 * q.y],
            [0, -4 * q.x, -4 * q.y, 0]
        ])
        step = j.T.dot(f)
        step /= norm(step)  # normalise step magnitude
        q_step = quat.quaternion(step[0], step[1], step[2], step[3])

        # Compute rate of change of quaternion
        qdot = (q * quat.quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - self.beta * q_step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = q / norm(quat.as_float_array(q))  # normalise quaternion
