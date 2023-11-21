import math
import signal
import sys
from ischedule import schedule, run_loop
from control.chassis import Chassis

chs = Chassis(imu_sample_period=0.01, odometry_update_period=0.01)

chs.set_speeds(0.0, 0.0)


def signal_handler(sig, frame):
    chs.set_speeds(0,0)
    sys.exit(0)


def imu_update_task():
    chs.update_imu()


def odometry_update_task():
    chs.update_odometry()


def telemetry_task():
    heading = chs.pose[2] * 180.0 / math.pi


signal.signal(signal.SIGINT, signal_handler)
schedule(imu_update_task, interval=chs.imu_sample_period)
schedule(odometry_update_task, interval=chs.odometry_update_period)
schedule(telemetry_task, interval=0.01)

run_loop()
