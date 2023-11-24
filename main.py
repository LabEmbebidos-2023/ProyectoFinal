
import math
import threading
import time

from flask import Flask
from flask_sock import Sock

import signal
import sys
from ischedule import schedule, run_loop
from control.chassis import Chassis

chs = Chassis(imu_sample_period=0.01, odometry_update_period=0.01)

chs.set_speeds(0.0, 0.0)


def imu_update_task():
    chs.update_imu()


def odometry_update_task():
    chs.update_odometry()

schedule(imu_update_task, interval=chs.imu_sample_period)
schedule(odometry_update_task, interval=chs.odometry_update_period)

running = True
event = threading.Event()


def run_scheduler():
    global event
    run_loop(stop_event=event)


thread = threading.Thread(target=run_scheduler)
thread.start()


def signal_handler(sig, frame):
    chs.set_speeds(0, 0)

    global event
    event.set()

    while thread.is_alive():
        print("Waiting for thread to end...")
        time.sleep(0.5)

    thread.join()

    sys.exit(0)


app = Flask(__name__)
sock = Sock(app)

signal.signal(signal.SIGINT, signal_handler)


@sock.route('/')
def robot_socket(ws):
    while True:
        data = ws.receive()

        if data.startswith("enabled:"):
            print("enabled")
        elif data.startswith("opmode:"):
            print("changing op mode")
        elif data.startswith("control:"):
            x = data.split(":")
            y = x[1]
            z = y.split(",")
            chs.set_speeds(float(z[0]),float(z[1]))
        elif data.startswith("pose"):
            ws.send("%lf, %lf, %lf" % (chs.pose[0], chs.pose[1], chs.pose[2]))
