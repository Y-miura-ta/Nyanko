import numpy as np
import threading
import time
import quaternion
import signal
import sys
import csv

import LegTrajectory as lt

class quadrupedGait():
    def __init__(self, _leg_center, _leg_start_pos, _body_h, _leg_up_h, _h_offset, _T, _dt, _classIMU):
        self.leg_center = _leg_center
        self.leg_now_pos = _leg_start_pos
        self.body_h = _body_h
        self.leg_up_h = _leg_up_h
        self.h_offset = _h_offset
        self.T = _T
        self.dt = _dt
        self.classIMU = _classIMU
        self.target_vel = [0.0, 0.0, 0.0]
        self.t_start = [0.0, self.T/2.0, self.T/2.0, 0.0]
        self.real_dt = self.dt
        self.controll_time = time.time()
        self.event = threading.Event()
        signal.signal(signal.SIGALRM, self.setEvent)
        # IMUの値が安定するまで3秒待つ
        signal.setitimer(signal.ITIMER_REAL, 3.0, self.dt)
        thread = threading.Thread(target=self.quadrupedTrajectory)
        thread.start()
    
    def setEvent(self, arg1, arg2):
        self.event.set()

    def calcBodyVelHorizontal(self):
        # ボディ座標系での水平方向速度を計算
        rot_inv = self.classIMU.rot.T
        vel_world_horizontal = np.array([self.classIMU.v[0], self.classIMU.v[1], 0.0])
        vel_body_horizontal = rot_inv@vel_world_horizontal

        return vel_body_horizontal

    def hoge():

    def trajectoryLoop(self):
        while(True):
            time_now = time.time()
            self.real_dt = time_now - self.controll_time
            
            vel_b_h = self.calcBodyVelHorizontal()

            self.controll_time = time_now
            print()