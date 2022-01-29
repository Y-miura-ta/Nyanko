import numpy as np
import threading
import time
import quaternion
import signal
import sys
import csv

import LegTrajectory as lt
import IK
import TimerIMU as ti

class quadrupedGait():
    def __init__(self, _leg_centers, _leg_start_states, _body_h, _leg_up_h, _h_offset, _T, _dt, _classIMU):
        self.leg_center = _leg_centers
        self.leg_cur_states = _leg_start_states
        self.leg_pre_states = _leg_start_states
        self.theta_list = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]
        self.theta_pre_list = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0]
        ]
        self.pre_theta_flag = [False, False, False, False]
        self.body_h = _body_h
        self.leg_up_h = _leg_up_h
        self.h_offset = _h_offset
        self.T = _T
        self.dt = _dt
        self.tIMU = _classIMU # ti.timerIMU(0.015)
        self.target_vel = [0.0, 0.0, 0.0]
        self.t_start = [0.0, self.T/2.0, self.T/2.0, 0.0]
        self.t_cur_list = [0.0, self.T/2.0, self.T/2.0, 0.0]
        self.real_dt = self.dt
        self.controll_time = time.time()

        # self.event = threading.Event()
        # signal.signal(signal.SIGALRM, self.setEvent)
        # # IMUの待ち時間が終わるまで待つ
        # signal.setitimer(signal.ITIMER_REAL, 3.5, self.dt)

        self.thread = threading.Thread(target=self.trajectoryLoop)
        self.thread.start()
    
    def setEvent(self, arg1, arg2):
        self.event.set()

    def calcBodyVelHorizontal(self):
        # ボディ座標系での水平方向速度を計算
        rot_inv = self.tIMU.rot.T
        vel_world_horizontal = np.array([self.tIMU.v[0], self.tIMU.v[1], 0.0])
        vel_body_horizontal = rot_inv@vel_world_horizontal

        return vel_body_horizontal

    def calcNextLegState(self, vel_body_horizontal, leg_num, t_cur):
        p, v, a = lt.calcBalancePVA(
            self.leg_cur_states[leg_num],
            self.target_vel,
            vel_body_horizontal,
            self.leg_center[leg_num],
            self.body_h,
            self.leg_up_h,
            self.h_offset,
            self.T,
            self.dt
        )

        if(t_cur+self.dt > self.T):
            state_next = [p, v, a, 0]
        else:
            state_next = [p, v, a, t_cur+self.dt]

        return state_next

    def trajectoryLoop(self):
        t = threading.Timer(self.dt, self.trajectoryLoop)
        t.start()
        time_now = time.time()
        self.real_dt = time_now - self.controll_time
        vel_b_h = self.calcBodyVelHorizontal()
        for leg_num in range(4):
            self.leg_cur_states[leg_num][3] = self.t_cur_list[leg_num]
            state_next = self.calcNextLegState(vel_b_h, leg_num, self.t_cur_list[leg_num])
            # ここで関節可動域内か判断する
            joint_range_out = False
            self.theta_list[leg_num], self.pre_theta_flag[leg_num] = IK.legSmartIK(state_next[0], self.theta_pre_list[leg_num], joint_range_out)
            self.theta_pre_list[leg_num] = self.theta_list[leg_num]
            self.t_cur_list[leg_num] = state_next[3]
        # print(self.theta_list[0], self.pre_theta_flag)
        print("aaa:{}".format(self.tIMU.v))
        # print(self.t_cur_list)
        self.controll_time = time_now
        
    # def trajectoryLoop(self):
    #     while(True):
    #         self.event.wait()
    #         time_now = time.time()
    #         self.real_dt = time_now - self.controll_time
    #         vel_b_h = self.calcBodyVelHorizontal()
    #         for leg_num in range(4):
    #             self.leg_cur_states[leg_num][3] = self.t_cur_list[leg_num]
    #             state_next = self.calcNextLegState(vel_b_h, leg_num, self.t_cur_list[leg_num])
    #             # ここで関節可動域内か判断する
    #             joint_range_out = False
    #             self.theta_list[leg_num], self.pre_theta_flag[leg_num] = IK.legSmartIK(state_next[0], self.theta_pre_list[leg_num], joint_range_out)
    #             self.theta_pre_list[leg_num] = self.theta_list[leg_num]
    #             self.t_cur_list[leg_num] = state_next[3]
    #         # print(self.theta_list[0], self.pre_theta_flag)
    #         print("aaa:{}".format(self.real_dt))#self.tIMU.v)
    #         # print(self.t_cur_list)
    #         self.controll_time = time_now
    #         self.event.clear()

def main():
    leg_centers = [
        [0.0, 0.0, -170.0],
        [0.0, 0.0, -170.0],
        [0.0, 0.0, -170.0],
        [0.0, 0.0, -170.0]
    ]
    leg_start_states = [
        [[0.0, 0.0, -170.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0],
        [[0.0, 0.0, -170.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0],
        [[0.0, 0.0, -170.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0],
        [[0.0, 0.0, -170.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0]
    ]
    body_h = 170.0 + IK.sole_r
    leg_up_h = 30.0
    h_offset = 3.0
    T = 0.5
    dt = 0.005

    tIMU = ti.timerIMU(0.015)
    qGait = quadrupedGait(leg_centers, leg_start_states, body_h, leg_up_h, h_offset, T, dt, tIMU)

    while(True):
        print("-------------------------------")
        time.sleep(1)

if __name__ == '__main__':
    main()