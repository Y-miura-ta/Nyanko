# -*- coding: utf-8 -*-

import threading
import time
import board
import adafruit_bno055
import numpy as np
import quaternion
import signal
import sys
import csv

class timerIMU():
    def __init__(self, _dt):
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        print("IMU calibration is done.")
        self.q = np.quaternion(0.0, 0.0, 0.0, 1.0)
        self.rot = quaternion.as_rotation_matrix(self.q)
        # 以下のp, v, aはワールド座標系
        self.p = np.array([0.0, 0.0, 0.0])
        self.v = np.array([0.0, 0.0, 0.0])
        self.a = np.array([0.0, 0.0, 0.0])
        self.alpha = 0.8 # 過去の値に係数をかけて積分値を安定させる
        self.dt = _dt
        self.real_dt = self.dt
        self.controll_time = time.time()
        self.event = threading.Event()
        signal.signal(signal.SIGALRM, self.setEvent)
        # IMUの値が安定するまで3秒待つ
        signal.setitimer(signal.ITIMER_REAL, 3.0, self.dt)
        self.thread = threading.Thread(target=self.velIntegral)
        self.thread.start()
    
    def setEvent(self, arg1, arg2):
        self.event.set()

    def velIntegral(self):
        while(True):
            # 外部から呼ばれる変数にアクセスするので他の処理を停止
            self.event.wait()
            time_now = time.time()
            quat = self.sensor.quaternion
            acc = self.sensor.linear_acceleration
            self.q = np.quaternion(quat[0], quat[1], quat[2], quat[3])
            Acc = np.array([acc[0], acc[1], acc[2]])*1000 # 単位をmmに変換
            self.rot = quaternion.as_rotation_matrix(self.q)
            self.real_dt = time_now - self.controll_time
            self.a = self.rot@Acc
            # 積分
            self.v = self.v*self.alpha + self.a*self.real_dt
            self.p = self.p + self.v*self.real_dt
            self.controll_time = time_now
            self.event.clear()

def main():

    N = 100
    x = np.zeros(N)
    y1 = np.zeros(N)
    y2 = np.zeros(N)
    y3 = np.zeros(N)
    t = 0
    dt = 0.01

    log_file = './IMU_log.csv'
    file = open(log_file, 'w')
    writer = csv.writer(file)
    writer.writerow(['TimeStamp', 'dt', 'ax', 'ay', 'az', 'vx', 'vy', 'vz', 'ax', 'ay', 'az'])
    
    tIMU = timerIMU(0.015)

    print("IMU loop start")

    while(True):
        a_IMU = [tIMU.a[0], tIMU.a[1], tIMU.a[2]]
        v_IMU = [tIMU.v[0], tIMU.v[1], tIMU.v[2]]
        p_IMU = [tIMU.p[0], tIMU.p[1], tIMU.p[2]]
        time_now = time.time()
        real_dt = tIMU.real_dt
        data_IMU = []
        data_IMU.extend([time_now, real_dt])
        data_IMU.extend(a_IMU)
        data_IMU.extend(v_IMU)
        data_IMU.extend(p_IMU)
        writer.writerow(data_IMU)
        print("time:{}".format(time_now))
        #print("dt:{}, world_vel_x:{}, world_vel_y:{}, world_vel_z:{}".format(real_dt, v_IMU[0], v_IMU[1], v_IMU[2]))
        #print("------------------------------------------------------------------------------------------")
        time.sleep(0.005)
    file.close()

if __name__ == '__main__':
    sys.exit(main())

    