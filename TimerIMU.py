from asyncore import write
import threading
import time
import board
import adafruit_bno055
import numpy as np
from numpy import real
import quaternion
import matplotlib.pyplot as plt
import signal
import sys
import csv

class timerIMU():
    def __init__(self, _dt):
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        print("IMU calibration is done.")
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
        thread = threading.Thread(target=self.velIntegral)
        thread.start()
    
    def setEvent(self, arg1, arg2):
        self.event.set()

    def velIntegral(self):
        while(True):
            # 外部から呼ばれる変数にアクセスするので他の処理を停止
            self.event.wait()
            time_now = time.time()
            quat = self.sensor.quaternion
            acc = self.sensor.linear_acceleration
            Quat = np.quaternion(quat[0], quat[1], quat[2], quat[3])
            Acc = np.array([acc[0], acc[1], acc[2]])
            Rot = quaternion.as_rotation_matrix(Quat)
            self.real_dt = time_now - self.controll_time
            self.a = Rot@Acc
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
    # plt.ion()
    # plt.figure()
    # line, = plt.plot(x, y1, color='blue')
    # line, = plt.plot(x, y2, color='red')
    # line, = plt.plot(x, y3, color='green')
    # # ax.set_ylim((-10, 10))
    # plt.ylim(-10, 10)
    # plt.xlabel("time [s]")
    # plt.ylabel("value [-]")

    # log_file = './IMU_log.csv'
    # file = open(log_file, 'w')
    # writer = csv.writer(file)
    # writer.writerow(['TimeStamp', 'dt', 'ax', 'ay', 'az', 'vx', 'vy', 'vz', 'ax', 'ay', 'az'])
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
        # writer.writerow(data_IMU)
        print("time:{}".format(time_now))
        #print("dt:{}, world_vel_x:{}, world_vel_y:{}, world_vel_z:{}".format(real_dt, v_IMU[0], v_IMU[1], v_IMU[2]))
        #print("------------------------------------------------------------------------------------------")
        time.sleep(0.005)
        # x = np.append(x, t)
        # x = np.delete(x, 0)
        # t = t + dt
        # y1 = np.append(y1, v_IMU[0]*1000)
        # y1 = np.delete(y1, 0)
        # y2 = np.append(y2, v_IMU[1]*1000)
        # y2 = np.delete(y2, 0)
        # y3 = np.append(y3, v_IMU[2]*1000)
        # y3 = np.delete(y3, 0)
        # line, = plt.plot(x, y1, color='blue')
        # line, = plt.plot(x, y2, color='red')
        # line, = plt.plot(x, y3, color='green')
        # plt.xlim(min(x), max(x))
        # plt.draw()
        # plt.pause(dt)
    # file.close()

if __name__ == '__main__':
    sys.exit(main())

    