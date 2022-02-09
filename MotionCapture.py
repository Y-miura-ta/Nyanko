import numpy as np
import threading
import signal
import time
import csv

import IK
import DynamixelController as dc

def dxlPos2Rad(dxl_pos_list):
    list_len = len(dxl_pos_list)
    dxl_pos_array = np.array(dxl_pos_list)
    rad_array = (dxl_pos_array - dc.DXL_MEDIUM_POSITION_VALUE*np.ones(list_len))*dc.DXLPOS_2_RAD*dc.JOINT_DIREC_FLAT

    return rad_array.reshape(4, 3)

def dxlPos2XYZ(dxl_pos_list):
    rad_array = dxlPos2Rad(dxl_pos_list)
    leg_pos = []
    for leg_num in range(4):
        (J12, J3, pos_xyz)  = IK.legFK(rad_array[leg_num], [0, 0, 0])
        leg_pos.append(pos_xyz)

    return leg_pos

class timerCapture():
    def __init__(self, _dt):
        self.dt = _dt
        self.real_dt = self.dt
        self.controll_time = time.time()

        self.pos_list = []
        self.xyz_list = []

        self.capture_flag = True

        self.log_file = './capture_log.csv'
        self.file = open(self.log_file, 'w')
        print("file open")
        self.writer = csv.writer(self.file)
        data = ['TimeStamp']
        self.writer.writerow(data)
        #self.file.close()
        #print("file close")

        dc.setDXL()
        dc.torqueOFF(dc.DXL_ID_LIST)

        self.event = threading.Event()
        signal.signal(signal.SIGALRM, self.setEvent)
        # 念のため1秒待つ
        signal.setitimer(signal.ITIMER_REAL, 1.0, self.dt)

    def startCapture(self):
        self.capture_flag = True
        dc.setID(dc.DXL_ID_LIST)
        self.thread = threading.Thread(target=self.captureLoop)
        self.thread.start()

    def stopCapture(self):
        self.capture_flag = False
        self.thread.join()
        dc.clearID()
    
    def setEvent(self, arg1, arg2):
        self.event.set()

    def captureLoop(self):
        while(self.capture_flag):
            self.event.wait()
            #dc.setID(dc.DXL_ID_LIST)
            self.pos_list = dc.syncreadPos(dc.DXL_ID_LIST)
            #self.xyz_list = dxlPos2XYZ(self.pos_list)
            #dc.clearID()
            
            time_now = time.time()
            self.real_dt = time_now - self.controll_time
            data = [self.real_dt]
            data.append(self.pos_list)
            self.writer.writerow(data)
            #print(self.real_dt)
            # print(self.real_dt, self.xyz_list)
            #print(self.pos_list)

            self.controll_time = time_now
            self.event.clear()

def main():
    tCap = timerCapture(0.07)
    
    # self.writer = csv.writer(file)
    # self.writer.writerow(['TimeStamp'])
    tCap.startCapture()
    # while(True):
    #     print("-------------------------------")
    
    time.sleep(3)
    tCap.stopCapture()
    time.sleep(1)
    #tCap.startCapture()
    tCap.file.close()
    
if __name__ == '__main__':
    main()
