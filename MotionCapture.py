import numpy as np
import threading
import time

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

        dc.setDXL()
        dc.torqueOFF(dc.DXL_ID_LIST)

        self.thread = threading.Thread(target=self.captureLoop)
        self.capture_flag = True
        self.thread.start()

    def captureLoop(self):
        if(self.capture_flag):
            t = threading.Timer(self.dt, self.captureLoop)
            t.start()
        
        time_now = time.time()
        self.real_dt = time_now - self.controll_time
        print(self.real_dt)
        self.pos_list = dc.syncreadPos(dc.DXL_ID_LIST)
        self.xyz_list = dxlPos2XYZ(self.pos_list)
        # print(self.real_dt, self.xyz_list)
        print(self.pos_list)
        
        self.controll_time = time_now

    def startCapture(self):
        self.capture_flag = True
        t = threading.Timer(self.dt, self.captureLoop)
        t.start()

    def stopCapture(self):
        self.capture_flag = False

def main():
    tCap = timerCapture(0.01)
    tCap.startCapture()
    # while(True):
    #     print("-------------------------------")
    time.sleep(5)
    tCap.stopCapture()
    print("stop 1sec")
    time.sleep(1)
    tCap.startCapture()
    time.sleep(5)
    tCap.stopCapture()

if __name__ == '__main__':
    main()
