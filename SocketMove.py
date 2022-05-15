import numpy as np
import socket
import pickle

import threading
import signal
import time
import csv
import pandas as pd

import IK
import DynamixelController as dc

host = "192.168.0.232"
port = 50000

serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
serversock.bind((host,port))
serversock.listen(10)

def dxlPos2Rad(dxl_pos_list):
    list_len = len(dxl_pos_list)
    dxl_pos_array = np.array(dxl_pos_list)
    rad_array = (dxl_pos_array - dc.DXL_MEDIUM_POSITION_VALUE*np.ones(list_len))*dc.DXLPOS_2_RAD*dc.JOINT_DIREC_FLAT

    return rad_array

def rad2dxlPos(rad_array):
    pos_f = dc.DXL_MEDIUM_POSITION_VALUE*np.ones(len(rad_array)) - rad_array*dc.JOINT_DIREC_FLAT2*dc.RAD_2_DXLPOS
    pos_i = pos_f.astype(int)

    return pos_i

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
        print("Logfile open")
        self.writer = csv.writer(self.file)
        data = ['TimeStamp']
        for i in range(12):
            data.append("joint_{}_rad".format(i+1))
        self.writer.writerow(data)
        
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
            self.pos_list = dc.syncreadPos(dc.DXL_ID_LIST)
            rad_list = dxlPos2Rad(self.pos_list)
            #self.xyz_list = dxlPos2XYZ(self.pos_list)
            
            time_now = time.time()
            self.real_dt = time_now - self.controll_time
            data = np.insert(rad_list, 0, self.real_dt)
            self.writer.writerow(data)
            #print(self.real_dt)
            # print(self.real_dt, self.xyz_list)
            #print(self.pos_list)

            self.controll_time = time_now
            self.event.clear()

class timerPlayback():
    def __init__(self, _dt):
        self.dt = _dt
        self.real_dt = self.dt
        self.controll_time = time.time()

        self.pos_list = []
        self.xyz_list = []

        self.capture_flag = True

        self.log_file = './capture_log.csv'
        log_data_df = pd.read_csv(self.log_file)
        self.log_data = log_data_df.values
        print("Log file loaded")
        self.playback_index = 0

        dc.setDXL()
        dc.torqueON(dc.DXL_ID_LIST)

        self.event = threading.Event()
        signal.signal(signal.SIGALRM, self.setEvent)
        # 念のため1秒待つ
        signal.setitimer(signal.ITIMER_REAL, 1.0, self.dt)

    def startPlayback(self):
        self.capture_flag = True
        #dc.setID(dc.DXL_ID_LIST)
        self.thread = threading.Thread(target=self.playbackLoop)
        self.thread.start()

    def stopPlayback(self):
        self.capture_flag = False
        self.thread.join()
        dc.torqueOFF(dc.DXL_ID_LIST)
    
    def setEvent(self, arg1, arg2):
        self.event.set()

    def playbackLoop(self):
        while(self.capture_flag):
            self.event.wait()
            rad_array = self.log_data[self.playback_index]
            # 先頭には実行時間が入っているので削除
            pos_array = rad2dxlPos(np.delete(rad_array, 0))
            dc.syncwritePos(dc.DXL_ID_LIST, pos_array)
            self.playback_index += 5

            time_now = time.time()
            self.real_dt = time_now - self.controll_time
            
            self.controll_time = time_now
            self.event.clear()
 
def main():
    print('Waiting for connections...')
    clientsock, client_address = serversock.accept()
    dc.setDXL()
    dc.torqueON(dc.DXL_ID_LIST)
    while True:
        rcvmsg = clientsock.recv(1024)
        data = pickle.loads(rcvmsg)
        pos_array = rad2dxlPos(data[0:12])
        end_signal = data[12]
        dc.syncwritePos(dc.DXL_ID_LIST, pos_array)
        print('Received -> %s, %s' % (pos_array, end_signal))
    clientsock.close()
    
if __name__ == '__main__':
    main()
