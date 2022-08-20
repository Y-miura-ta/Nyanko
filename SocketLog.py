import sys
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

class timerPlayback():
    def __init__(self, _dt):
        self.dt = _dt
        self.real_dt = self.dt
        self.controll_time = time.time()

        self.pos_list = []
        self.xyz_list = []

        self.capture_flag = True

        self.log_file = './test_walk.csv'
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
            pos_array = rad2dxlPos(rad_array[1:13])
            print(rad_array[1:13])
            dc.syncwritePos(dc.DXL_ID_LIST, pos_array)
            self.playback_index += 1

            time_now = time.time()
            self.real_dt = time_now - self.controll_time
            
            self.controll_time = time_now
            self.event.clear()

class socketCapture():
    def __init__(self):
        self.host = "192.168.0.232"
        self.port = 50000
        
        self.serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.serversock.bind((self.host, self.port))
        self.serversock.listen(10)

        print('Waiting for connections...')
        self.clientsock, self.client_address = self.serversock.accept()

        self.log_file = './test_walk.csv'
        self.file = open(self.log_file, 'w')
        print("Logfile open")
        self.writer = csv.writer(self.file)
        data = ['TimeStamp']
        for i in range(12):
            data.append("joint_{}_rad".format(i+1))
        data.append("EndSignal")
        self.writer.writerow(data)
        
        dc.setDXL()
        dc.torqueON(dc.DXL_ID_LIST)

    def captureLoop(self):
        try:
            while(True):
                rcvmsg = self.clientsock.recv(1024)
                data = pickle.loads(rcvmsg)
                pos_array = rad2dxlPos(data[0:12])
                end_signal = data[12]
                dc.syncwritePos(dc.DXL_ID_LIST, pos_array)
                print('Received -> %s, %s' % (pos_array, end_signal))
                
                time_now = time.time()
                data = np.insert(data, 0, time_now)
                self.writer.writerow(data)

        finally:
            self.clientsock.close()
            
def main():
    args = sys.argv
    dt = 60.0/(115.0*10*2)
    if(len(args)>=2):
        if(args[1] == "c"):
            sCap = socketCapture()
            sCap.captureLoop()
        elif(args[1] == "p"):
            tPlay = timerPlayback(dt)
            tPlay.startPlayback()
        elif(args[1] == "cp"):
            sCap = socketCapture()
            sCap.captureLoop()
            tPlay = timerPlayback(dt)
            tPlay.startPlayback()
        else:
            print("Type args c p cp")
    else:
        print("Type args")
        
if __name__ == '__main__':
    main()
