# -*- coding: utf-8 -*-

from this import d
from turtle import pos
from dynamixel_sdk import *
import time
import math
import numpy as np

ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4
DXL_MINIMUM_POSITION_VALUE  = 0
DXL_MEDIUM_POSITION_VALUE = 2048
DXL_MAXIMUM_POSITION_VALUE  = 4095
BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0
DXL_ID_LIST = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
DEVICENAME                  = '/dev/ttyUSB0'
TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 20

DXLPOS_2_RAD = 2*math.pi/(DXL_MAXIMUM_POSITION_VALUE-DXL_MINIMUM_POSITION_VALUE)
RAD_2_DXLPOS = (DXL_MAXIMUM_POSITION_VALUE-DXL_MINIMUM_POSITION_VALUE)/(2*math.pi)

# 回転方向の符号
JOINT_DIREC = np.array([
    [-1.0, -1.0, 1.0],
    [-1.0, -1.0, 1.0],
    [1.0, 1.0, -1.0],
    [1.0, 1.0, -1.0]
])

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

def setDXL():
    # ポートを開く
    if(portHandler.openPort()):
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        quit()

    # ボーレート設定
    if(portHandler.setBaudRate(BAUDRATE)):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        quit()

def closePort():
    portHandler.closePort()

def torqueON(dxl_id_list):
    # トルクをオン
    for id in dxl_id_list:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if(dxl_comm_result != COMM_SUCCESS):
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif(dxl_error != 0):
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % id)

def torqueOFF(dxl_id_list):
    # トルクをオフ
    for id in dxl_id_list:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

def setID(dxl_id_list):
    # Syncwriteで書き込むIDをセット
    for id in dxl_id_list:
        dxl_addparam_result = groupSyncRead.addParam(id)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % id)
            quit()

def setGoalPos(dxl_id_list, goal_pos_list):
    # Syncwriteで書き込むポジションをセット
    for id, pos in zip(dxl_id_list, goal_pos_list):
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(pos)), DXL_HIBYTE(DXL_LOWORD(pos)), DXL_LOBYTE(DXL_HIWORD(pos)), DXL_HIBYTE(DXL_HIWORD(pos))]
        dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % id)
            quit()

def syncwritePos(dxl_id_list, goal_pos_list):
    #setID(dxl_id_list)
    setGoalPos(dxl_id_list, goal_pos_list)
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Syncwriteの設定をクリア
    groupSyncWrite.clearParam()

def checkSyncreadData(dxl_id_list):
    for id in dxl_id_list:
        dxl_getdata_result = groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % id)
            quit()

def getPosData(dxl_id_list):
    pos_list = []
    for id in dxl_id_list:
        pos = groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        pos_list.append(pos)
    
    return pos_list

def syncreadPos(dxl_id_list):
    setID(dxl_id_list)
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    checkSyncreadData(dxl_id_list)
    pos_list = getPosData(dxl_id_list)
    # Syncreadの設定をクリア
    groupSyncRead.clearParam()
    
    return pos_list

def main():
    dxl_id_list = [1, 7]
    pos_list1 = [DXL_MINIMUM_POSITION_VALUE, DXL_MINIMUM_POSITION_VALUE]
    pos_list2 = [DXL_MAXIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]
    setDXL()
    torqueON(dxl_id_list)
    syncwritePos(dxl_id_list,pos_list1)
    time.sleep(3)
    pos_list_now = syncreadPos(dxl_id_list)
    print(pos_list_now)
    time.sleep(1)
    syncwritePos(dxl_id_list,pos_list2)
    time.sleep(3)
    pos_list_now = syncreadPos(dxl_id_list)
    print(pos_list_now)
    time.sleep(1)
    torqueOFF(dxl_id_list)
    closePort()

if __name__ == '__main__':
    main()
