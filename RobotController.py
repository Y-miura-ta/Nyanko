# -*- coding: utf-8 -*-

import numpy as np
import time

import IK
import DynamixelController as dc
import TimerIMU as ti
import GaitGenerator as gg

JOINT_DIREC = np.array([
    [-1.0, -1.0, 1.0],
    [-1.0, -1.0, 1.0],
    [1.0, 1.0, -1.0],
    [1.0, 1.0, -1.0]
])

# ロボットの関節を直接動かして座標を確認
def posPrint():
    pos_list = dc.syncreadPos(dc.DXL_ID_LIST)
    pos_leg = np.array([
        [pos_list[0], pos_list[1], pos_list[2]],
        [pos_list[3], pos_list[4], pos_list[5]],
        [pos_list[6], pos_list[7], pos_list[8]],
        [pos_list[9], pos_list[10], pos_list[11]]
    ])
    xyz_pos = []
    rad_list = (pos_leg-np.ones((4, 3))*dc.DXL_MEDIUM_POSITION_VALUE)*JOINT_DIREC*dc.DXLPOS_2_RAD
    for rad in rad_list:
        xyz_pos.append(IK.legFK(rad, [0, 0, 0])[2])

    print("DXLpos : {}".format((pos_leg-np.ones((4, 3))*dc.DXL_MEDIUM_POSITION_VALUE)*JOINT_DIREC))
    print("XYZpos list : {}".format(xyz_pos))

def main():
    dc.setDXL()
    dc.torqueOFF(dc.DXL_ID_LIST)
    while(True):
        posPrint()
        time.sleep(0.1)

if __name__ == '__main__':
    main()