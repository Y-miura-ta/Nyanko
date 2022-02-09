# -*- coding: utf-8 -*-

import numpy as np
import time

import IK
import DynamixelController as dc
import TimerIMU as ti
import GaitGenerator as gg

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
    rad_list2 = []
    rad_list = (pos_leg-np.ones((4, 3))*dc.DXL_MEDIUM_POSITION_VALUE)*dc.JOINT_DIREC*dc.DXLPOS_2_RAD
    for rad in rad_list:
        xyz = IK.legFK(rad, [0, 0, 0])[2]
        xyz_pos.append(xyz)
        rad_list2.append(IK.legSmartIK(xyz, rad, False))

    print("DXLpos : {}".format((pos_leg-np.ones((4, 3))*dc.DXL_MEDIUM_POSITION_VALUE)*dc.JOINT_DIREC))
    print("XYZpos list : {}".format(xyz_pos))
    print("rad2 : {}".format(rad_list2))

def trotWalk():
    leg_centers = [
        [0.0, 0.0, -170.0],
        [-40.0, 0.0, -170.0],
        [0.0, 0.0, -170.0],
        [-40.0, 0.0, -170.0]
    ]
    leg_start_states = [
        [[0.0, 0.0, -170.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0],
        [[-40.0, 0.0, -170.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0],
        [[0.0, 0.0, -170.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0],
        [[-40.0, 0.0, -170.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0]
    ]
    body_h = 170.0 + IK.sole_r
    leg_up_h = 45.0
    h_offset = 3.0
    T = 0.5
    dt = 0.05

    tIMU = ti.timerIMU(0.015)
    qGait = gg.quadrupedGait(leg_centers, leg_start_states, body_h, leg_up_h, h_offset, T, dt, tIMU)

def main():
    # dc.setDXL()
    # dc.torqueOFF(dc.DXL_ID_LIST)
    trotWalk()
    while(True):
        # posPrint()
        time.sleep(0.1)

if __name__ == '__main__':
    main()
