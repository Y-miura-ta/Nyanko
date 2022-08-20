# -*- coding: utf-8 -*-

#前がX軸、左がy軸、上がZ軸
from mpl_toolkits import mplot3d
import numpy as np
from math import *
import matplotlib.pyplot as plt

import IK

CP=np.array([
        [IK.body_l/2, IK.body_w/2, 0],
        [-IK.body_l/2, IK.body_w/2, 0],
        [IK.body_l/2, -IK.body_w/2, 0],
        [-IK.body_l/2, -IK.body_w/2, 0]
    ])

# 描画の設定
def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    return ax

def drawLegPoints(p):
    plt.plot([x[0] for x in p],[x[1] for x in p],[x[2] for x in p], 'k-', lw=3)
    plt.plot([p[0][0]],[p[0][1]],[p[0][2]],'bo',lw=2)
    plt.plot([p[2][0]],[p[2][1]],[p[2][2]],'ro',lw=2)   

def drawRobotFromTheta(legThetas, p_legThetas): # 関節角度を直接指定(仮想ロボット)
    CPs=[CP[x] for x in [0,1,3,2,0]]
    plt.plot([x[0] for x in CPs],[x[1] for x in CPs],[x[2] for x in CPs], 'bo-', lw=2)
    
def drawRobotFromXYZ(lp0, lp1, lp2, lp3):
    CPs=[CP[x] for x in [0,1,3,2,0]]
    plt.plot([x[0] for x in CPs],[x[1] for x in CPs],[x[2] for x in CPs], 'bo-', lw=2)

    theta, rangeOut = IK.calcIK(lp0)
    drawLegPoints(IK.legFK(theta)+CP[0])
    theta, rangeOut = IK.calcIK(lp1)
    drawLegPoints(IK.legFK(theta)+CP[1])
    theta, rangeOut = IK.calcIK(lp2)
    drawLegPoints(IK.legFK(theta)+CP[2])
    theta, rangeOut = IK.calcIK(lp3)
    drawLegPoints(IK.legFK(theta)+CP[3])

def main():
    # グラフで確認
    setupView(200).view_init(elev=12., azim=28)
    drawRobotFromXYZ([80, 30, -150], [80, 30, -150], [80, 30, -150], [80, 30, -150])
    plt.show()
    
if __name__ == '__main__':
    main()

