# -*- coding: utf-8 -*-

from math import *
#import time

l1 = 90
l2 = 90
MIN_RXZ = 10
L = 100
W = 50

def legIK(p):
    x = p[0]
    y = p[1]
    z = p[2]
    
    r2 = x**2 + y**2 + z**2
    if(r2 > (l1+l2)**2):
        theta1 = 0.0
        theta2 = pi/2
        theta3 = 0.0
        rangeOut = True
        
        return [theta1, theta2, theta3], rangeOut

    rxz2 = x**2 + z**2
    rxz = rxz2**0.5
    if(rxz < MIN_RXZ):
        theta1 = 0.0
        theta2 = 0.0
        theta3 = 0.0
        rangeOut = True

        return [theta1, theta2, theta3], rangeOut
        
    theta3 = acos(-(l1**2+l2**2-r2)/(2*l1*l2))
    lx = l2*sin(theta3)
    al = atan2(-z, x)
    lxrxz = lx/rxz
    if(lx > rxz): # 丸め誤差などで起こりうる
        lxrxz = 1.0
    be = acos(lxrxz)
    theta1 = be - al
    zd = -x*sin(theta1) + z*cos(theta1)
    theta2 = atan2(y, -zd)
    rangeOut = False

    return [theta1, theta2, theta3], rangeOut

def legSmartIK(p, p_theta, jointRangeOut): # レンジ外が入力された場合、前回位置に向かう
    if(jointRangeOut):
        theta[0] = p_theta[0]
        theta[1] = p_theta[1]
        theta[1] = p_theta[1]
        preTheta = True

        return theta, preTheta
    else:
        theta, rangeOut = legIK(p)
        if(rangeOut):
            theta[0] = p_theta[0]
            theta[1] = p_theta[1]
            theta[1] = p_theta[1]
            preTheta = True
            
            return theta, preTheta
        
        else:
            preTheta = False

            return theta, preTheta

def legFK(theta, offset): # 関節の正回転方向と座標変換の正回転方向が逆の場合があるので注意
    J12 = [0 + offset[0],
           0 + offset[1],
           0 + offset[2]]
    J3 = [-l1*sin(-theta[0])*cos(theta[1]) + offset[0],
          l1*sin(theta[1]) + offset[1],
          -l1*cos(-theta[0])*cos(theta[1]) + offset[2]]
    J4 = [-l2*sin(-theta[2])*cos(-theta[0]) + (-l2*cos(-theta[2]) - l2)*sin(-theta[0])*cos(theta[1]) + offset[0],
           -(-l2*cos(-theta[2]) - l2)*sin(theta[1]) + offset[1],
           l2*sin(-theta[0])*sin(-theta[2]) + (-l2*cos(-theta[2]) - l2)*cos(-theta[0])*cos(theta[1]) + offset[2]]
    
    return (J12, J3, J4)

def calcErrRev(theta): # 入力の単位はdeg
    J12, J3, J4 = legFK([theta[0]/180*pi, theta[1]/180*pi, theta[2]/180*pi], [0, 0, 0])
    thetaIK, rangeOut = legIK(J4)
    # エラーの単位もdeg
    if(rangeOut):
        err1 = "OUT"
        err2 = "OUT"
        err3 = "OUT"
    else:
        err1 = thetaIK[0]/pi*180 - theta[0]
        err2 = thetaIK[1]/pi*180 - theta[1]
        err3 = thetaIK[2]/pi*180 - theta[2]

    return(err1, err2, err3)

def calcErrP(p, p_theta):
    thetaIK, pastTheta = legSmartIK(p, p_theta, False)
    J12, J3, J4 = legFK(thetaIK, [0, 0, 0])
    
    if(pastTheta):
        err1 = "pre_pos"
        err2 = "pre_pos"
        err3 = "pre_pos"
    else:
        err1 = p[0] - J4[0]
        err2 = p[1] - J4[1]
        err3 = p[2] - J4[2]

    return(err1, err2, err3), thetaIK

def main():
    p_theta = [0, 0, 0]
    #start_time = time.perf_counter()
    for i1 in range(-180, 180, 10):
        for i2 in range(-180, 180, 10):
            for i3 in range(-180, 180, 10):
                err, theta = calcErrP([i1, i2, i3], p_theta)
                print(err)
                p_theta[0] = theta[0]
                p_theta[1] = theta[1]
                p_theta[2] = theta[2]
                
    #for i1 in range(-180, 180, 10):
    #    for i2 in range(-180, 180, 10):
    #        for i3 in range(-180, 180, 10):
    #            print("------------------------------")
    #            print(calcErrRev([i1, i2, i3]))
    #            print(i1, i2, i3)

    #end_time = time.perf_counter()
    #elapsed_time = end_time - start_time
    #print(elapsed_time)
    
    ########################
    # 計測結果:1IKに0.028ms#
    ########################
    
if __name__ == '__main__':
    main()
