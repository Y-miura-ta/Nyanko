# -*- coding: utf-8 -*-

import time
import pygame
import os

def main():
    # GUIが使えないのでダミーを設定
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    
    pygame.init()
    joy = pygame.joystick.Joystick(0)
    joy.init()
    
    try:
        while True:
            inputX = -joy.get_axis(1)
            inputY = -joy.get_axis(0)
            inputWY = -joy.get_axis(2)
            inputWZ = -joy.get_axis(3)
            btn0 = joy.get_button(0)
            btn1 = joy.get_button(1)
            btn2 = joy.get_button(2)
            btn3 = joy.get_button(3)
            # イベント更新
            pygame.event.pump()

            print("x=%f  y=%f  wy=%f  wz=%f  btn0=%f  btn1=%d  btn2=%d  btn3=%d"%(inputX, inputY, inputWY, inputWZ, btn0, btn1, btn2, btn3))

            time.sleep(0.1)

    except( KeyboardInterrupt, SystemExit):
        print( "Exit" )

if __name__ == "__main__":
    main()