from unicodedata import name
from matplotlib.pyplot import title
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import sys

import TimerIMU as timu

class PlotGraph:
    def __init__(self):
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('IMU Data')
        
        self.plt_1 = self.win.addPlot(title="Position")
        self.plt_1.enableAutoRange(axis='y')
        #self.plt_1.setAutoVisible(y=True)
        #self.plt_1.setYRange(0, 1)

        self.win.nextRow()

        self.plt_2 = self.win.addPlot(title="Gyro")
        self.plt_2.enableAutoRange(axis='y')
        
        self.curve_x = self.plt_1.plot(pen=(255, 0, 0), name="X")
        self.curve_y = self.plt_1.plot(pen=(0, 255, 0), name="Y")
        self.curve_z = self.plt_1.plot(pen=(0, 0, 255), name="Z")

        self.curve_gx = self.plt_2.plot(pen=(255, 0, 0), name="X")
        self.curve_gy = self.plt_2.plot(pen=(0, 255, 0), name="Y")
        self.curve_gz = self.plt_2.plot(pen=(0, 0, 255), name="Z")
        
        self.legend_1 = self.plt_1.addLegend()
        self.legend_1.addItem(self.curve_x, self.curve_x.name())
        self.legend_1.addItem(self.curve_y, self.curve_y.name())
        self.legend_1.addItem(self.curve_z, self.curve_z.name())

        self.legend_2 = self.plt_2.addLegend()
        self.legend_2.addItem(self.curve_gx, self.curve_gx.name())
        self.legend_2.addItem(self.curve_gy, self.curve_gy.name())
        self.legend_2.addItem(self.curve_gz, self.curve_gz.name())

        self.IMU = timu.timerIMU(0.015)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

        self.data_x = np.zeros(300)
        self.data_y = np.zeros(300)
        self.data_z = np.zeros(300)

        self.data_gx = np.zeros(300)
        self.data_gy = np.zeros(300)
        self.data_gz = np.zeros(300)

    def update(self):
        self.data_x = np.delete(self.data_x, 0)
        self.data_x = np.append(self.data_x, self.IMU.p[0])
        self.data_y = np.delete(self.data_y, 0)
        self.data_y = np.append(self.data_y, self.IMU.p[1])
        self.data_z = np.delete(self.data_z, 0)
        self.data_z = np.append(self.data_z, self.IMU.p[2])
        self.curve_x.setData(self.data_x)
        self.curve_y.setData(self.data_y)
        self.curve_z.setData(self.data_z)

        self.data_gx = np.delete(self.data_gx, 0)
        self.data_gx = np.append(self.data_gx, self.IMU.gyro[0])
        self.data_gy = np.delete(self.data_gy, 0)
        self.data_gy = np.append(self.data_gy, self.IMU.gyro[1])
        self.data_gz = np.delete(self.data_gz, 0)
        self.data_gz = np.append(self.data_gz, self.IMU.gyro[2])
        self.curve_gx.setData(self.data_gx)
        self.curve_gy.setData(self.data_gy)
        self.curve_gz.setData(self.data_gz)

if __name__ == "__main__":
    graphWin = PlotGraph()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()