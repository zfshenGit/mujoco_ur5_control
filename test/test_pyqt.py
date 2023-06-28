# -*- coding: utf-8 -*-
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

# 创建一个图形布局小部件
win = pg.GraphicsLayoutWidget(show=True, size=(600,600), title="布局小部件标题")

p = win.addPlot(title="Updating plot")
curve = p.plot(pen='y')
data = np.random.normal(size=(10,1000))
ptr = 0
def update():
    global curve, data, ptr, p
    curve.setData(data[ptr%10])
    if ptr == 0:
        p.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr += 1
# 定时器相关配置
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

if __name__ == '__main__':
    pg.exec()
