from PyQt5 import QtWidgets,QtCore,QtGui
import pyqtgraph as pg
import sys
import traceback
import psutil

class MainUi(QtWidgets.QMainWindow):

    def __init__(self):

        super().__init__()

        self.setWindowTitle("CPU使用率监控 - 州的先生https://zmister.com")

        self.main_widget = QtWidgets.QWidget() # 创建一个主部件

        self.main_layout = QtWidgets.QGridLayout() # 创建一个网格布局

        self.main_widget.setLayout(self.main_layout) # 设置主部件的布局为网格

        self.setCentralWidget(self.main_widget) # 设置窗口默认部件

        self.plot_widget = QtWidgets.QWidget() # 实例化一个widget部件作为K线图部件

        self.plot_layout = QtWidgets.QGridLayout() # 实例化一个网格布局层

        self.plot_widget.setLayout(self.plot_layout) # 设置K线图部件的布局层

        self.plot_plt = pg.PlotWidget() # 实例化一个绘图部件

        self.plot_plt.showGrid(x=True,y=True) # 显示图形网格

        self.plot_layout.addWidget(self.plot_plt) # 添加绘图部件到K线图部件的网格布局层

        # 将上述部件添加到布局层中

        self.main_layout.addWidget(self.plot_widget, 1, 0, 3, 3)

        self.setCentralWidget(self.main_widget)

        self.plot_plt.setYRange(max=100,min=0)

        self.data_list = []

        self.timer_start()

# 启动定时器 时间间隔秒

    def timer_start(self):

        self.timer = QtCore.QTimer(self)

        self.timer.timeout.connect(self.get_cpu_info)

        self.timer.start(1000)

    # 获取CPU使用率

    def get_cpu_info(self):

        try:

            cpu = "%0.2f" % psutil.cpu_percent(interval=1)

            self.data_list.append(float(cpu))

            print(float(cpu))

            self.plot_plt.plot().setData(self.data_list,pen='g')

        except Exception as e:

            print(traceback.print_exc())

def main():

    app = QtWidgets.QApplication(sys.argv)

    gui = MainUi()

    gui.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    
    main()