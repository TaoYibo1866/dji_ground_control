#!/usr/bin/env python2.7

import sys; sys.dont_write_bytecode = True

import signal
import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget

from ros_bridge import RosBridge

from widgets import TelemWidget, LocusWidget #, GenPlotWidget, TabWidget

signal.signal(signal.SIGINT, signal.SIG_DFL) # press CTRL+C quit immediately without unregister rosnode

class MainWindow(QMainWindow):
    def __init__(self, ros_bridge):
        QMainWindow.__init__(self)
        self.central_widget = QWidget()
        self.layout = QGridLayout(self.central_widget)
        self.col0_layout = QGridLayout()
        # self.col1_layout = QGridLayout()
        # self.col2_layout = QGridLayout()

        # Widgets: Buttons, Sliders, ...
        self.locus_widget = LocusWidget(ros_bridge)
        self.telem_widget = TelemWidget(ros_bridge)
        # self.plot_widget_0 = GenPlotWidget(ros_bridge, 1, 0)
        # self.plot_widget_1 = GenPlotWidget(ros_bridge, 1, 1)
        # self.plot_widget_2 = GenPlotWidget(ros_bridge, 1, 2)
        # self.tab_widget = TabWidget(ros_bridge)

        self.col0_layout.addWidget(self.locus_widget, 0, 0)
        self.col0_layout.addWidget(self.telem_widget, 1, 0)
        # self.col1_layout.addWidget(self.plot_widget_0, 0, 0)
        # self.col1_layout.addWidget(self.plot_widget_1, 1, 0)
        # self.col1_layout.addWidget(self.plot_widget_2, 2, 0)
        # self.col2_layout.addWidget(self.tab_widget, 0, 0)

        self.layout.addLayout(self.col0_layout, 0, 0)
        # self.layout.addLayout(self.col1_layout, 0, 1)
        # self.layout.addLayout(self.col2_layout, 0, 2)

        self.setCentralWidget(self.central_widget)

if __name__ == '__main__':
    rospy.init_node('ground_control_gui', disable_signals=True)
    bridge = RosBridge()
    app = QApplication([])
    window = MainWindow(bridge)
    window.show()
    app.exec_()
    rospy.signal_shutdown('GUI CLOSED')