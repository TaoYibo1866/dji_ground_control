#!/usr/bin/env python2.7

import sys; sys.dont_write_bytecode = True

import signal
import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget

from ros_bridge import RosBridge

from widgets import LocusWidget, TelemWidget, HorizWidget, VertWidget, YawWidget

signal.signal(signal.SIGINT, signal.SIG_DFL) # press CTRL+C quit immediately without unregister rosnode

class MainWindow(QMainWindow):
    def __init__(self, ros_bridge):
        QMainWindow.__init__(self)
        self.central_widget = QWidget()
        self.layout = QGridLayout(self.central_widget)
        self.layout_in_layout = QGridLayout()

        # Widgets: Buttons, Sliders, ...
        self.locus_widget = LocusWidget(ros_bridge)
        self.telem_widget = TelemWidget(ros_bridge)
        self.horiz_widget = HorizWidget(ros_bridge)
        self.vert_widget = VertWidget(ros_bridge)
        self.yaw_widget = YawWidget(ros_bridge)

        self.layout_in_layout.addWidget(self.horiz_widget, 0, 0)
        self.layout_in_layout.addWidget(self.vert_widget, 1, 0)
        self.layout_in_layout.addWidget(self.yaw_widget, 2, 0)

        self.layout.addWidget(self.locus_widget, 0, 0)
        self.layout.addWidget(self.telem_widget, 1, 0)
        self.layout.addLayout(self.layout_in_layout, 0, 1, 2, 1)

        self.setCentralWidget(self.central_widget)

if __name__ == '__main__':
    rospy.init_node('ground_control_gui', disable_signals=True)
    bridge = RosBridge()
    app = QApplication([])
    window = MainWindow(bridge)
    window.show()
    app.exec_()
    rospy.signal_shutdown('GUI CLOSED')