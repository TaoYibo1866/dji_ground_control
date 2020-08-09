#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import sys; sys.dont_write_bytecode = True

from PyQt5.QtWidgets import QGridLayout, QLabel, QFrame
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
# import numpy as np
# from widgets import split, find_nearest

class VisionWidget(QFrame):
    def __init__(self, ros_bridge):
        QFrame.__init__(self)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)
        #plot widget
        self.vision_label = QLabel("目标相对位置")
        self.vision_label.setAlignment(Qt.AlignCenter)
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.x_scatter = pg.ScatterPlotItem()
        self.y_scatter = pg.ScatterPlotItem()
        self.z_scatter = pg.ScatterPlotItem()
        self.yaw_scatter = pg.ScatterPlotItem()

        self.horiz_plot = self.plot_widget.addPlot(row=0, col=0)
        self.horiz_plot.setXRange(0, 10)
        self.horiz_plot.showGrid(x=1, y=1)
        self.horiz_plot.showAxis('right')
        self.horiz_plot.showAxis('top')
        self.horiz_plot.addItem(self.x_scatter)
        self.horiz_plot.addItem(self.y_scatter)

        self.vert_plot = self.plot_widget.addPlot(row=1, col=0)
        self.vert_plot.setXRange(0, 10)
        self.vert_plot.showGrid(x=1, y=1)
        self.vert_plot.showAxis('right')
        self.vert_plot.showAxis('top')
        self.vert_plot.addItem(self.z_scatter)

        self.yaw_plot = self.plot_widget.addPlot(row=2, col=0)
        self.yaw_plot.setXRange(0, 10)
        self.yaw_plot.showGrid(x=1, y=1)
        self.yaw_plot.showAxis('right')
        self.yaw_plot.showAxis('top')
        self.yaw_plot.addItem(self.yaw_scatter)

        self.layout.addWidget(self.vision_label, 0, 0, 1, 1)
        self.layout.addWidget(self.plot_widget, 1, 0, 1, 1)

        self.vision_timer = QTimer()
        self.vision_timer.start(100)
        self.vision_timer.timeout.connect(self.update)

    def update(self):
        return

class MissionTelemWidget(QFrame):
    def __init__(self, ros_bridge):
        QFrame.__init__(self)
        self.setFrameShape(QFrame.Box)
        self.setFrameShadow(QFrame.Raised)
        self.setLineWidth(2)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)

        self.x_label = QLabel("x[m]:")
        self.y_label = QLabel("y[m]:")
        self.z_label = QLabel("z[m]:")

        self.x_label.setFrameShape(QFrame.Box)
        self.y_label.setFrameShape(QFrame.Box)
        self.z_label.setFrameShape(QFrame.Box)

        self.layout.addWidget(self.x_label, 0, 0)
        self.layout.addWidget(self.y_label, 0, 1)
        self.layout.addWidget(self.z_label, 0, 2)

        self.telem_timer = QTimer()
        self.telem_timer.start(50)
        self.telem_timer.timeout.connect(self.update)

    def update(self):
        ibvs_fb = self.ros_bridge.ibvs_feedback_queue.read()
        if ibvs_fb is not None:
            self.x_label.setText("x[m]: {: 2.1f}".format(ibvs_fb[1]))
            self.y_label.setText("y[m]: {: 2.1f}".format(ibvs_fb[2]))
            self.z_label.setText("z[m]: {: 2.1f}".format(ibvs_fb[3]))
        return