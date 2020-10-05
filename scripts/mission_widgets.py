#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import sys; sys.dont_write_bytecode = True

from PyQt5.QtWidgets import QGridLayout, QLabel, QFrame
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
import numpy as np
# from widgets import split, find_nearest

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

def split(msg_queue, cols):
    result = [[] for i in range(len(cols))]
    for msg in msg_queue:
        for i, col in enumerate(cols):
            result[i].append(msg[col])
    return result

class VisionWidget(QFrame):
    def __init__(self, ros_bridge):
        QFrame.__init__(self)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)
        #plot widget
        self.vision_label = QLabel("目标相对位置")
        self.vision_label.setAlignment(Qt.AlignCenter)
        self.plot_widget = pg.GraphicsLayoutWidget()
        # self.x_scatter = pg.ScatterPlotItem()
        # self.y_scatter = pg.ScatterPlotItem()
        # self.z_scatter = pg.ScatterPlotItem()
        # self.yaw_scatter = pg.ScatterPlotItem()
        self.x_scatter1 = pg.ScatterPlotItem()
        self.x_scatter2 = pg.PlotCurveItem()
        self.x_scatter3 = pg.PlotCurveItem()
        self.y_scatter1 = pg.ScatterPlotItem()
        self.y_scatter2 = pg.PlotCurveItem()
        self.y_scatter3 = pg.PlotCurveItem()
        self.z_scatter1 = pg.ScatterPlotItem()
        self.z_scatter2 = pg.PlotCurveItem()
        self.z_scatter3 = pg.PlotCurveItem()
        
        self.horiz_plot = self.plot_widget.addPlot(row=0, col=0)
        self.horiz_plot.setXRange(0, 10)
        self.horiz_plot.showGrid(x=1, y=1)
        self.horiz_plot.showAxis('right')
        self.horiz_plot.showAxis('top')
        # self.horiz_plot.addItem(self.x_scatter)
        # self.horiz_plot.addItem(self.y_scatter)
        self.horiz_plot.addItem(self.x_scatter1)
        self.horiz_plot.addItem(self.x_scatter2)
        self.horiz_plot.addItem(self.x_scatter3)

        self.vert_plot = self.plot_widget.addPlot(row=1, col=0)
        self.vert_plot.setXRange(0, 10)
        self.vert_plot.showGrid(x=1, y=1)
        self.vert_plot.showAxis('right')
        self.vert_plot.showAxis('top')
        # self.vert_plot.addItem(self.z_scatter)
        self.vert_plot.addItem(self.y_scatter1)
        self.vert_plot.addItem(self.y_scatter2)
        self.vert_plot.addItem(self.y_scatter3)

        self.yaw_plot = self.plot_widget.addPlot(row=2, col=0)
        self.yaw_plot.setXRange(0, 10)
        self.yaw_plot.showGrid(x=1, y=1)
        self.yaw_plot.showAxis('right')
        self.yaw_plot.showAxis('top')
        # self.yaw_plot.addItem(self.yaw_scatter)
        self.yaw_plot.addItem(self.z_scatter1)
        self.yaw_plot.addItem(self.z_scatter2)
        self.yaw_plot.addItem(self.z_scatter3)

        self.layout.addWidget(self.vision_label, 0, 0, 1, 1)
        self.layout.addWidget(self.plot_widget, 1, 0, 1, 1)

        self.vision_timer = QTimer()
        self.vision_timer.start(100)
        self.vision_timer.timeout.connect(self.update)

    def update(self):
        self.plot_curve(self.x_scatter1, self.ros_bridge.ekf_preprocess_pose_position_queue.copy(), 0, 1, pg.mkPen('r', width=2), (255,0,0,70))
        self.plot_curve(self.x_scatter2, self.ros_bridge.odometry_filtered_queue.copy(), 0, 1, pg.mkPen('g', width=2), (0,255,0,70))
        self.plot_curve(self.x_scatter3, self.ros_bridge.cmd_horiz_vel_queue.copy(), 0, 1, pg.mkPen('b', width=2), (0,0,255,70))
        self.plot_curve(self.y_scatter1, self.ros_bridge.ekf_preprocess_pose_position_queue.copy(), 0, 2, pg.mkPen('r', width=2), (255,0,0,70))
        self.plot_curve(self.y_scatter2, self.ros_bridge.odometry_filtered_queue.copy(), 0, 2, pg.mkPen('g', width=2), (0,255,0,70))
        self.plot_curve(self.y_scatter3, self.ros_bridge.cmd_horiz_vel_queue.copy(), 0, 2, pg.mkPen('b', width=2), (0,0,255,70))
        self.plot_curve(self.z_scatter1, self.ros_bridge.ekf_preprocess_pose_position_queue.copy(), 0, 3, pg.mkPen('r', width=2), (255,0,0,70))
        self.plot_curve(self.z_scatter2, self.ros_bridge.odometry_filtered_queue.copy(), 0, 3, pg.mkPen('g', width=2), (0,255,0,70))
        self.plot_curve(self.z_scatter3, self.ros_bridge.cmd_vert_vel_queue.copy(), 0, 1, pg.mkPen('b', width=2), (0,0,255,70))
        return

    def plot_curve(self, curve, queue, x, y, pen, brush):
        if queue:
            ts, val = split(queue, [x, y])
            ts = np.asarray(ts, np.uint64)
            val = np.asarray(val, np.float32)
            idx = find_nearest(ts, ts[-1] - 20 * 10**9)
            t = (ts[idx:] - ts[idx]) * 10**-9
            curve.setData(t, val[idx:], pen=pen, brush=brush, fillLevel=0)
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
        ekf_pos = self.ros_bridge.ekf_preprocess_pose_position_queue.read()
        if ekf_pos is not None:
            self.x_label.setText("x[m]: {: 2.1f}".format(ekf_pos[1]))
            self.y_label.setText("y[m]: {: 2.1f}".format(ekf_pos[2]))
            self.z_label.setText("z[m]: {: 2.1f}".format(ekf_pos[3]))
        return