#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import sys; sys.dont_write_bytecode = True

from PyQt5.QtWidgets import QGridLayout, QLabel, QFrame
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
#from mem_top import mem_top
import numpy as np

from tf.transformations import euler_from_quaternion, rotation_matrix

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

class HorizWidget(QFrame):
    def __init__(self, ros_bridge):
        QFrame.__init__(self)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)

        #plot widget
        self.horiz_label = QLabel("水平速度(ENU)")
        self.horiz_label.setAlignment(Qt.AlignCenter)
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.vx_curve = pg.PlotCurveItem()
        self.vy_curve = pg.PlotCurveItem()

        self.horiz_plot = self.plot_widget.addPlot(row=0, col=0)
        self.horiz_plot.setXRange(0, 10)
        self.horiz_plot.showGrid(x=1, y=1)
        self.horiz_plot.showAxis('right')
        self.horiz_plot.showAxis('top')

        self.horiz_plot.addItem(self.vx_curve)
        self.horiz_plot.addItem(self.vy_curve)

        self.layout.addWidget(self.horiz_label, 0, 0, 1, 1)
        self.layout.addWidget(self.plot_widget, 1, 0, 1, 1)

        self.horiz_timer = QTimer()
        self.horiz_timer.start(100)
        self.horiz_timer.timeout.connect(self.update)

    def update(self):
        velocity_queue = self.ros_bridge.velocity_queue.copy()
        if velocity_queue is not None:
            ts, vx, vy = split(velocity_queue, [0, 1, 2])

            ts = np.asarray(ts, np.uint64)
            vx = np.asarray(vx, np.float32)
            vy = np.asarray(vy, np.float32)

            idx = find_nearest(ts, ts[-1] - 10 * 10**9)
            t = (ts[idx:] - ts[idx]) * 10**-9
            self.vx_curve.setData(t, vx[idx:], pen=pg.mkPen('r', width=2), brush=(255,0,0,70), fillLevel=0)
            self.vy_curve.setData(t, vy[idx:], pen=pg.mkPen('g', width=2), brush=(0,255,0,70), fillLevel=0)
        return

class VertWidget(QFrame):
    def __init__(self, ros_bridge):
        QFrame.__init__(self)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)

        #plot widget
        self.vert_label = QLabel("竖直速度(ENU)")
        self.vert_label.setAlignment(Qt.AlignCenter)
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.vx_curve = pg.PlotCurveItem()

        self.vert_plot = self.plot_widget.addPlot(row=0, col=0)
        self.vert_plot.setXRange(0, 10)
        self.vert_plot.showGrid(x=1, y=1)
        self.vert_plot.showAxis('right')
        self.vert_plot.showAxis('top')

        self.vert_plot.addItem(self.vx_curve)

        self.layout.addWidget(self.vert_label, 0, 0, 1, 1)
        self.layout.addWidget(self.plot_widget, 1, 0, 1, 1)

        self.vert_timer = QTimer()
        self.vert_timer.start(100)
        self.vert_timer.timeout.connect(self.update)

    def update(self):
        return

class YawWidget(QFrame):
    def __init__(self, ros_bridge):
        QFrame.__init__(self)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)

        #plot widget
        self.yaw_label = QLabel("偏航角")
        self.yaw_label.setAlignment(Qt.AlignCenter)
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.yaw_curve = pg.PlotCurveItem()

        self.yaw_plot = self.plot_widget.addPlot(row=0, col=0)
        self.yaw_plot.setXRange(0, 10)
        self.yaw_plot.showGrid(x=1, y=1)
        self.yaw_plot.showAxis('right')
        self.yaw_plot.showAxis('top')

        self.yaw_plot.addItem(self.yaw_curve)

        self.layout.addWidget(self.yaw_label, 0, 0, 1, 1)
        self.layout.addWidget(self.plot_widget, 1, 0, 1, 1)

        self.yaw_timer = QTimer()
        self.yaw_timer.start(100)
        self.yaw_timer.timeout.connect(self.update)

    def update(self):
        return

class LocusWidget(QFrame):
    def __init__(self, ros_bridge):
        QFrame.__init__(self)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)
        #plot widget
        self.locus_label = QLabel("航迹俯视图")
        self.locus_label.setAlignment(Qt.AlignCenter)
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.locus_curve = pg.PlotCurveItem()
        self.locus_scatter = pg.ScatterPlotItem()
        self.locus_arrow = pg.PlotCurveItem()

        self.locus_plot = self.plot_widget.addPlot(row=0, col=0)
        self.locus_plot.setXRange(-30, 10)
        self.locus_plot.setYRange(-10, 30)
        self.locus_plot.setAspectLocked()
        self.locus_plot.showGrid(x=1, y=1)
        self.locus_plot.showAxis('right')
        self.locus_plot.showAxis('top')
        self.locus_plot.addItem(self.locus_curve)
        self.locus_plot.addItem(self.locus_scatter)
        self.locus_plot.addItem(self.locus_arrow)

        self.layout.addWidget(self.locus_label, 0, 0, 1, 1)
        self.layout.addWidget(self.plot_widget, 1, 0, 1, 1)

        self.locus_timer = QTimer()
        self.locus_timer.start(100)
        self.locus_timer.timeout.connect(self.update)

    def update(self):
        local_position_queue = self.ros_bridge.local_position_queue.copy()
        attitude = self.ros_bridge.attitude_queue.read()
        if local_position_queue is not None and attitude is not None:
            e, n = split(local_position_queue, [1, 2])
            e = np.asarray(e, np.float32)
            n = np.asarray(n, np.float32)
            self.locus_curve.setData(-n, e, pen=pg.mkPen('g', width=3))
            self.locus_scatter.setData([-n[-1]], [e[-1]], pen=pg.mkPen('g', width=2))
            
            q_i2b = attitude[1:]
            yaw, _, _ = euler_from_quaternion(q_i2b, axes='rzyx')
            arrow = self.calc_arrow(x=-n[-1], y=e[-1], yaw=yaw)
            self.locus_arrow.setData(arrow[0,:], arrow[1,:], pen=pg.mkPen('r', width=2))
        
        return
    
    def calc_arrow(self, x=0, y=0, yaw=0):
        arrow = np.transpose(np.float32([[0, 2, 0], [1, -1, 0], [0, 0, 0], [-1, -1, 0], [0, 2, 0]]))
        Rz = rotation_matrix(yaw, [0, 0, 1])[:3,:3]
        return np.matmul(Rz, arrow) + np.float32([[x], [y], [0]])

class TelemWidget(QFrame):
    def __init__(self, ros_bridge):
        QFrame.__init__(self)
        self.setFrameShape(QFrame.Box)
        self.setFrameShadow(QFrame.Raised)
        self.setLineWidth(2)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)

        #label widget
        self.local_position_x_label = QLabel("E[m]:")
        self.local_position_y_label = QLabel("N[m]:")
        self.local_position_z_label = QLabel("U[m]:")
        
        self.local_position_x_label.setFrameShape(QFrame.Box)
        self.local_position_y_label.setFrameShape(QFrame.Box)
        self.local_position_z_label.setFrameShape(QFrame.Box)

        self.layout.addWidget(self.local_position_x_label, 0, 0, 1, 1)
        self.layout.addWidget(self.local_position_y_label, 0, 1, 1, 1)
        self.layout.addWidget(self.local_position_z_label, 0, 2, 1, 1)

        self.telem_timer = QTimer()
        self.telem_timer.start(50)
        self.telem_timer.timeout.connect(self.update)

    def update(self):
        local_position = self.ros_bridge.local_position_queue.read()
        if local_position is not None:
            self.local_position_x_label.setText("E[m]: {: 2.1f}".format(local_position[1]))
            self.local_position_y_label.setText("N[m]: {: 2.1f}".format(local_position[2]))
            self.local_position_z_label.setText("U[m]: {: 2.1f}".format(local_position[3]))
        return