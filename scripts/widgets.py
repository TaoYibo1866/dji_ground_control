#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import sys; sys.dont_write_bytecode = True

from PyQt5.QtWidgets import QGridLayout, QLabel, QFrame
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
#from mem_top import mem_top
import numpy as np

from tf.transformations import euler_from_quaternion, rotation_matrix
import math

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
        self.height_label = QLabel("Height[m]:")
        self.acceleration_x_label = QLabel("E[m/s^2]:")
        self.acceleration_y_label = QLabel("N[m/s^2]:")
        self.acceleration_z_label = QLabel("U[m/s^2]:")
        self.velocity_x_label = QLabel("E[m/s]:")
        self.velocity_y_label = QLabel("N[m/s]:")
        self.velocity_z_label = QLabel("U[m/s]:")
        self.gps_position_latitude_label = QLabel("纬度:")
        self.gps_position_longitude_label = QLabel("经度:")
        #self.gps_position_altitude_label = QLabel("高度:")
        self.angular_velocity_x_label = QLabel("E[rad/s]:")
        self.angular_velocity_y_label = QLabel("N[rad/s]:")
        self.angular_velocity_z_label = QLabel("U[rad/s]:")
        self.attitude_x_label = QLabel("欧拉角r:")
        self.attitude_y_label = QLabel("欧拉角p:")
        self.attitude_z_label = QLabel("欧拉角y:")
        #self.attitude_w_label = QLabel("四元数w:")
        self.gps_health_label = QLabel("GPS强度:")
        self.battery_state_voltage_label = QLabel("电压[v]:")
        self.battery_state_current_label = QLabel("电流[A]:")
        self.battery_state_percentage_label = QLabel("百分比:")
        self.flight_status_label = QLabel("当前飞行模式:")
        self.rc_axes0_label = QLabel("Roll Channel:")
        self.rc_axes1_label = QLabel("Pitch Channel:")
        self.rc_axes2_label = QLabel("Yaw Channel:")
        self.rc_axes3_label = QLabel("Throttle Channel:")
        self.rc_axes4_label = QLabel("Mode switch :")
        self.rc_axes5_label = QLabel("Landing gear (H) switch:")
 
        self.local_position_x_label.setFrameShape(QFrame.Box)
        self.local_position_y_label.setFrameShape(QFrame.Box)
        self.local_position_z_label.setFrameShape(QFrame.Box)
        self.height_label.setFrameShape(QFrame.Box)
        self.acceleration_x_label.setFrameShape(QFrame.Box)
        self.acceleration_y_label.setFrameShape(QFrame.Box)
        self.acceleration_z_label.setFrameShape(QFrame.Box)
        self.velocity_x_label.setFrameShape(QFrame.Box)
        self.velocity_y_label.setFrameShape(QFrame.Box)
        self.velocity_z_label.setFrameShape(QFrame.Box)
        self.gps_position_latitude_label.setFrameShape(QFrame.Box)
        self.gps_position_longitude_label.setFrameShape(QFrame.Box)
        #self.gps_position_altitude_label.setFrameShape(QFrame.Box)
        self.angular_velocity_x_label.setFrameShape(QFrame.Box)
        self.angular_velocity_y_label.setFrameShape(QFrame.Box)
        self.angular_velocity_z_label.setFrameShape(QFrame.Box)
        self.attitude_x_label.setFrameShape(QFrame.Box)
        self.attitude_y_label.setFrameShape(QFrame.Box)
        self.attitude_z_label.setFrameShape(QFrame.Box)
        #self.attitude_w_label.setFrameShape(QFrame.Box)
        self.gps_health_label.setFrameShape(QFrame.Box)
        self.battery_state_voltage_label.setFrameShape(QFrame.Box)
        self.battery_state_current_label.setFrameShape(QFrame.Box)
        self.battery_state_percentage_label.setFrameShape(QFrame.Box)
        self.flight_status_label.setFrameShape(QFrame.Box)
        self.rc_axes0_label.setFrameShape(QFrame.Box)
        self.rc_axes1_label.setFrameShape(QFrame.Box)
        self.rc_axes2_label.setFrameShape(QFrame.Box)
        self.rc_axes3_label.setFrameShape(QFrame.Box)
        self.rc_axes4_label.setFrameShape(QFrame.Box)
        self.rc_axes5_label.setFrameShape(QFrame.Box)

        self.layout.addWidget(self.local_position_x_label, 0, 0, 1, 1)
        self.layout.addWidget(self.local_position_y_label, 0, 1, 1, 1)
        self.layout.addWidget(self.local_position_z_label, 0, 2, 1, 1)

        self.layout.addWidget(self.height_label, 3, 2, 1, 1)

        self.layout.addWidget(self.acceleration_x_label, 2, 0, 1, 1)
        self.layout.addWidget(self.acceleration_y_label, 2, 1, 1, 1)
        self.layout.addWidget(self.acceleration_z_label, 2, 2, 1, 1)
        self.layout.addWidget(self.velocity_x_label, 1, 0, 1, 1)
        self.layout.addWidget(self.velocity_y_label, 1, 1, 1, 1)
        self.layout.addWidget(self.velocity_z_label, 1, 2, 1, 1)
        self.layout.addWidget(self.gps_position_latitude_label, 3, 0, 1, 1)
        self.layout.addWidget(self.gps_position_longitude_label, 3, 1, 1, 1)
        #self.layout.addWidget(self.gps_position_altitude_label, 3, 2, 1, 1)
        self.layout.addWidget(self.angular_velocity_x_label, 4, 0, 1, 1)
        self.layout.addWidget(self.angular_velocity_y_label, 4, 1, 1, 1)
        self.layout.addWidget(self.angular_velocity_z_label, 4, 2, 1, 1)
        self.layout.addWidget(self.attitude_x_label, 5, 0, 1, 1)
        self.layout.addWidget(self.attitude_y_label, 5, 1, 1, 1)
        self.layout.addWidget(self.attitude_z_label, 5, 2, 1, 1)
        #self.layout.addWidget(self.attitude_w_label, 5, 3, 1, 1)
        self.layout.addWidget(self.gps_health_label, 6, 0, 1, 1)
        self.layout.addWidget(self.battery_state_voltage_label, 7, 0, 1, 1)
        self.layout.addWidget(self.battery_state_current_label, 7, 1, 1, 1)
        self.layout.addWidget(self.battery_state_percentage_label, 7, 2, 1, 1)
        self.layout.addWidget(self.flight_status_label, 6, 1, 1, 1)
        self.layout.addWidget(self.rc_axes0_label, 8, 0, 1, 1)
        self.layout.addWidget(self.rc_axes1_label, 8, 1, 1, 1)
        self.layout.addWidget(self.rc_axes2_label, 8, 2, 1, 1)
        self.layout.addWidget(self.rc_axes3_label, 9, 0, 1, 1)
        self.layout.addWidget(self.rc_axes4_label, 9, 1, 1, 1)
        self.layout.addWidget(self.rc_axes5_label, 9, 2, 1, 1)

        self.telem_timer = QTimer()
        self.telem_timer.start(50)
        self.telem_timer.timeout.connect(self.update)

    def update(self):
        local_position = self.ros_bridge.local_position_queue.read()
        if local_position is not None:
            self.local_position_x_label.setText("E[m]: {: 2.1f}".format(local_position[1]))
            self.local_position_y_label.setText("N[m]: {: 2.1f}".format(local_position[2]))
            self.local_position_z_label.setText("U[m]: {: 2.1f}".format(local_position[3]))
        
        height = self.ros_bridge.height_queue.read()
        if height is not None:
            self.height_label.setText("Height[m]: {: 2.1f}".format(height))

        acceleration = self.ros_bridge.acceleration_queue.read()
        if acceleration is not None:
            self.acceleration_x_label.setText("E[m/s^2]: {: 2.1f}".format(acceleration[1]))
            self.acceleration_y_label.setText("N[m/s^2]: {: 2.1f}".format(acceleration[2]))
            self.acceleration_z_label.setText("U[m/s^2]: {: 2.1f}".format(acceleration[3]))
        
        velocity = self.ros_bridge.velocity_queue.read()
        if velocity is not None:
            self.velocity_x_label.setText("E[m/s]: {: 2.1f}".format(velocity[1]))
            self.velocity_y_label.setText("N[m/s]: {: 2.1f}".format(velocity[2]))
            self.velocity_z_label.setText("U[m/s]: {: 2.1f}".format(velocity[3]))

        gps_position = self.ros_bridge.gps_position_queue.read()
        if gps_position is not None:
            self.gps_position_latitude_label.setText("纬度: {: 2.1f}".format(gps_position[1]))
            self.gps_position_longitude_label.setText("经度: {: 2.1f}".format(gps_position[2]))
            #self.gps_position_altitude_label.setText("高度: {: 2.1f}".format(gps_position[3]))
        
        angular_velocity = self.ros_bridge.angular_velocity_queue.read()
        if angular_velocity is not None:
            self.angular_velocity_x_label.setText("E[rad/s]: {: 2.1f}".format(angular_velocity[1]))
            self.angular_velocity_y_label.setText("N[rad/s]: {: 2.1f}".format(angular_velocity[2]))
            self.angular_velocity_z_label.setText("U[rad/s]: {: 2.1f}".format(angular_velocity[3]))

        attitude = self.ros_bridge.attitude_queue.read()
        if attitude is not None:
            q_i2b = attitude[1:]
            y, p, r = euler_from_quaternion(q_i2b, axes='rzyx')
            r_=r*180/math.pi
            p_=p*180/math.pi
            y_=y*180/math.pi
            self.attitude_x_label.setText("滚转: {: 2.1f}".format(r_))
            self.attitude_y_label.setText("俯仰: {: 2.1f}".format(p_))
            self.attitude_z_label.setText("偏航: {: 2.1f}".format(y_))
            #self.attitude_w_label.setText("欧拉角w: {: 2.3f}".format(attitude[4]))

        gps_health = self.ros_bridge.gps_health_queue.read()
        if gps_health is not None:
            self.gps_health_label.setText("GPS强度: {}".format(gps_health))
        
        battery_state = self.ros_bridge.battery_state_queue.read()
        if gps_position is not None:
            self.battery_state_voltage_label.setText("电压[V]: {: 2.1f}".format(gps_position[1]))
            self.battery_state_current_label.setText("电流[A]: {: 2.1f}".format(gps_position[2]))
            self.battery_state_percentage_label.setText("百分比[%]: {: 2.1f}".format(gps_position[3]))

        flight_status = self.ros_bridge.flight_status_queue.read()
        if flight_status is not None:
            self.flight_status_label.setText("飞行模式: {: 2.1f}".format(flight_status))

        rc = self.ros_bridge.rc_queue.read()
        if rc is not None:
            self.rc_axes0_label.setText("Roll Channel: {: 2.1f}".format(rc[1]))
            self.rc_axes1_label.setText("Pitch Channel: {: 2.1f}".format(rc[2]))
            self.rc_axes2_label.setText("Yaw Channel: {: 2.1f}".format(rc[3]))
            self.rc_axes3_label.setText("Throttle Channel: {: 2.1f}".format(rc[4]))
            self.rc_axes4_label.setText("Mode switch: {: 2.1f}".format(rc[5]))
            self.rc_axes5_label.setText("支撑架: {: 2.1f}".format(rc[6]))
      
        return