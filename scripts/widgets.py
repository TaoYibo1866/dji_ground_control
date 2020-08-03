#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import sys; sys.dont_write_bytecode = True

from PyQt5.QtWidgets import QGridLayout, QLabel, QFrame
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
#from mem_top import mem_top
import numpy as np

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

        self.locus_plot = self.plot_widget.addPlot(row=0, col=0)
        self.locus_plot.setXRange(-30, 10)
        self.locus_plot.setYRange(-10, 30)
        self.locus_plot.setAspectLocked()
        self.locus_plot.showGrid(x=1, y=1)
        self.locus_plot.showAxis('right')
        self.locus_plot.showAxis('top')
        self.locus_plot.addItem(self.locus_curve)
        self.locus_plot.addItem(self.locus_scatter)

        self.layout.addWidget(self.locus_label, 0, 0, 1, 1)
        self.layout.addWidget(self.plot_widget, 1, 0, 1, 1)

        self.locus_timer = QTimer()
        self.locus_timer.start(100)
        self.locus_timer.timeout.connect(self.update)

    def update(self):
        local_position_queue = self.ros_bridge.local_position_queue.copy()
        if not local_position_queue:
            return
        local_position_array = np.asarray(local_position_queue, np.float32)
        e_array = local_position_array[:, 1].reshape(-1)
        n_array = local_position_array[:, 2].reshape(-1)
        self.locus_curve.setData(-n_array, e_array, pen=pg.mkPen('g', width=3))
        self.locus_scatter.setData([-n_array[-1]], [e_array[-1]], pen=pg.mkPen('r', width=5))
        return

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
        self.gps_position_altitude_label = QLabel("高度:")
        self.angular_velocity_x_label = QLabel("E[rad/s]:")
        self.angular_velocity_y_label = QLabel("N[rad/s]:")
        self.angular_velocity_z_label = QLabel("U[rad/s]:")
        self.attitude_x_label = QLabel("四元数x:")
        self.attitude_y_label = QLabel("四元数y:")
        self.attitude_z_label = QLabel("四元数z:")
        self.attitude_w_label = QLabel("四元数w:")
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
        self.gps_position_altitude_label.setFrameShape(QFrame.Box)
        self.angular_velocity_x_label.setFrameShape(QFrame.Box)
        self.angular_velocity_y_label.setFrameShape(QFrame.Box)
        self.angular_velocity_z_label.setFrameShape(QFrame.Box)
        self.attitude_x_label.setFrameShape(QFrame.Box)
        self.attitude_y_label.setFrameShape(QFrame.Box)
        self.attitude_z_label.setFrameShape(QFrame.Box)
        self.attitude_w_label.setFrameShape(QFrame.Box)
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
        self.layout.addWidget(self.height_label, 1, 0, 1, 1)
        self.layout.addWidget(self.acceleration_x_label, 2, 0, 1, 1)
        self.layout.addWidget(self.acceleration_y_label, 2, 1, 1, 1)
        self.layout.addWidget(self.acceleration_z_label, 2, 2, 1, 1)
        self.layout.addWidget(self.velocity_x_label, 3, 0, 1, 1)
        self.layout.addWidget(self.velocity_y_label, 3, 1, 1, 1)
        self.layout.addWidget(self.velocity_z_label, 3, 2, 1, 1)
        self.layout.addWidget(self.gps_position_latitude_label, 4, 0, 1, 1)
        self.layout.addWidget(self.gps_position_longitude_label, 4, 1, 1, 1)
        self.layout.addWidget(self.gps_position_altitude_label, 4, 2, 1, 1)
        self.layout.addWidget(self.angular_velocity_x_label, 5, 0, 1, 1)
        self.layout.addWidget(self.angular_velocity_y_label, 5, 1, 1, 1)
        self.layout.addWidget(self.angular_velocity_z_label, 5, 2, 1, 1)
        self.layout.addWidget(self.attitude_x_label, 6, 0, 1, 1)
        self.layout.addWidget(self.attitude_y_label, 6, 1, 1, 1)
        self.layout.addWidget(self.attitude_z_label, 6, 2, 1, 1)
        self.layout.addWidget(self.attitude_w_label, 6, 3, 1, 1)
        self.layout.addWidget(self.gps_health_label, 7, 0, 1, 1)
        self.layout.addWidget(self.battery_state_voltage_label, 8, 0, 1, 1)
        self.layout.addWidget(self.battery_state_current_label, 8, 1, 1, 1)
        self.layout.addWidget(self.battery_state_percentage_label, 8, 2, 1, 1)
        self.layout.addWidget(self.flight_status_label, 9, 0, 1, 1)
        self.layout.addWidget(self.rc_axes0_label, 10, 0, 1, 1)
        self.layout.addWidget(self.rc_axes1_label, 10, 1, 1, 1)
        self.layout.addWidget(self.rc_axes2_label, 10, 2, 1, 1)
        self.layout.addWidget(self.rc_axes3_label, 10, 3, 1, 1)
        self.layout.addWidget(self.rc_axes4_label, 10, 4, 1, 1)
        self.layout.addWidget(self.rc_axes5_label, 10, 5, 1, 1)

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
            self.gps_position_altitude_label.setText("高度: {: 2.1f}".format(gps_position[3]))
        
        angular_velocity = self.ros_bridge.angular_velocity_queue.read()
        if angular_velocity is not None:
            self.angular_velocity_x_label.setText("E[rad/s]: {: 2.1f}".format(angular_velocity[1]))
            self.angular_velocity_y_label.setText("N[rad/s]: {: 2.1f}".format(angular_velocity[2]))
            self.angular_velocity_z_label.setText("U[rad/s]: {: 2.1f}".format(angular_velocity[3]))

        attitude = self.ros_bridge.attitude_queue.read()
        if attitude is not None:
            self.attitude_x_label.setText("四元数x: {: 2.3f}".format(attitude[1]))
            self.attitude_y_label.setText("四元数y: {: 2.3f}".format(attitude[2]))
            self.attitude_z_label.setText("四元数z: {: 2.3f}".format(attitude[3]))
            self.attitude_w_label.setText("四元数w: {: 2.3f}".format(attitude[4]))

        gps_health = self.ros_bridge.gps_health_queue.read()
        if gps_health is not None:
            self.gps_health_label.setText("GPS强度: {: 2.1f}".format(gps_health))
        
        battery_state = self.ros_bridge.battery_state_queue.read()
        if gps_position is not None:
            self.battery_state_voltage_label.setText("电压[v]:: {: 2.1f}".format(gps_position[1]))
            self.battery_state_current_label.setText("电流[A]: {: 2.1f}".format(gps_position[2]))
            self.battery_state_percentage_label.setText("百分比[%]: {: 2.1f}".format(gps_position[3]))

        flight_status = self.ros_bridge.flight_status_queue.read()
        if flight_status is not None:
            self.flight_status_label.setText("当前飞行模式: {: 2.1f}".format(flight_status))

        rc = self.ros_bridge.rc_queue.read()
        if rc is not None:
            self.rc_axes0_label.setText("Roll Channel: {: 2.1f}".format(rc[1]))
            self.rc_axes1_label.setText("Pitch Channel: {: 2.1f}".format(rc[2]))
            self.rc_axes2_label.setText("Yaw Channel: {: 2.1f}".format(rc[3]))
            self.rc_axes3_label.setText("Throttle Channel: {: 2.1f}".format(rc[4]))
            self.rc_axes4_label.setText("Mode switch: {: 2.1f}".format(rc[5]))
            self.rc_axes5_label.setText("Landing gear (H) switch: {: 2.1f}".format(rc[6]))

        return