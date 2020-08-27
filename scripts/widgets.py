#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import sys; sys.dont_write_bytecode = True

from PyQt5.QtWidgets import QGridLayout, QLabel, QFrame, QComboBox, QPushButton, QTabWidget, QPlainTextEdit
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
#from mem_top import mem_top
import numpy as np
from tf.transformations import rotation_matrix
import math
from datetime import datetime
from mission_widgets import VisionWidget, MissionTelemWidget
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

class GenPlotWidget(QFrame):
    def __init__(self, ros_bridge, quan=0, axis=0):
        QFrame.__init__(self)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)
        
        #plot widget
        self.scope_label = QLabel()
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.quan_combo = QComboBox()
        self.axis_combo = QComboBox()
        self.enter_button = QPushButton()
        self.keep_button = QPushButton()

        self.scope_label.setAlignment(Qt.AlignCenter)

        self.plot = self.plot_widget.addPlot(row=0, col=0)
        self.plot.setXRange(0, 20)
        self.plot.showGrid(x=1, y=1)
        self.plot.showAxis('right')
        self.plot.showAxis('top')

        self.curve = pg.PlotCurveItem()
        self.curve1 = pg.PlotCurveItem()
        self.plot.addItem(self.curve)
        self.plot.addItem(self.curve1)

        self.quan_combo.addItems(['Velocity_ENU',
                                  'Local_Position',
                                  'Attitude'])
        self.axis_combo.addItems(['X', 'Y', 'Z'])
        
        self.quan_combo.setCurrentIndex(quan)
        self.axis_combo.setCurrentIndex(axis)

        self.enter_button.setText('Enter')
        self.enter_button.clicked.connect(self.switch_case)

        self.stop_button = ToggleButton(["暂停", "继续"], self)
        self.stop_button.clicked.connect(self.stop)

        self.case = [self.quan_combo.currentIndex(), self.axis_combo.currentIndex()]
        self.scope_label.setText("{}->{}".format(self.quan_combo.currentText(), self.axis_combo.currentText()))

        self.layout.addWidget(self.scope_label, 0, 0, 1, 4)
        self.layout.addWidget(self.plot_widget, 1, 0, 1, 4)
        self.layout.addWidget(self.quan_combo, 2, 0, 1, 1)
        self.layout.addWidget(self.axis_combo, 2, 1, 1, 1)
        self.layout.addWidget(self.enter_button, 2, 2, 1, 1)
        self.layout.addWidget(self.stop_button, 2, 3, 1, 1)
        
        self.timer = QTimer() 
        self.timer.start(100)        
        self.timer.timeout.connect(self.update)
    def update(self):
        quan, axis = self.case
        flag = self.stop_button.flag
        if flag == 0:
            if quan == 0:
                if axis == 0:
                    self.plot_curve(self.curve, self.ros_bridge.velocity_queue.copy(), 0, 1, pg.mkPen('r', width=2), (255,0,0,70))
                    self.plot_curve1(self.curve1, self.ros_bridge.Horizontal_velocity_queue.copy(), 0, 1, pg.mkPen('w', width=2), (255,255,255,70))
                elif axis == 1:
                    self.plot_curve(self.curve, self.ros_bridge.velocity_queue.copy(), 0, 2, pg.mkPen('g', width=2), (0,255,0,70))
                    self.plot_curve1(self.curve1, self.ros_bridge.Horizontal_velocity_queue.copy(), 0, 2, pg.mkPen('w', width=2), (255,255,255,70))
                elif axis == 2:
                    self.plot_curve(self.curve, self.ros_bridge.velocity_queue.copy(), 0, 3, pg.mkPen('b', width=2), (0,0,255,70))
                    self.plot_curve1(self.curve1, self.ros_bridge.Vertical_velocity_queue.copy(), 0, 1, pg.mkPen('w', width=2), (255,255,255,70))     
            elif quan == 1:
                if axis == 0:
                    self.plot_curve(self.curve, self.ros_bridge.local_position_queue.copy(), 0, 1, pg.mkPen('r', width=2), (255,0,0,70))           
                elif axis == 1:
                    self.plot_curve(self.curve, self.ros_bridge.local_position_queue.copy(), 0, 2, pg.mkPen('g', width=2), (0,255,0,70))               
                elif axis == 2:
                    self.plot_curve(self.curve, self.ros_bridge.local_position_queue.copy(), 0, 3, pg.mkPen('b', width=2), (0,0,255,70))
            elif quan == 2:
                if axis == 0:
                    self.plot_curve(self.curve, self.ros_bridge.attitude_queue.copy(), 0, -3, pg.mkPen('r', width=2), (255,0,0,70))
                    self.plot_curve1(self.curve1, self.ros_bridge.Yaw_angle_queue.copy(), 0, -1, pg.mkPen('w', width=2), (255,255,255,70))
                elif axis == 1:
                    self.plot_curve(self.curve, self.ros_bridge.attitude_queue.copy(), 0, -2, pg.mkPen('g', width=2), (0,255,0,70))
                    self.plot_curve1(self.curve1, self.ros_bridge.Yaw_angle_queue.copy(), 0, -1, pg.mkPen('w', width=2), (255,255,255,70))
                elif axis == 2:
                    self.plot_curve(self.curve, self.ros_bridge.attitude_queue.copy(), 0, -1, pg.mkPen('b', width=2), (0,0,255,70))
                    self.plot_curve1(self.curve1, self.ros_bridge.Yaw_angle_queue.copy(), 0, -1, pg.mkPen('w', width=2), (255,255,255,70))
        return        
    
    def switch_case(self, _):
        self.case = [self.quan_combo.currentIndex(), self.axis_combo.currentIndex()]
        self.scope_label.setText("{}->{}".format(self.quan_combo.currentText(), self.axis_combo.currentText()))    
    
    def stop(self):       
        if self.stop_button.state == 0:      
            self.stop_button.flag=1
            self.stop_button.state = 1
            self.stop_button.setText(self.stop_button.text[1])
        elif self.stop_button.state == 1:           
            self.stop_button.flag=0
            self.stop_button.state = 0
            self.stop_button.setText(self.stop_button.text[0])
        return 
    
    def plot_curve(self, curve, queue, x, y, pen, brush):
        if queue is not None:
            ts, val = split(queue, [x, y])
            ts = np.asarray(ts, np.uint64)
            val = np.asarray(val, np.float32)
            idx = find_nearest(ts, ts[-1] - 20 * 10**9)
            t = (ts[idx:] - ts[idx]) * 10**-9
            curve.setData(t, val[idx:], pen=pen, brush=brush, fillLevel=0)
        return
    
    def plot_curve1(self, curve1, queue, x, y, pen, brush):
        if queue is not None:
            ts, val = split(queue, [x, y])
            ts = np.asarray(ts, np.uint64)
            val = np.asarray(val, np.float32)
            idx = find_nearest(ts, ts[-1] - 20 * 10**9)
            t = (ts[idx:] - ts[idx]) * 10**-9
            curve1.setData(t, val[idx:], pen=pen, brush=brush, fillLevel=0)
        return

class TabWidget(QTabWidget):
    def __init__(self, ros_bridge):
        QTabWidget.__init__(self)
        
        self.plot_widget_0 = GenPlotWidget(ros_bridge, 0, 0)
        self.plot_widget_1 = GenPlotWidget(ros_bridge, 0, 1)
        self.plot_widget_2 = GenPlotWidget(ros_bridge, 0, 2)
        self.vision_widget = VisionWidget(ros_bridge)
        self.mission_telem_widget = MissionTelemWidget(ros_bridge)

        self.tab_one = QFrame()
        self.tab_one_layout = QGridLayout(self.tab_one)
        self.tab_one_layout.addWidget(self.plot_widget_0, 0, 0)
        self.tab_one_layout.addWidget(self.plot_widget_1, 1, 0)
        self.tab_one_layout.addWidget(self.plot_widget_2, 2, 0)
        
        self.tab_two = QFrame()
        self.tab_two_layout = QGridLayout(self.tab_two)
        self.tab_two_layout.addWidget(self.vision_widget, 0, 0)
        self.tab_two_layout.addWidget(self.mission_telem_widget, 1, 0)

        self.addTab(self.tab_one, '水平速度竖直速度偏航角')
        self.addTab(self.tab_two, '目标相对位置')

        self.tab_timer = QTimer()
        self.tab_timer.start(300)
        self.tab_timer.timeout.connect(self.update)
    def update(self):
        self.currentWidget().update()
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
            
            yaw = attitude[-1]
            arrow = self.calc_arrow(x=-n[-1], y=e[-1], yaw=yaw * math.pi / 180)
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
        self.height_label = QLabel("高度[m]:")
        self.acceleration_x_label = QLabel("E[m/s^2]:")
        self.acceleration_y_label = QLabel("N[m/s^2]:")
        self.acceleration_z_label = QLabel("U[m/s^2]:")
        self.velocity_x_label = QLabel("E[m/s]:")
        self.velocity_y_label = QLabel("N[m/s]:")
        self.velocity_z_label = QLabel("U[m/s]:")
        self.gps_position_latitude_label = QLabel("纬度:")
        self.gps_position_longitude_label = QLabel("经度:")
        #self.gps_position_altitude_label = QLabel("高度:")
        self.angular_velocity_x_label = QLabel("E[deg/s]:")
        self.angular_velocity_y_label = QLabel("N[deg/s]:")
        self.angular_velocity_z_label = QLabel("U[deg/s]:")
        self.attitude_x_label = QLabel("滚转[deg]:")
        self.attitude_y_label = QLabel("俯仰[deg]:")
        self.attitude_z_label = QLabel("偏航[deg]:")
        #self.attitude_w_label = QLabel("四元数w:")
        self.gps_health_label = QLabel("GPS强度:")
        self.battery_state_voltage_label = QLabel("电压[V]:")
        self.battery_state_current_label = QLabel("电流[A]:")
        self.battery_state_percentage_label = QLabel("百分比[%]:")
        self.flight_status_label = QLabel("飞行模式:")
        self.rc_axes0_label = QLabel("滚动通道:")
        self.rc_axes1_label = QLabel("俯仰通道:")
        self.rc_axes2_label = QLabel("偏航通道:")
        self.rc_axes3_label = QLabel("推力通道:")
        self.rc_axes4_label = QLabel("模式开关:")
        self.rc_axes5_label = QLabel("支撑架:")
 
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

        self.layout.addWidget(self.local_position_x_label, 1, 0, 1, 1)
        self.layout.addWidget(self.local_position_y_label, 1, 1, 1, 1)
        self.layout.addWidget(self.local_position_z_label, 1, 2, 1, 1)

        self.layout.addWidget(self.height_label, 0, 2, 1, 1)

        self.layout.addWidget(self.acceleration_x_label, 3, 0, 1, 1)
        self.layout.addWidget(self.acceleration_y_label, 3, 1, 1, 1)
        self.layout.addWidget(self.acceleration_z_label, 3, 2, 1, 1)
        self.layout.addWidget(self.velocity_x_label, 2, 0, 1, 1)
        self.layout.addWidget(self.velocity_y_label, 2, 1, 1, 1)
        self.layout.addWidget(self.velocity_z_label, 2, 2, 1, 1)
        self.layout.addWidget(self.gps_position_latitude_label, 0, 0, 1, 1)
        self.layout.addWidget(self.gps_position_longitude_label, 0, 1, 1, 1)
        #self.layout.addWidget(self.gps_position_altitude_label, 3, 2, 1, 1)
        self.layout.addWidget(self.angular_velocity_x_label, 5, 0, 1, 1)
        self.layout.addWidget(self.angular_velocity_y_label, 5, 1, 1, 1)
        self.layout.addWidget(self.angular_velocity_z_label, 5, 2, 1, 1)
        self.layout.addWidget(self.attitude_x_label, 4, 0, 1, 1)
        self.layout.addWidget(self.attitude_y_label, 4, 1, 1, 1)
        self.layout.addWidget(self.attitude_z_label, 4, 2, 1, 1)
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
            self.height_label.setText("高度[m]: {: 2.1f}".format(height))

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
            self.angular_velocity_x_label.setText("E[deg/s]: {: 2.1f}".format(angular_velocity[1]))
            self.angular_velocity_y_label.setText("N[deg/s]: {: 2.1f}".format(angular_velocity[2]))
            self.angular_velocity_z_label.setText("U[deg/s]: {: 2.1f}".format(angular_velocity[3]))

        attitude = self.ros_bridge.attitude_queue.read()
        if attitude is not None:
            yaw = attitude[-1]
            pitch = attitude[-2]
            roll = attitude[-3]
            self.attitude_x_label.setText("滚转[deg]: {: 2.1f}".format(roll))
            self.attitude_y_label.setText("俯仰[deg]: {: 2.1f}".format(pitch))
            self.attitude_z_label.setText("偏航[deg]: {: 2.1f}".format(yaw))
            #self.attitude_w_label.setText("欧拉角w: {: 2.3f}".format(attitude[4]))

        gps_health = self.ros_bridge.gps_health_queue.read()
        if gps_health is not None:
            self.gps_health_label.setText("GPS强度: {}".format(gps_health))
        
        battery_state = self.ros_bridge.battery_state_queue.read()
        if battery_state is not None:
            self.battery_state_voltage_label.setText("电压[V]: {: 2.1f}".format(battery_state[1]))
            self.battery_state_current_label.setText("电流[A]: {: 2.1f}".format(battery_state[2]))
            self.battery_state_percentage_label.setText("百分比[%]: {: 2.1f}".format(battery_state[3]))

        flight_status = self.ros_bridge.flight_status_queue.read()
        if flight_status is not None:
            self.flight_status_label.setText("飞行模式: {: 2.1f}".format(flight_status))

        rc = self.ros_bridge.rc_queue.read()
        if rc is not None:
            self.rc_axes0_label.setText("滚动通道: {: 2.1f}".format(rc[1]))
            self.rc_axes1_label.setText("俯仰通道: {: 2.1f}".format(rc[2]))
            self.rc_axes2_label.setText("偏航通道: {: 2.1f}".format(rc[3]))
            self.rc_axes3_label.setText("推力通道: {: 2.1f}".format(rc[4]))
            self.rc_axes4_label.setText("模式开关: {: 2.1f}".format(rc[5]))
            self.rc_axes5_label.setText("支撑架: {: 2.1f}".format(rc[6]))
        return

class ToggleButton(QPushButton):
    def __init__(self, text, _self):
        QPushButton.__init__(self, text[0], _self)
        self.text = text
        self.state = 0
        self.flag =0