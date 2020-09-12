#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import sys; sys.dont_write_bytecode = True

from PyQt5.QtWidgets import QGridLayout, QVBoxLayout, QLabel, QFrame, QPushButton, QTabWidget, QRadioButton, QButtonGroup
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
#from mem_top import mem_top
import numpy as np
from tf.transformations import rotation_matrix
from mission_widgets import VisionWidget, MissionTelemWidget

# from mission_widgets import VisionWidget, MissionTelemWidget

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

class TabWidget(QTabWidget):
    def __init__(self, ros_bridge):
        QTabWidget.__init__(self)
        
        self.plot_widget_0 = GenPlotWidget(ros_bridge, (0, 0, (255,0,0,70)), (1, 1, (0,255,0,70)))
        self.plot_widget_1 = GenPlotWidget(ros_bridge, (0, 0, (255,0,0,70)), (1, 1, (0,255,0,70)))
        self.plot_widget_2 = GenPlotWidget(ros_bridge, (0, 0, (255,0,0,70)), (1, 1, (0,255,0,70)))
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

        self.addTab(self.tab_one, '通用')
        self.addTab(self.tab_two, '任务')

        self.tab_timer = QTimer()
        self.tab_timer.start(300)
        self.tab_timer.timeout.connect(self.update)
    def update(self):
        self.currentWidget().update()
        return

class CurveConfigWidget(QFrame):
    def __init__(self, parent, idx):
        QFrame.__init__(self)
        layout = QVBoxLayout(self)

        g0_rb0 = QRadioButton("位置E")
        g0_rb1 = QRadioButton("位置N")
        g0_rb2 = QRadioButton("位置U")
        g0_rb3 = QRadioButton("速度E")
        g0_rb4 = QRadioButton("速度N")
        g0_rb5 = QRadioButton("速度U")
        g0_rb6 = QRadioButton("Roll角")
        g0_rb7 = QRadioButton("Pitch角")
        g0_rb8 = QRadioButton("Yaw角")

        g1_rb0 = QRadioButton("red")
        g1_rb1 = QRadioButton("green")
        g1_rb2 = QRadioButton("blue")

        self.group0 = QButtonGroup(self)
        self.group0.addButton(g0_rb0, 0)
        self.group0.addButton(g0_rb1, 1)
        self.group0.addButton(g0_rb2, 2)
        self.group0.addButton(g0_rb3, 3)
        self.group0.addButton(g0_rb4, 4)
        self.group0.addButton(g0_rb5, 5)
        self.group0.addButton(g0_rb6, 6)
        self.group0.addButton(g0_rb7, 7)
        self.group0.addButton(g0_rb8, 8)

        self.group1 = QButtonGroup(self)
        self.group1.addButton(g1_rb0, 0)
        self.group1.addButton(g1_rb1, 1)
        self.group1.addButton(g1_rb2, 2)

        group0_layout = QGridLayout()
        group0_layout.addWidget(g0_rb0, 0, 0)
        group0_layout.addWidget(g0_rb1, 0, 1)
        group0_layout.addWidget(g0_rb2, 0, 2)
        group0_layout.addWidget(g0_rb3, 1, 0)
        group0_layout.addWidget(g0_rb4, 1, 1)
        group0_layout.addWidget(g0_rb5, 1, 2)
        group0_layout.addWidget(g0_rb6, 2, 0)
        group0_layout.addWidget(g0_rb7, 2, 1)
        group0_layout.addWidget(g0_rb8, 2, 2)

        group1_layout = QGridLayout()
        group1_layout.addWidget(g1_rb0, 0, 0)
        group1_layout.addWidget(g1_rb1, 0, 1)
        group1_layout.addWidget(g1_rb2, 0, 2)
    
        enter_button = QPushButton("确认")
        enter_button.clicked.connect(lambda: self.enter(parent, idx))
        enter_button.clicked.connect(self.close)

        layout.addLayout(group0_layout, 0)
        layout.addLayout(group1_layout, 1)
        layout.addWidget(enter_button, 2)
    def enter(self, parent, idx):
        if self.group0.checkedId() < 0 and self.group1.checkedId() < 0:
            pass
        else:
            xian = self.group0.checkedId()
            color = self.group1.checkedId()
            rgb = None
            if color == 0:
                rgb = (255,0,0,70)
            elif color == 1:
                rgb = (0,255,0,70)
            elif color == 2:
                rgb = (0,0,255,70)
            parent.curve_configs[idx] = (xian, color, rgb)    
            parent.curve_buttons[idx].setText(self.group0.checkedButton().text())
            parent.curve_buttons[idx].setStyleSheet("color: {};".format(self.group1.checkedButton().text()))  


class GenPlotWidget(QFrame):
    def __init__(self, ros_bridge, curve0_config=(0, 0, (255,0,0,70)), curve1_config=(1, 1, (0,255,0,70))):
        QFrame.__init__(self)
        self.ros_bridge = ros_bridge
        self.layout = QGridLayout(self)
        self.curve_config_widget = None

        #plot widget
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.curve0_button = QPushButton("曲线0")
        self.curve1_button = QPushButton("曲线1")
        self.stop_button = ToggleButton(["暂停", "绘制"], self)

        self.plot = self.plot_widget.addPlot(row=0, col=0)
        #self.plot.setXRange(0, 20)
        self.plot.showGrid(x=1, y=1)
        self.plot.showAxis('right')
        self.plot.showAxis('top')

        self.curve0 = pg.PlotCurveItem()
        self.curve1 = pg.PlotCurveItem()
        self.plot.addItem(self.curve0)
        self.plot.addItem(self.curve1)

        self.curve0_button.clicked.connect(lambda: self.config_curve(0))
        self.curve1_button.clicked.connect(lambda: self.config_curve(1))
        self.stop_button.clicked.connect(self.stop)

        self.layout.addWidget(self.plot_widget, 0, 0, 1, 3)
        self.layout.addWidget(self.curve0_button, 1, 0, 1, 1)
        self.layout.addWidget(self.curve1_button, 1, 1, 1, 1)
        self.layout.addWidget(self.stop_button, 1, 2, 1, 1)

        self.curves = [self.curve0, self.curve1]
        self.curve_buttons = [self.curve0_button, self.curve1_button]
        self.curve_configs = [curve0_config, curve1_config]
        

        self.timer = QTimer()
        self.timer.start(100)
        self.timer.timeout.connect(self.update)
    def update(self):
        num = 2
        flag = self.stop_button.flag
        plot_parameter = [[] for i in range(num)]
        if flag == 0:
            for i in range(num):
                kind = self.curve_configs[i][0]
                ts, val = self.switch_case(kind)
                plot_parameter[i] = (ts, val, self.curve_configs[i][1],self.curve_configs[i][-1])
            self.plot_curve(self.curves, plot_parameter)
        return
    
    def switch_case(self, kind):
        switcher = {
            0:split(self.ros_bridge.local_position_queue.copy(), [0, 1]),
            1:split(self.ros_bridge.local_position_queue.copy(), [0, 2]),
            2:split(self.ros_bridge.local_position_queue.copy(), [0, 3]),
            3:split(self.ros_bridge.velocity_queue.copy(), [0, 1]),
            4:split(self.ros_bridge.velocity_queue.copy(), [0, 2]),
            5:split(self.ros_bridge.velocity_queue.copy(), [0, 3]),
            6:split(self.ros_bridge.attitude_queue.copy(), [0, -3]),
            7:split(self.ros_bridge.attitude_queue.copy(), [0, -2]),
            8:split(self.ros_bridge.attitude_queue.copy(), [0, -1]),
        }
        return switcher[kind]

    def config_curve(self, idx):
        if self.curve_config_widget is None or not self.curve_config_widget.isVisible():
            self.curve_config_widget = CurveConfigWidget(self, idx)
            self.curve_config_widget.show()

    def plot_curve(self, curve, queue):
        if queue is not None: 
            ts_sta = np.asarray(queue[0][0], np.uint64)
            idx_sta = find_nearest(ts_sta, ts_sta[-1] - 20 * 10**9)
            for i in [0,1]:
                ts = np.asarray(queue[i][0], np.uint64)
                val = np.asarray(queue[i][1], np.float32)
                idx = find_nearest(ts, ts[-1] - 20 * 10**9)
                t = (ts[idx:] - ts[idx_sta]) * 10**-9
                pen = queue[i][2]
                if pen == 0: 
                    curve[i].setData(t, val[idx:], pen=pg.mkPen('r', width=2), brush=queue[i][3], fillLevel=0)
                elif pen == 1:
                    curve[i].setData(t, val[idx:], pen=pg.mkPen('g', width=2), brush=queue[i][3], fillLevel=0)
                elif pen == 2:
                    curve[i].setData(t, val[idx:], pen=pg.mkPen('b', width=2), brush=queue[i][3], fillLevel=0)
        return
    
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
            arrow = self.calc_arrow(x=-n[-1], y=e[-1], yaw=yaw * np.pi / 180)
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
        #self.gps_position_altitude_label = QLabel("Height:")
        self.angular_velocity_x_label = QLabel("E[deg/s]:")
        self.angular_velocity_y_label = QLabel("N[deg/s]:")
        self.angular_velocity_z_label = QLabel("U[deg/s]:")
        self.attitude_x_label = QLabel("Roll[deg]:")
        self.attitude_y_label = QLabel("Pitch[deg]:")
        self.attitude_z_label = QLabel("Yaw[deg]:")
        #self.attitude_w_label = QLabel("四元数w:")
        self.gps_health_label = QLabel("GPS Health:")
        self.battery_state_voltage_label = QLabel("电压[V]:")
        self.battery_state_current_label = QLabel("电流[A]:")
        self.battery_state_percentage_label = QLabel("Battery[%]:")
        self.flight_status_label = QLabel("Flight Status:")
        self.rc_axes0_label = QLabel("Roll通道:")
        self.rc_axes1_label = QLabel("Pitch通道:")
        self.rc_axes2_label = QLabel("Yaw通道:")
        self.rc_axes3_label = QLabel("推力通道:")
        self.rc_axes4_label = QLabel("Switch:")
        self.rc_axes5_label = QLabel("支撑架:")
 
        self.local_position_x_label.setFrameShape(QFrame.Box)
        self.local_position_y_label.setFrameShape(QFrame.Box)
        self.local_position_z_label.setFrameShape(QFrame.Box)
        self.height_label.setFrameShape(QFrame.Box)
        self.height_label.setStyleSheet("color: blue; font: bold;")
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
        self.attitude_z_label.setStyleSheet("color: blue; font: bold;")
        self.gps_health_label.setFrameShape(QFrame.Box)
        self.gps_health_label.setStyleSheet("color: red; font: bold;")
        self.battery_state_voltage_label.setFrameShape(QFrame.Box)
        self.battery_state_current_label.setFrameShape(QFrame.Box)
        self.battery_state_percentage_label.setFrameShape(QFrame.Box)
        self.battery_state_percentage_label.setStyleSheet("color: red; font: bold;")
        self.flight_status_label.setFrameShape(QFrame.Box)
        self.flight_status_label.setStyleSheet("color: blue; font: bold;")
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
            #self.gps_position_altitude_label.setText("Height: {: 2.1f}".format(gps_position[3]))
        
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
            self.attitude_x_label.setText("Roll[deg]: {: 2.1f}".format(roll))
            self.attitude_y_label.setText("Pitch[deg]: {: 2.1f}".format(pitch))
            self.attitude_z_label.setText("Yaw[deg]: {: 2.1f}".format(yaw))
            #self.attitude_w_label.setText("欧拉角w: {: 2.3f}".format(attitude[4]))

        gps_health = self.ros_bridge.gps_health_queue.read()
        if gps_health is not None:
            self.gps_health_label.setText("GPS Health: {}".format(gps_health))
        
        battery_state = self.ros_bridge.battery_state_queue.read()
        if battery_state is not None:
            self.battery_state_voltage_label.setText("电压[V]: {: 2.1f}".format(battery_state[1]))
            self.battery_state_current_label.setText("电流[A]: {: 2.1f}".format(battery_state[2]))
            self.battery_state_percentage_label.setText("Battery[%]: {: 2.1f}".format(battery_state[3]))

        flight_status = self.ros_bridge.flight_status_queue.read()
        if flight_status is not None:
            self.flight_status_label.setText("Flight Status: {: 2.1f}".format(flight_status))

        rc = self.ros_bridge.rc_queue.read()
        if rc is not None:
            self.rc_axes0_label.setText("Roll通道: {: 2.1f}".format(rc[1]))
            self.rc_axes1_label.setText("Pitch通道: {: 2.1f}".format(rc[2]))
            self.rc_axes2_label.setText("Yaw通道: {: 2.1f}".format(rc[3]))
            self.rc_axes3_label.setText("推力通道: {: 2.1f}".format(rc[4]))
            self.rc_axes4_label.setText("Switch: {: 2.1f}".format(rc[5]))
            self.rc_axes5_label.setText("支撑架: {: 2.1f}".format(rc[6]))
        return

class ToggleButton(QPushButton):
    def __init__(self, text, _self):
        QPushButton.__init__(self, text[0], _self)
        self.text = text
        self.state = 0
        self.flag = 0