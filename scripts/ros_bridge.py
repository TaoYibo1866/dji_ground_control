#!/usr/bin/env python2.7

import sys; sys.dont_write_bytecode = True

from collections import deque
import copy
from threading import Condition
import rospy
from geometry_msgs.msg import PointStamped, Vector3Stamped, QuaternionStamped
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import NavSatFix, BatteryState, Joy 

class Queue:
    def __init__(self, queue_size=1):
        self.data = deque(maxlen=queue_size) # this deque is already thread-safe
        self.cond = Condition()
    def append(self, item):
        with self.cond:
            self.data.append(item)
            self.cond.notifyAll()
    def wait(self, timeout=0.1):
        with self.cond:
            self.cond.wait(timeout=timeout)
        return
    def read(self):
        if not self.data:
            return None
        return self.data[-1]
    def clear(self):
        self.data.clear()
    def copy(self):
        return copy.deepcopy(self.data)

class RosBridge:
    def __init__(self):
        
        self.local_position_queue = Queue(queue_size=150)
        self.height_queue = Queue(queue_size=150)
        self.acceleration_queue = Queue(queue_size=150)
        self.velocity_queue = Queue(queue_size=150)
        self.gps_position_queue = Queue(queue_size=150)
        self.angular_velocity_queue = Queue(queue_size=150)
        self.attitude_queue = Queue(queue_size=150)
        self.gps_health_queue = Queue(queue_size=150)
        self.battery_state_queue = Queue(queue_size=150)
        self.flight_status_queue = Queue(queue_size=150)
        self.rc_queue = Queue(queue_size=150)

        rospy.Subscriber('/dji_sdk/local_position', PointStamped, callback=self.loc_pos_cb, queue_size=1) 
        rospy.Subscriber('/dji_sdk/height_above_takeoff', Float32, callback=self.height_cb, queue_size=1) 
        rospy.Subscriber('/dji_sdk/acceleration_ground_fused', Vector3Stamped, callback=self.acceleration_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/velocity', Vector3Stamped, callback=self.velocity_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/gps_position', NavSatFix, callback=self.gps_pos_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/angular_velocity_fused', Vector3Stamped, callback=self.angular_velocity_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, callback=self.attitude_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/gps_health', UInt8, callback=self.gps_health_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/battery_state', BatteryState, callback=self.battery_state_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/flight_status', UInt8, callback=self.flight_status_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/rc', Joy, callback=self.rc_cb, queue_size=1)

    def loc_pos_cb(self, pos):
        # TODO: stamp synchronize
        stamp = 0.0
        self.local_position_queue.append((stamp, pos.point.x, pos.point.y, pos.point.z))
        return
    def height_cb(self, height):
        self.height_queue.append((height.data))
        return
    def acceleration_cb(self, acceleration):
        # TODO: stamp synchronize
        stamp = 0.0
        self.acceleration_queue.append((stamp, acceleration.vector.x, acceleration.vector.y, acceleration.vector.z))
        return
    def velocity_cb(self, velocity):
        # TODO: stamp synchronize
        stamp = 0.0
        self.velocity_queue.append((stamp, velocity.vector.x, velocity.vector.y, velocity.vector.z))
        return
    def gps_pos_cb(self, gps_pos):
        # TODO: stamp synchronize
        stamp = 0.0
        self.gps_position_queue.append((stamp, gps_pos.latitude, gps_pos.longitude, gps_pos.altitude))
        return
    def angular_velocity_cb(self, angular_velocity):
        # TODO: stamp synchronize
        stamp = 0.0
        self.angular_velocity_queue.append((stamp, angular_velocity.vector.x, angular_velocity.vector.y, angular_velocity.vector.z))
        return
    def attitude_cb(self, attitude):
        # TODO: stamp synchronize
        stamp = 0.0
        self.attitude_queue.append((stamp, attitude.quaternion.x, attitude.quaternion.y, attitude.quaternion.z, attitude.quaternion.w))
        return
    def gps_health_cb(self, gps_health):
        self.gps_health_queue.append((gps_health.data))
        return
    def battery_state_cb(self, battery_state):
        # TODO: stamp synchronize
        stamp = 0.0
        self.battery_state_queue.append((stamp, battery_state.voltage, battery_state.current, battery_state.percentage))
        return
    def flight_status_cb(self, flight_status):
        self.flight_status_queue.append((flight_status.data))
        return
    def rc_cb(self, rc):
        # TODO: stamp synchronize
        stamp = 0.0
        self.rc_queue.append((stamp, rc.axes[0], rc.axes[1],rc.axes[2],rc.axes[3],rc.axes[4],rc.axes[5]))
        return