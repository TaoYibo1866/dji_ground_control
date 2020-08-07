#!/usr/bin/env python2.7

import sys; sys.dont_write_bytecode = True

from collections import deque
import copy
from threading import Event
import rospy
from geometry_msgs.msg import PointStamped, QuaternionStamped, Vector3Stamped
from std_msgs.msg import Float32, UInt8
from sensor_msgs.msg import NavSatFix, BatteryState, Joy 
from tf.transformations import euler_from_quaternion, rotation_matrix
import math

class Queue:
    def __init__(self, queue_size=1):
        self.data = deque(maxlen=queue_size) # this deque is already thread-safe
        self.update_event = Event()
    def append(self, item):
        self.data.append(item)
        self.update_event.set()
    def wait(self, timeout=0.1):
        self.update_event.clear()
        return self.update_event.wait(timeout)
    def read(self):
        if not self.data:
            return None
        return self.data[-1]
    def clear(self):
        self.data.clear()
    def copy(self):
        if not self.data:
            return None
        return copy.deepcopy(self.data)

class RosBridge:
    def __init__(self):
        self.local_position_queue = Queue(queue_size=150) # 50Hz
        self.attitude_queue = Queue(queue_size=2200) # 100Hz
        self.velocity_queue = Queue(queue_size=1200) # 50Hz

        self.height_queue = Queue(queue_size=150)
        self.acceleration_queue = Queue(queue_size=150)
        
        self.gps_position_queue = Queue(queue_size=150)
        self.angular_velocity_queue = Queue(queue_size=150)
        
        self.gps_health_queue = Queue(queue_size=150)
        self.battery_state_queue = Queue(queue_size=150)
        self.flight_status_queue = Queue(queue_size=150)
        self.rc_queue = Queue(queue_size=150)

        rospy.Subscriber('/dji_sdk/local_position', PointStamped, callback=self.loc_pos_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, callback=self.att_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/velocity', Vector3Stamped, callback=self.vel_cb, queue_size=1)

        rospy.Subscriber('/dji_sdk/height_above_takeoff', Float32, callback=self.height_cb, queue_size=1) 
        rospy.Subscriber('/dji_sdk/acceleration_ground_fused', Vector3Stamped, callback=self.acceleration_cb, queue_size=1)

        rospy.Subscriber('/dji_sdk/gps_position', NavSatFix, callback=self.gps_pos_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/angular_velocity_fused', Vector3Stamped, callback=self.angular_velocity_cb, queue_size=1)

        rospy.Subscriber('/dji_sdk/gps_health', UInt8, callback=self.gps_health_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/battery_state', BatteryState, callback=self.battery_state_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/flight_status', UInt8, callback=self.flight_status_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/rc', Joy, callback=self.rc_cb, queue_size=1)



    def loc_pos_cb(self, pos):
        self.local_position_queue.append((pos.header.stamp.to_nsec(), pos.point.x, pos.point.y, pos.point.z))
        return
    def att_cb(self, att):
        q_i2b=[att.quaternion.x, att.quaternion.y, att.quaternion.z, att.quaternion.w]
        y, p, r = euler_from_quaternion(q_i2b, axes='rzyx')
        r_=r*180/math.pi
        p_=p*180/math.pi
        y_=y*180/math.pi
        self.attitude_queue.append((att.header.stamp.to_nsec(), att.quaternion.x, att.quaternion.y, att.quaternion.z, att.quaternion.w, r_, p_, y_))
        return
    def vel_cb(self, vel):
        self.velocity_queue.append((vel.header.stamp.to_nsec(), vel.vector.x, vel.vector.y, vel.vector.z))
        return
          

    def height_cb(self, height):
        self.height_queue.append((height.data))
        return
    def acceleration_cb(self, acceleration):
        # TODO: stamp synchronize
        stamp = 0.0
        self.acceleration_queue.append((stamp, acceleration.vector.x, acceleration.vector.y, acceleration.vector.z))
        return

    def gps_pos_cb(self, gps_pos):
        # TODO: stamp synchronize
        stamp = 0.0
        self.gps_position_queue.append((stamp, gps_pos.latitude, gps_pos.longitude, gps_pos.altitude))
        return
    def angular_velocity_cb(self, angular_velocity):
        stamp = 0
        # TODO: stamp synchronize
        self.angular_velocity_queue.append((stamp, angular_velocity.vector.x, angular_velocity.vector.y, angular_velocity.vector.z))
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