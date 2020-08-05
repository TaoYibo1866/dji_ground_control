#!/usr/bin/env python2.7

import sys; sys.dont_write_bytecode = True

from collections import deque
import copy
from threading import Event
import rospy
from geometry_msgs.msg import PointStamped, QuaternionStamped, Vector3Stamped

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
        self.attitude_queue = Queue(queue_size=5) # 100Hz
        self.velocity_queue = Queue(queue_size=600) # 50Hz
        rospy.Subscriber('/dji_sdk/local_position', PointStamped, callback=self.loc_pos_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, callback=self.att_cb, queue_size=1)
        rospy.Subscriber('/dji_sdk/velocity', Vector3Stamped, callback=self.vel_cb, queue_size=1)
    def loc_pos_cb(self, pos):
        self.local_position_queue.append((pos.header.stamp.to_nsec(), pos.point.x, pos.point.y, pos.point.z))
        return
    def att_cb(self, att):
        self.attitude_queue.append((att.header.stamp.to_nsec(), att.quaternion.x, att.quaternion.y, att.quaternion.z, att.quaternion.w))
        return
    def vel_cb(self, vel):
        self.velocity_queue.append((vel.header.stamp.to_nsec(), vel.vector.x, vel.vector.y, vel.vector.z))
        return