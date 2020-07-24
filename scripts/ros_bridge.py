#!/usr/bin/env python2.7

import sys; sys.dont_write_bytecode = True

from collections import deque
import copy
from threading import Condition
import rospy
from geometry_msgs.msg import PointStamped

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
        rospy.Subscriber('/dji_sdk/local_position', PointStamped, callback=self.loc_pos_cb, queue_size=1)
    def loc_pos_cb(self, pos):
        # TODO: stamp synchronize
        stamp = 0.0
        self.local_position_queue.append((stamp, pos.point.x, pos.point.y, pos.point.z))
        return