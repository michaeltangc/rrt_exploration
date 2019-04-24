#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import os
import rospy
import sys

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist
# Occupancy grid.
from nav_msgs.msg import OccupancyGrid

def callback(data):
    # Here would be a place to do some sort of computation to decide if you want
    # the data republished on the new topic. I've inserted a dummy computation.:
    if data is not None:
      print(np.count_nonzero(np.where(np.array(data.data, dtype=np.int8) == -1, False, True)))
    return

def listener():
    rospy.init_node('map_counter')
    # pub = rospy.Publisher("pcd_points", PointCloud2, queue_size=1)
    rospy.Subscriber("/robot_1/map", OccupancyGrid, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()