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
from geometry_msgs.msg import PointStamped


def autostart():
    rospy.init_node('autostart')
    delay=rospy.get_param('~delay',5.0)
    pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=100)
    # ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);
    print("Autostart sleeping")
    rospy.sleep(delay)
    coors = [[-10.0, 10.0], [-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [1.0, 0.0]]
    print("Autostart Start")
    for i, coor in enumerate(coors):
      pt = PointStamped()
      pt.header.seq = i + 5000
      pt.header.stamp = rospy.Time.now()
      pt.header.frame_id = "autostart"
      pt.point.x = coor[0]
      pt.point.y = coor[1]
      pt.point.z = 0.0
      pub.publish(pt)
    print("Autostart finished")
    
    # rospy.Subscriber("/robot_1/map", OccupancyGrid, callback)
    return

if __name__ == '__main__':
    autostart()