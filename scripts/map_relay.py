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
# Position.
from tf import TransformListener
# Goal.
from geometry_msgs.msg import PoseStamped
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf.transformations import euler_from_quaternion

NS = rospy.get_namespace()

X = 0
Y = 1
YAW = 2


class MapRelay(object):
  def __init__(self, master_relay, name, map_topic_name, relay_map_topic_suffix, max_dist):
    self._name = name
    self._namespace = '/' + name + '/'

    self.subscribe_topic = self._namespace + map_topic_name
    rospy.Subscriber(self.subscribe_topic, OccupancyGrid, self.callback)
    self.publish_topic = self._namespace + relay_map_topic_suffix
    self.publisher = rospy.Publisher(self.publish_topic, OccupancyGrid, queue_size = 5)

    self.master_relay = master_relay
    self.max_dist = max_dist
    self._tf = TransformListener()
    self._occupancy_grid = None
    self._occupancy_grid_pub = None
    self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    
  def callback(self, msg):
    self.update_pose()
    self._occupancy_grid = msg
    # if self.master_relay is None:
    #   self._occupancy_grid_pub = msg
    # else:
    #   self.master_relay.update_pose()
    #   p1 = self.master_relay.pose
    #   # p2 = self.master_relay.get_pose(self._name)
    #   p2 = self.pose
    #   dist = np.linalg.norm([p1[X] - p2[X], p1[Y] - p2[Y]])
    #   print("Distance ", self._name, self.master_relay._name, dist)
    #   if dist <= self.max_dist:
    #     print("Map updated ", dist, self.max_dist)
    #     self._occupancy_grid = msg

  def publish_map(self):
    if self.master_relay is None:
      self._occupancy_grid_pub = self._occupancy_grid
    else:
      dist = self.get_distance(self.master_relay._name)
      if dist is not None and dist <= self.max_dist:
        # print("AAAAAAAAAAAAAAAA Map updated ", dist, self.max_dist)
        self._occupancy_grid_pub = self._occupancy_grid
      # self.master_relay.update_pose()
      # self.update_pose()
      # p1 = self.master_relay.pose
      # # p2 = self.master_relay.get_pose(self._name)
      # p2 = self.pose
      # if not np.isnan(p2[X]) and not np.isnan(p1[X]):
      #   dist = np.linalg.norm([p1[X] - p2[X], p1[Y] - p2[Y]])
      #   print("AAAAAAAAAAAAAAA Distance ", self.master_relay._name, self._name, p1, p2, dist)
      #   if dist <= self.max_dist:
      #     print("AAAAAAAAAAAAAAAA Map updated ", dist, self.max_dist)
      #     self._occupancy_grid_pub = self._occupancy_grid
      # else:
      #   print("AAAAAAAAAAAAAAAAA Nan ", self.master_relay._name, self._name, p1, p2)

    if self._occupancy_grid_pub is not None:
      self.publisher.publish(self._occupancy_grid_pub)
      # print("Published")
  
  def get_distance(self, robot_name):
    a = self._name + '/base_link'
    b = robot_name + '/base_link'
    dist = None
    try:
      t = rospy.Time(0)
      position, orientation = self._tf.lookupTransform(a, b, t)
      dist = np.linalg.norm([position[X], position[Y]])
      # print(position, dist)
    except Exception as e:
      print(e)
    return dist
    

  def update_pose(self):
    # Get pose w.r.t. map.
    a = self._name + '/map'
    b = self._name + '/base_link'
    if True: # self._tf.frameExists(a) and self._tf.frameExists(b):
      try:
        t = rospy.Time(0)
        position, orientation = self._tf.lookupTransform(a, b, t)
        self._pose[X] = position[X]
        self._pose[Y] = position[Y]
        _, _, self._pose[YAW] = euler_from_quaternion(orientation)
        # print('pose updated')
        # print(self._pose)
      except Exception as e:
        print(e)
        print('Unable to find:', a, self._tf.frameExists(a), b, self._tf.frameExists(b))
    else:
      print('Unable to find:', a, self._tf.frameExists(a), b, self._tf.frameExists(b))
    pass

  def get_pose(self, robot_name):
    # Get pose w.r.t. map.
    a = self._name + '/map'
    b = robot_name + '/base_link'
    if self._tf.frameExists(a) and self._tf.frameExists(b):
      try:
        t = rospy.Time(0)
        position, orientation = self._tf.lookupTransform(a, b, t)
        self._pose[X] = position[X]
        self._pose[Y] = position[Y]
        _, _, self._pose[YAW] = euler_from_quaternion(orientation)
        # print('pose updated')
        # print(self._pose)
      except Exception as e:
        print(e)
    else:
      print('Unable to find:', a, self._tf.frameExists(a), b, self._tf.frameExists(b))
    pass

  @property
  def ready(self):
    return self._occupancy_grid is not None # and not np.isnan(self._pose[0])

  @property
  def pose(self):
    return self._pose

  @property
  def occupancy_grid(self):
    return self._occupancy_grid

def run(args):
  rospy.init_node('map_relay', anonymous=False)

  master_robot_name = rospy.get_param('~master_robot_name', 'robot_1')
  other_robot_names = rospy.get_param('~other_robot_names', 'robot_2,robot_3')
  map_topic_name = rospy.get_param('~map_topic_name', 'map')
  relay_map_topic_suffix = rospy.get_param('~relay_map_topic_suffix', 'map_relay_robot_1')
  maximum_range = rospy.get_param('~maximum_range', 10)

  master_relay_ = MapRelay(None, master_robot_name, map_topic_name, relay_map_topic_suffix, maximum_range)
  map_relays = []
  for name in other_robot_names.split(','):
    map_relays.append(MapRelay(master_relay_, name, map_topic_name, relay_map_topic_suffix, maximum_range))

  # map_topic= rospy.get_param('~map_topic','/map')

  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  while not rospy.is_shutdown():
    master_relay_.publish_map()
    for relay in map_relays:
      relay.publish_map()
    rate_limiter.sleep()

  # map_publisher = rospy.Publisher(NS + 'map_test', OccupancyGrid, queue_size=2)


  # rospy.spin()


  # publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  # path_publisher = rospy.Publisher('/path', Path, queue_size=1)
  # slam = SLAM()
  # goal = GoalPose()

  # frame_id = 0

  # current_path = []
  # previous_time = rospy.Time.now().to_sec()

  # # Stop moving message.
  # stop_msg = Twist()
  # stop_msg.linear.x = 0.
  # stop_msg.angular.z = 0.

  # # Make sure the robot is stopped.
  # i = 0
  # while i < 10 and not rospy.is_shutdown():
  #   publisher.publish(stop_msg)
  #   rate_limiter.sleep()
  #   i += 1

  # while not rospy.is_shutdown():

    # slam.update()

    # current_time = rospy.Time.now().to_sec()

    # if not map_combiner.ready:
    #   rate_limiter.sleep()
    #   continue

    # new_msg = OccupancyGrid()
    # new_msg.header = map_1._occupancy_grid.header
    # new_msg.info = map_1._occupancy_grid.info
    # new_msg.data = np.array(map_1._occupancy_grid.data, dtype=np.int8)
    # new_msg.data = map_1._occupancy_grid.data
    # map_listeners[0].update_pose()
    # map_combiner.publish()
    # map_publisher.publish(new_msg)

    # combined_map_msg = OccupancyGrid()
    # combined_map_msg.
    
    # path_msg = Path()

    # path_msg.header.seq = frame_id
    # path_msg.header.stamp = rospy.Time.now()
    # path_msg.header.frame_id = 'map'
    # for u in current_path:
    #   pose_msg = PoseStamped()
    #   pose_msg.header.seq = frame_id
    #   pose_msg.header.stamp = path_msg.header.stamp
    #   pose_msg.header.frame_id = 'map'
    #   pose_msg.pose.position.x = u[X]
    #   pose_msg.pose.position.y = u[Y]
    #   path_msg.poses.append(pose_msg)
    # path_publisher.publish(path_msg)

    # Make sure all measurements are ready.
     # Get map and current position through SLAM:
    # > roslaunch exercises slam.launch

    # if not goal.ready or not slam.ready:
    #   rate_limiter.sleep()
    #   continue

    # goal_reached = np.linalg.norm(slam.pose[:2] - goal.position) < .2
    # if goal_reached:
    #   publisher.publish(stop_msg)
    #   rate_limiter.sleep()
    #   continue

    # # Follow path using feedback linearization.
    # position = np.array([
    #     slam.pose[X] + EPSILON * np.cos(slam.pose[YAW]),
    #     slam.pose[Y] + EPSILON * np.sin(slam.pose[YAW])], dtype=np.float32)
    # v = get_velocity(position, np.array(current_path, dtype=np.float32))
    # u, w = feedback_linearized(slam.pose, v, epsilon=EPSILON)
    # vel_msg = Twist()
    # vel_msg.linear.x = u
    # vel_msg.angular.z = w
    # publisher.publish(vel_msg)

    # # Update plan every 1s.
    # time_since = current_time - previous_time
    # if current_path and time_since < 2.:
    #   rate_limiter.sleep()
    #   continue
    # previous_time = current_time

    # # Run RRT.
    # start_node, final_node = rrt.rrt(slam.pose, goal.position, slam.occupancy_grid)
    # current_path = get_path(final_node)
    # if not current_path:
    #   print('Unable to reach goal position:', goal.position)
    
    # # Publish path to RViz.
    # path_msg = Path()
    # path_msg.header.seq = frame_id
    # path_msg.header.stamp = rospy.Time.now()
    # path_msg.header.frame_id = 'map'
    # for u in current_path:
    #   pose_msg = PoseStamped()
    #   pose_msg.header.seq = frame_id
    #   pose_msg.header.stamp = path_msg.header.stamp
    #   pose_msg.header.frame_id = 'map'
    #   pose_msg.pose.position.x = u[X]
    #   pose_msg.pose.position.y = u[Y]
    #   path_msg.poses.append(pose_msg)
    # path_publisher.publish(path_msg)

    # rate_limiter.sleep()
    # frame_id += 1


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Run map relay')
  args, unknown = parser.parse_known_args()
  try:
    run(args)
  except rospy.ROSInterruptException:
    pass