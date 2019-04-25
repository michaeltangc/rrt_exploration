#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray
from time import time
import numpy as np
import os
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount
from numpy.linalg import norm
import datetime

frontiers_topic = ""
simulation_summary_string = ""
start_time = None
start_date = None
finished = False
finished_map = False
map_grid_count = 0
busy_robot_count = 1
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]
# frontiers_topic = ""
def callBack(data):
    global frontiers
    # global start_time, finished, busy_robot_count
    frontiers=[]
    for point in data.points:
        frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData, finished_map, start_date, start_time, map_grid_count, frontiers_topic, simulation_summary_string
    mapData = data
    if map_grid_count > 0 and not start_time is None:
        t = rospy.get_time()
        try:
            finished_grid_count = np.count_nonzero(np.where(np.array(data.data, dtype=np.int8) == -1, False, True))
            path = "/home/michael/ros_log/" + start_date + "_" + simulation_summary_string + ".txt"
            append_write = 'a'
            if os.path.exists(path):
                append_write = 'a' # append if already exists
            else:
                append_write = 'w' # make a new file if not
            with open(path, append_write) as f:
                f.write(str(t) + "," + str(t - start_time) + "," + str(finished_grid_count) + '\n')
        except Exception as e:
            print(e)
    # 
# Node----------------------------------------------

def node():
    global start_time, start_date, finished, busy_robot_count, map_grid_count, frontiers_topic, simulation_summary_string
    global frontiers,mapData,global1,global2,global3,globalmaps
    # global frontiers_topic
    rospy.init_node('assigner', anonymous=False)
    
    # fetching all parameters
    map_topic= rospy.get_param('~map_topic','/map')
    info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_multiplier=rospy.get_param('~info_multiplier',3.0)		
    hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
    hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
    frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
    n_robots = rospy.get_param('~n_robots',1)
    namespace = rospy.get_param('~namespace','')
    namespace_init_count = rospy.get_param('~namespace_init_count',1)
    delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
    rateHz = rospy.get_param('~rate',100)
    task_reset_time_out_s = rospy.get_param('~task_reset_time_out', 4)
    map_grid_count=rospy.get_param('~map_grid_count',1)
    simulation_summary_string=rospy.get_param('~simulation_summary_string',"")
    
    rate = rospy.Rate(rateHz)
#-------------------------------------------
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)
#---------------------------------------------------------------------------------------------------------------
        
# wait if no frontier is received yet 
    while len(frontiers)<1:
        pass
    centroids=copy(frontiers)	
#wait if map is not received yet
    while (len(mapData.data)<1):
        pass

    robots=[]
    if len(namespace)>0:
        for i in range(0,n_robots):
            robots.append(robot(namespace+str(i+namespace_init_count)))
    elif len(namespace)==0:
            robots.append(robot(namespace))
    for i in range(0,n_robots):
        robots[i].sendGoal(robots[i].getPosition(), None)
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
    start_time = rospy.get_time()
    start_date = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    while not rospy.is_shutdown():
        centroids=copy(frontiers)		
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
        infoGain=[]
        for ip in range(0,len(centroids)):
            infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots
        na=[] #available robots
        nb=[] #busy robots
        for i in range(0,n_robots):
            robots[i].cancelIfOverTime(rospy.get_time(), task_reset_time_out_s)
            if (robots[i].getState()==1):
                nb.append(i)
            else:
                na.append(i)	
        rospy.loginfo("available robots: "+str(na))	
        busy_robot_count = len(nb)
        if len(centroids) == 0:
            for r in na:
                robots[i].sendGoal(array([0.0, 0.0]), rospy.get_time())
        # print("BBBBBBBBBB ", len(frontiers), busy_robot_count)
        if not finished and len(frontiers) < 1 and busy_robot_count == 0:
            finish_time = rospy.get_time()
            # print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
            print("Start time", start_time, "Finish time", finish_time)
            try:
                with open('/home/michael/ros_log/log.txt', 'a+') as f:
                    f.write(simulation_summary_string + ",Topic," + frontiers_topic + ",Stamp," + start_date + ",Start_time,"+ str(start_time) + ",Finish_time," + str(finish_time) + ",Diff," + str(finish_time - start_time) + '\n')
                # print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
                finished = True
            except Exception as e:
                print(e)
#------------------------------------------------------------------------- 
#get dicount and update informationGain
        for i in nb+na:
            infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------            
        revenue_record=[]
        centroid_record=[]
        id_record=[]
        
        for ir in na:
            for ip in range(0,len(centroids)):
                cost=norm(robots[ir].getPosition()-centroids[ip])		
                threshold=1
                information_gain=infoGain[ip]
                if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):

                    information_gain*=hysteresis_gain
                revenue=information_gain*info_multiplier-cost
                revenue_record.append(revenue)
                centroid_record.append(centroids[ip])
                id_record.append(ir)
        
        if len(na)<1:
            revenue_record=[]
            centroid_record=[]
            id_record=[]
            for ir in nb:
                for ip in range(0,len(centroids)):
                    cost=norm(robots[ir].getPosition()-centroids[ip])		
                    threshold=1
                    information_gain=infoGain[ip]
                    if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
                        information_gain*=hysteresis_gain
                
                    if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
                        information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

                    revenue=information_gain*info_multiplier-cost
                    revenue_record.append(revenue)
                    centroid_record.append(centroids[ip])
                    id_record.append(ir)
        
        rospy.loginfo("revenue record: "+str(revenue_record))	
        rospy.loginfo("centroid record: "+str(centroid_record))	
        rospy.loginfo("robot IDs record: "+str(id_record))	
        
#-------------------------------------------------------------------------	
        if (len(id_record)>0):
            winner_id=revenue_record.index(max(revenue_record))
            robots[id_record[winner_id]].sendGoal(centroid_record[winner_id], rospy.get_time())
            rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))	
            rospy.sleep(delay_after_assignement)
#------------------------------------------------------------------------- 
        rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
