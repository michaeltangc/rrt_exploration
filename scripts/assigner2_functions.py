import rospy
import tf
from numpy import array
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped


class robot:
	assigned_point=[]
	goal = MoveBaseGoal()
	start = PoseStamped()
	end = PoseStamped()
	
	def __init__(self,name):
		self.name=name
		self.global_frame=rospy.get_param('~global_frame','/robot_1/map')
		self.listener=tf.TransformListener()
		self.listener.waitForTransform(self.global_frame, name+'/base_link', rospy.Time(0),rospy.Duration(10.0))
		cond=0;	
		while cond==0:	
			try:
				(trans,rot) = self.listener.lookupTransform(self.global_frame, self.name+'/base_link', rospy.Time(0))
				cond=1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				cond==0
		self.position=array([trans[0],trans[1]])		
		robot.assigned_point=self.position
		self.client=actionlib.SimpleActionClient(self.name+'/move_base', MoveBaseAction)
		self.client.wait_for_server()
		robot.goal.target_pose.header.frame_id=self.global_frame
		robot.goal.target_pose.header.stamp=rospy.Time.now()
		
		rospy.wait_for_service(self.name+'/move_base_node/NavfnROS/make_plan')
		self.make_plan = rospy.ServiceProxy(self.name+'/move_base_node/NavfnROS/make_plan', GetPlan)
		robot.start.header.frame_id=self.global_frame
		robot.end.header.frame_id=self.global_frame

	def getPosition(self):
		cond=0;	
		while cond==0:	
			try:
				(trans,rot) = self.listener.lookupTransform(self.global_frame, self.name+'/base_link', rospy.Time(0))
				cond=1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				cond==0
		self.position=array([trans[0],trans[1]])
		return self.position
		
	def sendGoal(self,point):
		robot.goal.target_pose.pose.position.x=point[0]
		robot.goal.target_pose.pose.position.y=point[1]
		robot.goal.target_pose.pose.orientation.w = 1.0
		self.client.send_goal(robot.goal)
		robot.assigned_point=array(point)
	
	def cancelGoal(self):
		self.client.cancel_goal()
		robot.assigned_point=self.getPosition()
	
	def getState(self):
		return self.client.get_state()
		
	def makePlan(self,start,end):
		robot.start.pose.position.x=start[0]
		robot.start.pose.position.y=start[1]
		robot.end.pose.position.x=end[0]
		robot.end.pose.position.y=end[1]
		start=self.listener.transformPose(self.name+'/map', robot.start)
		end=self.listener.transformPose(self.name+'/map', robot.end)
		plan=self.make_plan(start = start, goal = end, tolerance = 0.0)
		return plan.plan.poses
