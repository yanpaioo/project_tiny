#!/usr/bin/env python
import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Twist
from sensor_msgs.msg import LaserScan



class environment:
	frequency = 1
	done = False
	observation = None
	reward = None

	red_ugv_health = 100
	red_ugv_odom = Odometry()

	blue_ugv_health =100
	blue_ugv_odom = Odometry()

	def __init__(self):
		rospy.init_node('environment', anonymous=True)
		rospy.Subscriber("/red_ugv/odom", Odometry, self.redOdomCB)
		rospy.Subscriber("/blue_ugv/odom", Odometry, self.blueOdomCB)


	def redOdomCB(self, data):
		self.red_ugv_odom = data


	def blueOdomCB(self, data):
		self.blue_ugv_odom = data


	def step(self, red_action, blue_action):
		if red_action:
			pass

		if blue_action:
			pass

		return self.observation, self.reward, self.done

	def check_red_shoot(self):
		pass

	def check_blue_shoot(self):
		pass
	

myenv = environment()
observation, reward, done = myenv.step(0, 0)

rate = rospy.Rate(2)
while(1):

	print(observation, reward, done)
	rate.sleep()
	