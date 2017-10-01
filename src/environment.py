#!/usr/bin/env python
import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Twist
from sensor_msgs.msg import LaserScan
import tf
import numpy as np

frequency = 10	# hz
# laser_array_size = 128



class Odom:
	x = 0.
	y = 0.
	yaw = 0.

class environment:
	done = False
	observation = None
	reward = None

	max_linear_speed = 1.
	max_angular_speed = 1.

	red_ugv_health = 100
	red_ugv_odom = (0., 0., 0.)	# x, y, yaw
	red_ugv_laser = None

	blue_ugv_health = 100
	blue_ugv_odom = (0., 0., 0.)	# x, y, yaw
	blue_ugv_laser = None

	def __init__(self):
		rospy.init_node('environment', anonymous=True)
		rospy.Subscriber("/red_ugv/odom", Odometry, self.redOdomCB)
		rospy.Subscriber("/red_ugv/base_scan", LaserScan, self.redLaserCB)
		rospy.Subscriber("/blue_ugv/odom", Odometry, self.blueOdomCB)
		rospy.Subscriber("/blue_ugv/base_scan", LaserScan, self.blueLaserCB)

		self.Pub_red_ugv_cmdvel = rospy.Publisher('/red_ugv/cmd_vel', Twist, queue_size=1)
		self.Pub_blue_ugv_cmdvel = rospy.Publisher('/blue_ugv/cmd_vel', Twist, queue_size=1)
		


	def redOdomCB(self, data):
		# change from quaternion to euler, save x, y, yaw data

		my_pose = data
		quaternion = (
			my_pose.pose.pose.orientation.x,
			my_pose.pose.pose.orientation.y,
			my_pose.pose.pose.orientation.z,
			my_pose.pose.pose.orientation.w
			)

		euler = tf.transformations.euler_from_quaternion(quaternion)

		self.red_ugv_odom = (
			my_pose.pose.pose.position.x,
			my_pose.pose.pose.position.y,
			euler[2])

		# print("red_ugv_odom", self.red_ugv_odom)

	def redLaserCB(self, data):
		self.red_ugv_laser = data.ranges
		

	


	def blueOdomCB(self, data):
		# change from quaternion to euler, save x, y, yaw data

		my_pose = data
		quaternion = (
			my_pose.pose.pose.orientation.x,
			my_pose.pose.pose.orientation.y,
			my_pose.pose.pose.orientation.z,
			my_pose.pose.pose.orientation.w)

		euler = tf.transformations.euler_from_quaternion(quaternion)

		
		self.blue_ugv_odom = (
			my_pose.pose.pose.position.x,
			my_pose.pose.pose.position.y,
			euler[2])


	def blueLaserCB(self, data):
		self.blue_ugv_laser = data.ranges

	def red_cmdvel(self, action):
		cmdvel = Twist()

		# x
		if abs(action[0]) <= self.max_linear_speed:
			cmdvel.linear.x = action[0]
		else:
			cmdvel.linear.x = action[0]/abs(action[0])*self.max_linear_speed

		# y
		if abs(action[1]) <= self.max_linear_speed:
			cmdvel.linear.y = action[1]
		else:
			cmdvel.linear.y = action[1]/abs(action[1])*self.max_linear_speed

		# yaw
		if abs(action[2]) <= self.max_angular_speed:
			cmdvel.angular.z = action[2]
		else:
			cmdvel.angular.z = action[2]/abs(action[2])*self.max_angular_speed

		self.Pub_red_ugv_cmdvel.publish(cmdvel)
		

	def blue_cmdvel(self, action):
		cmdvel = Twist()

		# x
		if abs(action[0]) <= self.max_linear_speed:
			cmdvel.linear.x = action[0]
		else:
			cmdvel.linear.x = action[0]/abs(action[0])*self.max_linear_speed

		# y
		if abs(action[1]) <= self.max_linear_speed:
			cmdvel.linear.y = action[1]
		else:
			cmdvel.linear.y = action[1]/abs(action[1])*self.max_linear_speed

		# yaw
		if abs(action[2]) <= self.max_angular_speed:
			cmdvel.angular.z = action[2]
		else:
			cmdvel.angular.z = action[2]/abs(action[2])*self.max_angular_speed

		self.Pub_blue_ugv_cmdvel.publish(cmdvel)



	def step(self, red_action, blue_action):
		# process red_action
		dummy = (10., 10., 10.)
		self.red_cmdvel(dummy)
		dummy = (-10., -10., 10.)
		self.blue_cmdvel(dummy)

		# process blue_action

		return self.observation, self.reward, self.done

	def check_red_shoot(self):
		pass

	def check_blue_shoot(self):
		pass
	

myenv = environment()


rate = rospy.Rate(frequency)
while(1):
	observation, reward, done = myenv.step(0, 0)
	print(observation, reward, done)
	rate.sleep()
	