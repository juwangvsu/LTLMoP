#!/usr/bin/env python
import rospy, math, subprocess, os, sys
from gazebo_msgs import srv
from geometry_msgs.msg import Twist
"""
==================================================================
rosLocomotionCommand.py - ros Locomotion Command Handler
==================================================================
"""

import lib.handlers.handlerTemplates as handlerTemplates

class RosLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
	def __init__(self, executor, shared_data, velocityTopic='/cmd_vel_mux/input/navi'):
		"""
		The ROS Locomotion Command Handler

		velocityTopic (str): This is the topic which handles the movement commands (default='/cmd_vel_mux/input/navi')
		"""
		self.model_name = "mobile_base"
		self.relative_entity_name = 'world'
		try:
			#open a publisher for the base controller of the robot
			# TODO: queue size okay?
			self.pub = rospy.Publisher(velocityTopic, Twist, queue_size=10)
			# for the pr2, use /base_controller/command
			# the turtlebot takes /cmd_vel
		except:
			print >>sys.__stdout__, 'Problem setting up Locomotion Command Node'
        def getPose(self,cached = False):
                if (not cached) or self.last_pose is None:
                        #Ros service call to get model state
                        #This returns a GetModelStateResponse, which contains data on pose
                        rospy.wait_for_service('/gazebo/get_model_state')
                        try:
                                gms = rospy.ServiceProxy('/gazebo/get_model_state', srv.GetModelState)
                                resp = gms(self.model_name,self.relative_entity_name)
                                #Cartesian Pose
                                self.pos_x = resp.pose.position.x
                                self.pos_y = resp.pose.position.y
                                self.pos_z = resp.pose.position.z
                                #Quaternion Orientation
                                self.or_x = resp.pose.orientation.x
                                self.or_y = resp.pose.orientation.y
                                self.or_z = resp.pose.orientation.z
                                self.or_w = resp.pose.orientation.w
                        except rospy.ServiceException, e:
                                print "Service call failed: %s"%e
			print([self.pos_x, self.pos_y])
	def sendCommand(self, cmd):

		#Twist is the message type and consists of x,y,z linear velocities
		#and roll, pitch, yaw orientation velocities (x,y,z)
		twist=Twist()
		#Positive x is forward on robots in Gazebo
		twist.linear.x=cmd[0]*4
		#Positive z is upward on robots in Gazebo
		twist.linear.z=cmd[2]*4
		#Angluar z is yaw or rotation in the xy plane
		twist.angular.z=cmd[1]*1.5
		try:
			#Publish the command to the robot
			self.pub.publish(twist)
		except:
			print 'Error publishing Twist Command'

if __name__ == '__main__':
    rospy.init_node('twist_to_mavcmd', anonymous=True)
    try:
        loch=RosLocomotionCommandHandler(1,1,'/mobile_base/commands/velocity')
    except rospy.ROSInterruptException:
        pass
    rate = rospy.Rate(100) # 10hz
    mycount=0
    while not rospy.is_shutdown():
	loch.sendCommand([0,1,0])
	loch.getPose()
	#print "..."
	#rate.sleep()
