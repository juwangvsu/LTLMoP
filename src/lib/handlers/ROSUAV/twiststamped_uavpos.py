#!/usr/bin/env python
# 6/20/19 derived from twiststamped_pos.py for ltlmop ros handler for UAV movement cmd

# translate twist cmd to /maos/setpoint_position/local instead of vel.
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy 
import time
from mavros_msgs.srv import *
import sys
global localpose

# rosservice call /mavros/set_mode '{custom_mode: OFFBOARD}'
# rosservice call /mavros/set_mode '{base_mode: 29, custom_mode: AUTO.LOITER}'
#acceptable mode: MANUAL POSCTL OFFBOARD STABILIZED AUTO.MISSION AUTO.LOITER AUTO.RTL AUTO.LAND AUTO.TAKEOFF, note when ardupilotmega, modes are diff.
def setoffmode():
	print("call set mode to offboard");
	print >>sys.__stdout__, 'setoffmode '
	setmodecall = rospy.ServiceProxy('/mavros/set_mode', SetMode)
	resp1 = setmodecall(0,'OFFBOARD')

def rawjoycallback(joydata):
	global localpose
#	print(joydata.buttons[5])
	if joydata.buttons[5]==1:
		print("call set mode to offboard");
		setmodecall = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		resp1 = setmodecall(0,'OFFBOARD')
	if joydata.buttons[7]==1:
		print("call set mode to offboard");
		setmodecall = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		resp1 = setmodecall(0,'ALTCTL')

def localposecallback(currentpose):
	global localpose
	localpose=currentpose
#	print(localpose)

# publish at least a few pose command seems needed to make offboard mode
# smooth. if only pub one msg, offboard mode demonstrate crashing behavior.

def cmdcallback(cmdVelocity):
#input Twist
#rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped {'header: {stamp: now}, pose: {position: {x: 10.0, y: 0.0, z: 8.0}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 10}}'} -r 10
	print("joy_twist msg rcved, press right trigger to enter offboard")
#rosservice call /mavros/set_mode '{custom_mode: OFFBOARD}'
#	s = rospy.Service('/mavros/setmode', SetMode, handle_setmode)
	targetPose = PoseStamped()
        targetPose.pose.position.x  =localpose.pose.position.x 
        targetPose.pose.position.y  =localpose.pose.position.y 
        targetPose.pose.position.z  =localpose.pose.position.z 

        targetPose.pose.position.x = targetPose.pose.position.x + cmdVelocity.linear.x
        targetPose.pose.position.y = targetPose.pose.position.y + cmdVelocity.linear.y
        targetPose.pose.position.z = targetPose.pose.position.z + cmdVelocity.linear.z*2


        now = rospy.get_rostime()
        targetPose.header.stamp.secs = now.secs
        targetPose.header.stamp.nsecs = now.nsecs
	print("velocity: "+str(cmdVelocity.linear.x) +" " +str(cmdVelocity.linear.y)+"\n")
	print("current pose\n"+str(localpose.pose.position)+" target pose \n"+str(targetPose.pose.position))
        targetPosePub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        targetPosePub.publish(targetPose)
	print >>sys.__stdout__, 'send twist to /mavros/setpoint_position/local '

# publish a few more to make offboard stable and avoid eradic behavior.
	rate = rospy.Rate(5) # 10hz
	rate.sleep()
        targetPosePub.publish(targetPose)
	rate.sleep()
        targetPosePub.publish(targetPose)


def cmd_vel_listener():
	global localpose
	localpose= PoseStamped()

#        rospy.Subscriber("/input_joy/cmd_vel", Twist, cmdcallback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, localposecallback)
#geometry_msgs/PoseStamped ostopic info /mavros2/local_position/pose
#rostopic info /mavros2/global_position/local  Type: nav_msgs/Odometry
# rostopic info mavros2/local_position/pose Type: geometry_msgs/PoseStamped
# rostopic info mavros/local_position/odom Type: nav_msgs/Odometry

#        rospy.spin()

if __name__ == '__main__':
	global localpose
	localpose= PoseStamped()
        rospy.init_node('cmd_vel_listener', anonymous=True)
        cmd_vel_listener()
        rospy.spin()
