#!/usr/bin/env python
"""
=================================================
rosDrive.py - Differential Drive Handler
=================================================

6/20/2019 jw: modified to drive px4 uav (gazebo), first check if it is takeoff and armed,
		if yes then put into offboard mode and send setlocalposition message.

Converts a desired global velocity vector into translational and rotational rates for a differential-drive robot,
using feedback linearization.
"""
import rospy
from math import sin, cos
import sys
from mavros_msgs.msg import State
import lib.handlers.handlerTemplates as handlerTemplates
from mavros_msgs.srv import *
class RosDriveHandler(handlerTemplates.DriveHandler):
    def __init__(self, executor, shared_data,d=0.6):
        """
        Initialization method of differential drive handler.

        d (float): Distance from front axle to point we are abstracting to [m] (default=0.6,max=0.8,min=0.2)
        """

	print >> sys.__stdout__, "wang rosdriverhandler.py subscribe to /mavros/state"
        try:
	    #print >> sys.__stdout__, "dbg info"
            self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
            self.coordmap = executor.hsub.coordmap_lab2map
        except NameError:
            print >>sys.__stdout__, "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

        self.d = d
	self.last_takeoffcmd_tm=0
	self.statetopic = '/mavros/state'
	rospy.Subscriber(self.statetopic, State, self.processData)
	print >> sys.__stdout__, "wang rosdriverhandler.py subscribe to /mavros/state"

    def processData(self, data):
	
	print >> sys.__stdout__, "wang rosdriverhandler.py state: armed, guided, mode"
	print >> sys.__stdout__, data.armed
	print >> sys.__stdout__, data.guided
	print >> sys.__stdout__, data.mode
 	if not data.armed == True or not data.guided == True or not data.mode == "AUTO.TAKEOFF":
		print >> sys.__stdout__, "need arm and takeoff" 
		armingcall = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		resp1 = armingcall(True)
		now = rospy.get_time()
		if (now - self.last_takeoffcmd_tm) >10 :
			setmodecall = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			resp1 = setmodecall(0,'AUTO.TAKEOFF')
			self.last_takeoffcmd_tm = now
			
#desired state /mavros/state	
#armed: True
#guided: True
#mode: "AUTO.TAKEOFF"


    def setVelocity(self, x, y, theta=0, z = 0):
        #print "VEL:%f,%f" % tuple(self.coordmap([x, y]))

        # Feedback linearization code:
        #d = 0.125 # Distance from front axle to point we are abstracting to [m]
        #d = 0.6 # Distance from front axle to point we are abstracting to [m]
        #vx = 0.09*X[0,0]
        #vy = 0.09*X[1,0]
        # ^^ Changed the scaling because it was getting stuck - too high of a velocity ? - Hadas 20/12/07
	y=-y    # 3/22/19 jw: this api is called from high level controller, which assume pixel coord, to apply to gazebo or real coord, y
		# axis must be inverted.
        vx = 0.29*x
        vy = 0.29*y
        vz = 0.29*z
	#print >> sys.__stdout__, "wang rosdriverhandler.py setVelocity()"
        w = (1/self.d)*(-sin(theta)*vx + cos(theta)*vy)
        v = cos(theta)*vx + sin(theta)*vy

        self.loco.sendCommand([v,w,vz])
if __name__ == '__main__':
        rospy.init_node('rosdrivehandler', anonymous=True)
        rdh = RosDriveHandler(1,2,3) 
	rospy.spin()
