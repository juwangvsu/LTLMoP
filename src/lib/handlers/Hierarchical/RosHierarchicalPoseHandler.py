#!/usr/bin/env python
"""
=======================================
rosPose.py - ROS Interface Pose Handler
=======================================
"""

import lib.handlers.handlerTemplates as handlerTemplates
import rospy
import time
from tf import TransformListener, ExtrapolationException
from tf.transformations import euler_from_quaternion
import logging
import globalConfig


class RosHierarchicalPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data, modelName="mobile_base"):
        """
        Pose Handler for ROS and gazebo.

        modelName (str): The model name of the robot in gazebo to get the pose information from (default="mobile_base")
        """

        #GetModelState expects the arguments model_name and relative_entity_name
        #In this case it is turtlebot and world respectively but can be changed for different robots and environments
        self.model_name = modelName
        self.relative_entity_name = 'world'  #implies the gazebo global coordinates
        self.last_pose = None

        try:
            rospy.init_node(
                'PoseHandler', anonymous=True, disable_signals=True)

            # we need to reload logging, because rospy.init_node changes it...
            root = logging.getLogger()
            map(root.removeHandler, root.handlers[:])
            map(root.removeFilter, root.filters[:])
            reload(globalConfig)
        except:
            logging.warning("Rospy node already initialized")
        self.tf = TransformListener()
        self.shared_data = shared_data['ROS_INIT_HANDLER']
        self.executor = executor
        executor.postEvent("INFO", "Pose initialized")

    def getPose(self, cached=False):
        success = False
        while not success:
            try:
                if self.tf.frameExists('/base_link') and self.tf.frameExists(
                        '/map'):
                    # t = self.tf.getLatestCommonTime('/base_link', '/map')
                    # t = rospy.Time.now()
                    t = rospy.Time(0)
                    position, quaternion = self.tf.lookupTransform(
                        '/map', '/base_link', t)

                    angles = euler_from_quaternion(quaternion)
                    theta = angles[2]

                    self.last_pose = [position[0], position[1], theta]
                    return self.last_pose
                else:
                    time.sleep(0.5)
            except ExtrapolationException as e:
                self.executor.postEvent(
                    "INFO", "Couldn't get pose, please wait %s" % (e))
                time.sleep(0.1)
