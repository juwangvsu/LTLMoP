#!/usr/bin/env python
"""
=================================================
rosSim.py - ROS/Gazebo Initialization Handler
=================================================
"""

import lib.handlers.handlerTemplates as handlerTemplates


class RosHierarchicalInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor):
        """
        Do something
        """
        self.offset = [0, 0]

    def getSharedData(self):
        """
        Just in case we will need it at some point
        """
        return {'ROS_INIT_HANDLER': self}
