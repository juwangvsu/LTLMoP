import lib.handlers.handlerTemplates as handlerTemplates
import rospy
import actionlib
import logging
import globalConfig
import sys
import threading

import matplotlib.path as mplPath

from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# Goals
# 0 Pending
# 1 Active
# 2 Preempted
# 3 Succeeded
# 4 Aborted
# 5 Rejected
# 6 Preempting
# 7 Recalling
# 8 Recalled
# 9 Lost


class MoveBaseMotionHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data):
        self.executor = executor
        self.pose_handler = executor.hsub.getHandlerInstanceByType(
            handlerTemplates.PoseHandler)
        try:
            # We can only run one node per script, so ignore if it fails
            rospy.init_node(
                'move_base_motion_handler',
                anonymous=True,
                disable_signals=True)
        except:
            logging.warning("Node already initialized")
        self.current_goal = None

        self.points = []  # array of points of the region
        self.failed = False

        # Initialize the client to send goals to
        self.move_base = actionlib.SimpleActionClient('move_base',
                                                      MoveBaseAction)
        self.move_base.wait_for_server()

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        current_reg and next_reg are indices of the regions
        Returns ``True`` iff we've reached the destination region
        """
        if current_reg == next_reg:
            return True

        # Get the Region object of the index
        next_reg = self.executor.proj.rfi.regions[next_reg]

        # Check if we think we're in the correct region already
        arrived = self.check_inside_region(self.pose_handler.getPose(),
                                           next_reg)
        if arrived:
            return True

        # We have failed last time, so try another point in the region
        if self.failed:
            self.failed = False

            # Pop the next point and try to go there
            if len(self.points) > 0:
                pt = self.points.pop()
                goal = self.create_goal(pt.x, pt.y)
                self.move_base.send_goal(goal)
            else:
                self.executor.post_event_hierarchical(
                    "INFO", "We have ultimately failed getting to %s" %
                    self.find_region_mapping(next_reg.name))
                sys.exit(-1)

        # Only do something if the next_region is different from our current goal
        elif self.current_goal != next_reg:
            self.current_goal = next_reg

            # TODO: how many points should we try?
            # self.points = list(next_reg.getPoints())[2:]
            self.points = []

            center = next_reg.getCenter()
            goal = self.create_goal(center.x, center.y)

            self.executor.postEvent("INFO", "Sending goal to move_base")
            self.move_base.send_goal(goal)

        # Check what's happening
        state = self.move_base.get_state()

        if state == GoalStatus.SUCCEEDED:
            self.executor.postEvent("INFO", "We've arrived successfully: ")
            self.current_goal = None

            return True
        elif state in [
                GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST
        ]:

            # TODO: Check if in the correct region anyway?
            # TODO: communicate failure up
            if len(self.points) == 0:
                self.executor.post_event_hierarchical(
                    "FAIL", self.find_region_mapping(next_reg.name))
                logging.warning("We've told our parents of our demise")
            else:
                self.executor.postEvent(
                    "INFO", "Couldn't get to the goal, trying another point")

            self.failed = True
            return False
        elif state != GoalStatus.ACTIVE and state != GoalStatus.PENDING:
            self.executor.postEvent("INFO", "Something else: " + str(state))

        return False

    def create_goal(self, x, y):
        """
        Take coordinates from LTLMoP, transform them to real corrdinates and
        create a goal
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'

        # Translate the coordinates of the region into real coordinates
        (x, y) = self.executor.hsub.coordmap_map2lab((x, y))

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        return goal

    def find_region_mapping(self, name):
        """
        Takes the decomposed region name and tries to find the original name
        """
        for rname, subregs in self.executor.proj.regionMapping.iteritems():
            if name in subregs:
                return rname
        logging.error("RegionMapping didn't have our region")
        return None

    def check_inside_region(self, pose, region):
        """
        Check if the current pose is inside a region
        """
        pts = list(
            map(self.executor.hsub.coordmap_map2lab, region.getPoints()))

        path = mplPath.Path(pts, closed=True)
        return path.contains_point((pose[0], pose[1]))
