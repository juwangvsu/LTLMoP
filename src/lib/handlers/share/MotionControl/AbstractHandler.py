#!/usr/bin/env python
"""
===================================================================
abstractHandler.py
===================================================================

Uses the heat-controller to take a current position, current region, and destination region and return a global velocity vector that will help us get there
"""

from numpy import *
import time

import logging
import lib.handlers.handlerTemplates as handlerTemplates
from hierarchical import LocalGame, Hierarchical
import threading
from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib
import socket


class AbstractHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data, project_name, project_path,
                 initial_region):
        """
        project_name (string): Project name (default="Test")
        project_path (string): Absolute path to the project root folder (default="Test")
        initial_region (string): Initial region of the lowest level (default="11")
        """
        self.proj_name = project_name
        self.proj_path = project_path
        self.initial_region = initial_region

        self.pose_handler = executor.hsub.getHandlerInstanceByType(
            handlerTemplates.PoseHandler)
        self.fwd_coordmap = executor.hsub.coordmap_map2lab
        self.rfi = executor.proj.rfi
        self.executor = executor

        self.current_game = None
        self.arrived = False

        # This region is on level 0
        self.last_current_region = initial_region

        self.listen_port = None
        self.xmlrpc_server_thread = None
        self.setup_xmlrpc()

    def setup_xmlrpc(self):
        """
        Set up the xmlrpc server and store the port in the self.listen_port variable
        """
        while True:
            self.listen_port = random.randint(10000, 65535)
            try:
                serv = SimpleXMLRPCServer(
                    ("127.0.0.1", self.listen_port),
                    logRequests=False,
                    allow_none=True)
            except socket.error as e:
                pass
            else:
                break

        serv.register_function(self.handle_event)

        self.xmlrpc_server_thread = threading.Thread(target=serv.serve_forever)
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        If ``last`` is true, we will move to the center of the region.

        Returns ``True`` if we are outside the supposed ``current_reg``
        """
        current_reg = self.find_region_mapping(self.rfi.regions[current_reg]
                                               .name)
        next_reg = self.find_region_mapping(self.rfi.regions[next_reg].name)

        if self.arrived:
            # We seem to have arrived, so return true and stop the current game
            self.arrived = False
            if self.current_game is not None:
                self.stop_local_game()
            return True

        # We need to go somewhere else and a game is running
        if self.current_game is not None and (
                self.current_game.goal_region != exit_helper(current_reg,
                                                             next_reg)):
            self.last_current_region = self.current_game.current_region
            self.stop_local_game()

        # No game is running
        if not self.current_game:
            self.arrived = False
            if current_reg != next_reg:
                self.current_game = self.create_local_game(
                    current_reg, exit_helper(current_reg, next_reg),
                    self.last_current_region)
                current_thread = threading.Thread(target=self.current_game.run)
                current_thread.daemon = True
                current_thread.start()

        return self.arrived

    def handle_event(self, event_type, event_data):
        """
        Gets called if an event happened that is of interest
        """
        if event_type == "STATE":
            # If the state changes, stop the current game and create a new one
            (current_region, next_region) = event_data
            logging.info("STATE: {}".format(event_data))
        elif event_type == "BORDER":
            # TODO: trigger state evaluation/step
            logging.info("Borders crossed in MotionHandler to:{}".format(
                event_data))

            # Get the regions and swap them
            self.last_current_region = swap_exit(event_data)
            logging.info(
                "eventstuff: Setting the last current region to {}, event: {}".
                format(self.last_current_region, event_data))
            self.arrived = True
        elif event_type == "POSE":
            self.pose_handler.setPose(event_data)
        else:
            self.executor.postEvent(event_type, event_data)

    def find_region_mapping(self, name):
        """
        Takes the decomposed region name and tries to find the original name
        """
        for rname, subregs in self.executor.proj.regionMapping.iteritems():
            if name in subregs:
                break
        return rname

    def stop_local_game(self):
        if self.current_game is not None:
            self.current_game.game_done.set()
            self.current_game = None

    def create_local_game(self, cur_reg, next_reg, init_reg):
        logging.info("Creating a local game: {} {} {}".format(
            cur_reg, next_reg, init_reg))
        hier = Hierarchical(self.proj_name, self.proj_path, 2,
                            self.last_current_region)
        game = LocalGame(hier, 0, cur_reg, init_reg, next_reg,
                         self.listen_port)

        return game


def swap_exit(region):
    """
    Expects a region string as in: exit_{region1}_{region2}
    Returns exit_{region2}_{region1}
    """
    (fromr, to) = exit_dehelper(region)
    return exit_helper(to, fromr)


def exit_helper(fromr, to):
    """
    Returns exit_{fromr}_{to}
    """
    # TODO: what if there are several doors between the same two rooms?
    return "exit_" + "_".join([fromr, to])


def exit_dehelper(string):
    """
    Expects an exit string and returns the two regions as tuple
    """
    splits = string.split("_")
    return (splits[1], splits[2])
