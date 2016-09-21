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
        project_name (string): Do something, shit! (default="Test")
        project_path (string): Do something, shit! (default="Test")
        initial_region (string): Do something, shit! (default="11")
        """
        self.proj_name = project_name
        self.proj_path = project_path
        self.initial_region = initial_region

        self.drive_handler = executor.hsub.getHandlerInstanceByType(
            handlerTemplates.DriveHandler)
        self.pose_handler = executor.hsub.getHandlerInstanceByType(
            handlerTemplates.PoseHandler)
        self.fwd_coordmap = executor.hsub.coordmap_map2lab
        self.rfi = executor.proj.rfi
        self.executor = executor
        self.last_warning = 0

        self.has_game = threading.Event()
        self.current_game = None
        self.current_thread = None
        self.arrived = False

        # This region is on level 0
        self.last_current_region = initial_region

        self.listen_port = None
        self.xmlrpc_server_thread = None
        self.setup_xmlrpc()

    def setup_xmlrpc(self):
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
            self.arrived = False
            return True

        if self.has_game.is_set() and (self.current_game.goal_region !=
                                       exit_helper(current_reg, next_reg) or
                                       self.arrived):

            if self.current_game.current_region is not None and not self.arrived:
                logging.info("Setting the last current region to {}".format(
                    self.current_game.current_region))
                self.last_current_region = self.current_game.current_region

            self.stop_local_game()

        if not self.has_game.is_set():
            logging.info("New game! \o/".format(current_reg, exit_helper(
                current_reg, next_reg), self.last_current_region))
            self.arrived = False
            # TODO set correct regions
            self.current_game = self.create_local_game(
                current_reg, exit_helper(current_reg,
                                         next_reg), self.last_current_region)
            self.current_thread = threading.Thread(
                target=self.current_game.run)
            self.current_thread.daemon = True
            self.has_game.set()
            self.current_thread.start()

        return self.arrived

    def handle_event(self, event_type, event_data):
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
            logging.info("Setting the last current region to {}, event: {}".format(self.last_current_region, event_data))
            self.arrived = True
        elif event_type == "POSE":
            self.pose_handler.setPose(event_data)
        else:
            self.executor.postEvent(event_type, event_data)

    def find_region_mapping(self, name):
        for rname, subregs in self.executor.proj.regionMapping.iteritems():
            if name in subregs:
                break
        return rname

    def stop_local_game(self):
        self.current_game.game_done.set()
        self.current_game = None
        self.has_game.clear()

    def create_local_game(self, cur_reg, next_reg, init_reg):
        logging.info("Creating a local game: {} {} {}".format(
            cur_reg, next_reg, init_reg))
        hier = Hierarchical(self.proj_name, self.proj_path, 2,
                            self.initial_region)
        game = LocalGame(hier, 0, cur_reg, self.listen_port)
        game.set_init_region(init_reg)
        game.set_goal_region(next_reg)

        return game


def swap_exit(region):
    (fromr, to) = exit_dehelper(region)
    return exit_helper(to, fromr)


def exit_helper(fromr, to):
    # TODO: what if there are several doors between the same two rooms?
    return "exit_" + "_".join([fromr, to])


def exit_dehelper(string):
    splits = string.split("_")
    return (splits[1], splits[2])
