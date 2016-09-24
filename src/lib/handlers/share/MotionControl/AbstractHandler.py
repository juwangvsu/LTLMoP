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
import os
import json


class AbstractHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data, initial_region, num_layers):
        """
        initial_region (string): Initial region of the lowest level (default="11")
        num_layers (int): The total number of layers (default=3)
        """
        self.proj_name = executor.proj.project_basename.split(".")[0]
        self.proj_path = executor.proj.project_root + os.path.sep

        self.num_layers = num_layers

        # TODO: Don't allow "." in the file name other than those seperators
        self.layer = int(executor.proj.getFilenamePrefix().split(".")[1])

        thingys = executor.proj.getFilenamePrefix().split(".", 2)

        # We are not the top level
        # TODO: do something smart about this
        if len(thingys) > 2:
            self.id = thingys[2].split("#")[0]
        else:
            self.id = "0"

        self.pose_handler = executor.hsub.getHandlerInstanceByType(
            handlerTemplates.PoseHandler)
        self.fwd_coordmap = executor.hsub.coordmap_map2lab
        self.rfi = executor.proj.rfi
        self.executor = executor

        self.current_game = None
        self.arrived = False

        # will be the path to initial region of the lower level
        self.last_current_region = ".".join(initial_region.split(".")[1:])

        with open(self.proj_path + "mappings.json") as f:
            self.mappings = json.load(f)

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

        # logging.error("going from {} to {}".format(current_reg, next_reg))

        if self.arrived:
            # We seem to have arrived, so return true and stop the current game
            self.arrived = False
            if self.current_game is not None:
                # TODO: do we need to sleep here?
                time.sleep(1)
                self.stop_local_game()
            return True

        # We need to go somewhere else and a game is running
        nreg = next_reg if next_reg.startswith("exit") else self.exit_helper(
            current_reg, next_reg)
        if (self.current_game is not None
            ) and nreg != self.current_game.goal_region:
            # TODO remove debug
            logging.error(self.id + ": We need to change!")
            logging.error(nreg)
            logging.error(self.current_game.goal_region)
            self.last_current_region = self.current_game.current_region
            logging.error(self.last_current_region)
            self.stop_local_game()

        # No game is running
        if not self.current_game:
            self.arrived = False
            if current_reg != next_reg:
                self.current_game = self.create_local_game(
                    current_reg, next_reg, self.last_current_region)
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
            logging.info("Borders crossed in MotionHandler to:{}".format(
                event_data))

            # FOO
            # We got to an exit, get the next region from it
            if self.is_toplevel():
                (ex, level, regions) = event_data.split(".")
                (fr, to) = regions.split("_")
                self.last_current_region = self.mappings[fr][str(
                    self.layer - 1)]["{}.{}".format(level, regions)]
            else:
                (ex, regions) = event_data.split(".", 1)
                self.last_current_region = self.mappings[self.id][str(
                    self.layer - 1)][regions]

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
        hier = Hierarchical(self.proj_name, self.proj_path, self.num_layers,
                            self.last_current_region)

        # TODO: Do something smart so this isn't that ugly
        # If the next region starts with exit already we assume it is the exit of the upper level
        if next_reg.startswith("exit"):
            goal_for_game = next_reg
        else:
            goal_for_game = self.exit_helper(cur_reg, next_reg)

        # Check if we're the top level
        if self.is_toplevel():
            logging.info("Creating a local game: {}, {} {} {}".format(
                self.layer - 1, cur_reg, init_reg, next_reg))
            game = LocalGame(hier, self.layer - 1, cur_reg, init_reg,
                             goal_for_game, self.listen_port)
        else:
            # If not, create the id by appending the current region to our id
            logging.info("Creating a local game: {}, {} {} {}".format(
                self.layer - 1, ".".join(
                    x for x in [self.id, cur_reg]), init_reg, goal_for_game))
            game = LocalGame(hier, self.layer - 1,
                             ".".join(x for x in [self.id, cur_reg]), init_reg,
                             next_reg, self.listen_port)
        return game

    def exit_helper(self, fr, to):

        return "exit.{}.{}_{}".format(self.layer - 1, fr, to)

    def is_toplevel(self):
        return self.id == "0"

    # def find_exit(self, level, from_, to):
    #     for rname, subregs in self.executor.proj.regionMapping.iteritems():
    #         logging.error(rname)
    #         if not rname.startswith("exit"):
    #             continue
    #         if rname.startswith("exit." + str(level) + "." + from_ + "_" + to):
    #             logging.error("FOUND IT" + rname)
    #             return rname
    #         logging.error("rname doesn't start with " + "exit." + str(level) + "." + from_ + "_" + to + "|||" + rname)
    #     return None


def exit_string(str):
    """
    exit.level.from_to
    """
    splits = str.split(".")
    return (splits[0], splits[1], splits[2])


def swap_exit(str):
    (ex, level, from_to) = exit_string(str)
    (fr, to) = from_to.split("_")
    return "exit.{}.{}_{}".format(level, to, fr)
