#!/usr/bin/env python
"""
===================================================================
abstractHandler.py
===================================================================
"""

from numpy import *

import lib.handlers.handlerTemplates as handlerTemplates
from hierarchical import LocalGame, Hierarchical
import threading
from SimpleXMLRPCServer import SimpleXMLRPCServer
import socket
import os
import json, time
import logging
import globalConfig
import re
import regions


class AbstractHandler(handlerTemplates.MotionControlHandler):
    """
    Move to another region by instantiating a local game of the level below.
    """

    def __init__(self, executor, shared_data, initial_region, num_layers):
        """
        initial_region (string): Initial region of the lowest level (default="11")
        num_layers (int): The total number of layers (default=3)
        """
        self.proj_name = executor.proj.project_basename.split(".")[0]
        self.proj_path = executor.proj.project_root + os.path.sep

        # TODO: not really needed
        self.num_layers = num_layers

        # TODO: Don't allow "." in the file name other than those seperators
        prefix = executor.proj.getFilenamePrefix().split("#")[0]
        self.layer = int(prefix.split(".")[1])

        thingys = executor.proj.getFilenamePrefix().split(".", 2)

        # We are not the top level
        # TODO: do something smart about this
        # Example: name.1.1.3#hash
        # thingys == [name, 1, 1.3#hash], so we need to cut the hash
        # the id represents the path to the game, so we concatenate the regions of all levels
        if len(thingys) > 2:
            self.id = thingys[2].split("#")[0]
        else:
            self.id = None

        # self.pose_handler = executor.hsub.getHandlerInstanceByType(
        #     handlerTemplates.PoseHandler)
        self.fwd_coordmap = executor.hsub.coordmap_map2lab
        self.rfi = executor.proj.rfi
        self.executor = executor

        self.current_game = None
        self.arrived = False

        # will be the path to initial region of the lower level
        # Needed for basicSim
        self.last_current_region = ".".join(initial_region.split(".")[1:])

        # Only needed for basicSim for the next initial regions
        self.mappings = None
        if os.path.isfile(self.proj_path + "mappings.json"):
            with open(self.proj_path + "mappings.json") as f:
                self.mappings = json.load(f)

        self.listen_port = None
        self.xmlrpc_server_thread = None
        self.setup_xmlrpc()
        self.cur_reg = None
        self.current_goal = None

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

        self.xmlrpc_server_thread = threading.Thread(
            target=serv.serve_forever,
            name="AH.XMLrpc.{}.{}".format(self.id, self.listen_port))
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()

    def gotoRegion(self, current_reg, next_reg, last=False):
        """
        If ``last`` is true, we will move to the center of the region. Is ignored in this handler.

        Returns ``True`` if we are outside the supposed ``current_reg``
        """
        # `current_reg` and `next_reg` are the indices of the subregions, so convert them first
        current_reg = self.find_region_mapping(self.rfi.regions[current_reg]
                                               .name)
        self.cur_reg = current_reg
        next_reg = self.find_region_mapping(self.rfi.regions[next_reg].name)

        if self.arrived:
            # We seem to have arrived, so return true and stop the current game
            self.arrived = False
            if self.current_game is not None:
                self.stop_local_game()
            # self.need_exits = True
            return True

        # We should stay in this Building/Room, so set None as the goal
        if current_reg == next_reg:
            nreg = None
        else:
            nreg = next_reg

        # We need to go somewhere else and a game is running
        if (self.current_game is not None) and nreg != self.current_goal:
            logging.info("We need to change goals from %s to %s" %
                         (self.current_goal, nreg))
            self.stop_local_game()

        # No game is running => start a new game with the goal
        if not self.current_game:
            self.current_goal = nreg
            self.arrived = False
            self.current_game = self.create_local_game(
                current_reg, nreg, self.last_current_region)
            current_thread = threading.Thread(
                target=self.current_game.run,
                name="AbstractHandler_{}#{}".format(self.id, next_reg))
            current_thread.daemon = True
            current_thread.start()

        return self.arrived

    def handle_event(self, event_type, event_data):
        """
        Gets called if an event happened that is of interest
        """
        if event_type == "BORDER":
            logging.info("Borders crossed in MotionHandler to:{}".format(
                event_data))
            if event_data.startswith("exit") and self.mappings is not None:
                # We got to an exit, get the next region from it.
                # This is only used for the basic simulation, because the next
                # game has to know the initial region.
                # For ROS-simulation setting self.arrived to True is enough.
                if self.is_toplevel():
                    # Get the information based on the exit name
                    (ex, level, fr, to) = event_data.split("_")
                    regions = (fr + "_" + to)

                    # Search for the next region in the mapping
                    self.last_current_region = self.mappings[fr][str(
                        self.layer - 1)]["{}.{}".format(level, regions)]
                else:
                    # Get the information based on the exit name
                    (ex, level, fr, to) = event_data.split("_")
                    reg = (level + "_" + fr + "_" + to)

                    # Search for the next region in the mapping
                    self.last_current_region = self.mappings[self.id][str(
                        self.layer - 1)][reg]
            else:
                self.last_current_region = ".".join(
                    [self.executor.get_current_region(), event_data])
                self.executor.post_event_hierarchical("BORDER",
                                                      self.last_current_region)
            self.arrived = True
            logging.info(
                "eventstuff: Setting the last current region to {}, event: {}".
                format(self.last_current_region, event_data))
        elif event_type == "UNSYNTH":
            # From lower level, it was unsynthesizable
            self.executor.postEvent(
                "INFO", "The robot couldn't reach the goal " +
                self.current_goal + " because of " + str(event_data))
            logging.warning("Our child has failed us")
            # Let the local game decide what to do (reorder?)
            self.executor.post_event_hierarchical(event_type,
                                                  self.current_goal)
        elif event_type == "STATS":
            # Pass them on until it reaches the top
            self.executor.post_event_hierarchical(event_type, event_data)
        else:
            # Got something else
            self.executor.postEvent(event_type, event_data)

    def _stop(self):
        """
        Stop all of my children and then myself
        """
        # Stop the local game
        self.stop_local_game()

    def find_region_mapping(self, name):
        """
        Takes the decomposed region name and tries to find the original name
        """
        for rname, subregs in self.executor.proj.regionMapping.iteritems():
            if name in subregs:
                return rname
        logging.error("RegionMapping didn't have our region")
        return None

    def stop_local_game(self):
        """
        Shuts down the local game, which in turn shuts the children down
        """
        if self.current_game is not None:
            self.current_game.stop()
            self.current_game = None

    def create_local_game(self, cur_reg, next_region, init_reg):
        """
        Create the local game based on the current and next region
        """
        hier = Hierarchical(self.proj_name, self.proj_path, self.num_layers,
                            self.last_current_region)

        logging.info("Creating a local game: {}, {} {} {}".format(
            self.layer - 1, ".".join(s for s in [self.id, cur_reg]
                                     if s), init_reg, next_region))
        # The new ID will be the current id appended with the current region,
        # if we're not on the highest level
        game = LocalGame(hier, self.layer - 1, ".".join(
            s for s in [self.id, cur_reg]
            if s), next_region, init_reg, self.listen_port)
        return game

    def exit_helper(self, fr, to):
        return "exit_{}_{}_{}".format(self.layer - 1, fr, to)

    def is_toplevel(self):
        return self.id is None
