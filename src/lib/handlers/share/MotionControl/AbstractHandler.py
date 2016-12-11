#!/usr/bin/env python
"""
===================================================================
abstractHandler.py
===================================================================

Uses the heat-controller to take a current position, current region, and destination region and return a global velocity vector that will help us get there
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
    def __init__(self, executor, shared_data, initial_region, num_layers):
        """
        initial_region (string): Initial region of the lowest level (default="11")
        num_layers (int): The total number of layers (default=3)
        """
        self.proj_name = executor.proj.project_basename.split(".")[0]
        self.proj_path = executor.proj.project_root + os.path.sep

        self.num_layers = num_layers

        # TODO: Don't allow "." in the file name other than those seperators
        prefix = executor.proj.getFilenamePrefix().split("#")[0]
        self.layer = int(prefix.split(".")[1])

        thingys = executor.proj.getFilenamePrefix().split(".", 2)

        # We are not the top level
        # TODO: do something smart about this
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
        If ``last`` is true, we will move to the center of the region.

        Returns ``True`` if we are outside the supposed ``current_reg``
        """
        current_reg = self.find_region_mapping(self.rfi.regions[current_reg]
                                               .name)
        self.cur_reg = current_reg
        next_reg = self.find_region_mapping(self.rfi.regions[next_reg].name)

        # if self.need_exits:
        #     self.exits = self.find_exit_child(next_reg)
        #     self.need_exits = False
        #     logging.error(self.exits)

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

        # No game is running
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
        # if event_type == "STATE":
        #     # If the state changes, stop the current game and create a new one
        #     (current_region, next_region) = event_data
        #     logging.info("STATE: {}".format(event_data))
        if event_type == "BORDER":
            logging.info("Borders crossed in MotionHandler to:{}".format(
                event_data))
            if event_data.startswith("exit") and self.mappings is not None:
                # We got to an exit, get the next region from it.
                # This is only used for the basic simulation, because the next
                # game has to know the initial region.
                # For ROS-simulation setting self.arrived to True is enough.
                if self.is_toplevel():
                    (ex, level, fr, to) = event_data.split("_")
                    regions = (fr + "_" + to)
                    self.last_current_region = self.mappings[fr][str(
                        self.layer - 1)]["{}.{}".format(level, regions)]
                else:
                    (ex, level, fr, to) = event_data.split("_")
                    reg = (level + "_" + fr + "_" + to)
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
            # From lower level, it was unsynthesizable, try to reorder
            self.executor.postEvent(
                "INFO", "The robot couldn't reach the goal " +
                self.current_goal + " because of " + str(event_data))
            logging.warning("Our child has failed us")
            self.executor.post_event_hierarchical(event_type,
                                                  self.current_goal)
        elif event_type == "STATS":
            self.executor.post_event_hierarchical(event_type, event_data)
        else:
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
        hier = Hierarchical(self.proj_name, self.proj_path, self.num_layers,
                            self.last_current_region)

        # TODO: no longer needed, since we're doing it in the localgame?
        # if next_region.startswith("exit"):
        #     nreg = [next_region]
        # elif next_region is None:
        #     nreg = None
        # else:
        #     exits = self.find_exit_child(next_region)
        #     nreg = [x.name for x in exits]

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

    def find_exit_child(self, to):
        """
        Load the regions of the level below and return the list of exits
        so we can choose another one if the first one is blocked.
        TODO: no longer needed, because we handle the exits in the local game.
        """
        rfi = regions.RegionFileInterface()
        # self.id might be None, so throw it out
        reg_path = "{}{}.{}.{}.regions".format(
            self.proj_path, self.proj_name, self.layer - 1, ".".join(
                s for s in [self.id, self.cur_reg] if s))
        rfi.readFile(reg_path)
        exits = []
        pattern = "exit_{}_{}_{}".format(str(self.layer - 1), self.cur_reg, to)
        for reg in rfi.regions:
            if reg.name.startswith(pattern):
                exits.append(reg)
        return exits

    def remove_exit(self, name):
        """
        Removes the exit with name `name`, if it was in the list
        TODO: no longer needed, because we handle the exits in the loca game.
        """
        logging.debug("Exits before: %s" % str(self.exits))
        for i, o in enumerate(self.exits):
            if o.name == name:
                del self.exits[i]
                break

        logging.debug("Exits after: %s" % str(self.exits))


exit_regex = re.compile(
    "exit_(?P<level>.+?)_(?P<from>.+?)_(?P<to>.+?)(#.*)?", re.I)


def regions_from_exit(ex_region):
    """
    Matches the given string against the exit regex and returns a tuple of two
    regions `(from, to)`
    """
    m = exit_regex.match(ex_region)
    return (m.group('level'), m.group('from'), m.group('to'))
