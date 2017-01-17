import sys, os, random
import fileinput
import re
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading
from lib.execute import execute_main
import socket
import hashlib
import regions

import logging
import csv
import lib.globalConfig
import json, datetime
from lib.specCompiler import SpecCompiler

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p, "src", "lib"))
sys.path.append(os.path.join(p, "lib", "cores"))

goal_regex = re.compile("^(?P<spec>(Go to ))(?P<region>.*)", re.I)


# Highest level / Metainformation
# TODO: not really needed, remove it
class Hierarchical(object):
    def __init__(self, name, path, layers, init_region):
        self.path = path
        self.games = []
        self.initial_pose = 0
        self.name = name
        self.layers = layers
        self.init_region = init_region


# Local game
class LocalGame(object):
    def __init__(self,
                 project,
                 level,
                 id,
                 goal_region,
                 initial_region=None,
                 parent_port=None):
        self.proj = project  # Reference to Hierarchical, not an LTLMoP project
        self.level = int(level)  # Hierarchical level we are on
        self.id = id

        # Check if we represent the highest level
        if self.is_toplevel():
            self.region = None
            self.path_prefix = project.path + project.name + "." + level
        else:
            self.region = id.split(".")[-1]  # the region we represent
            self.path_prefix = path_helper(project.path, project.name, level,
                                           id)

        # Also, set up the stats logging if on highest level
        if self.is_toplevel():
            self.stats_logger = logging.getLogger('stats')
            fh = logging.FileHandler(self.path_prefix + ".log")
            self.stats_logger.addHandler(fh)
            self.stats_logger.info("#Stats started\n#=======")

        # Append positions to the csv file if we're the lowest level, so we don't
        # have to pass around the position across layers
        if self.level == 0:
            # initialize position handler
            self.position_fd = open(
                project.path + project.name + '_positions.csv', 'a')
            self.csv_writer = csv.writer(self.position_fd)

        self.spec_path = self.path_prefix + ".spec"
        self.region_path = self.path_prefix + ".regions"
        self.had_changes = threading.Event()
        self.last_outputs = {}

        # initialize with the original spec
        self.compiler = SpecCompiler(self.spec_path)

        # Get the text from the spec and remove empty lines
        self.spec_list = list(self.compiler.specText.splitlines(True))
        # Make sure the last line has a newline as well, since we will
        # need it when reordering
        if not self.spec_list[-1].endswith("\n"):
            self.spec_list[-1] = self.spec_list[-1] + "\n"

        self.current_region = initial_region

        # only needed for basicSim, otherwise the position is acquired by the posehandler
        if initial_region is not None:
            self.set_init_region(initial_region)

        # Set the goal region based on the goal_region we have got
        self.goal_region = self.get_goal_region(goal_region)
        self.set_goal_region()

        self.game_done = threading.Event()
        self.executor_ready_event = threading.Event()
        self.executor_port = None

        self.strat_path = None  # Gets set in write_spec_file()
        self.target_spec_path = None
        self.write_spec_file()

        # Get set up in executor_setup
        self.executor_proxy = None
        self.serv = None
        self.exec_thread = None
        self.xmlrpc_server_thread = None

        # self.parent is an AbstractHandler
        if parent_port is None:
            self.parent = None
        else:
            self.parent = xmlrpclib.ServerProxy(
                "http://127.0.0.1:{}".format(parent_port), allow_none=True)

        # Find a port for the local server and start it
        while True:
            self.listen_port = random.randint(10000, 65535)
            try:
                self.serv = SimpleXMLRPCServer(
                    ("127.0.0.1", self.listen_port),
                    logRequests=False,
                    allow_none=True)
            except socket.error:
                pass
            else:
                break

        # Register the callbacks for when the executor is ready and the messages from it
        self.serv.register_function(self.executor_ready)
        self.serv.register_function(self.handle_event)
        self.xmlrpc_server_thread = threading.Thread(
            target=self.serv.serve_forever,
            name="LG.XMLrpc.{}.{}".format(self.id, self.listen_port))
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()

    def synthesize(self):
        """
        Synthesize the current specification.
        Returns `True` if it was synthesizable, `False` otherwise
        """
        if self.is_dirty():
            compiler = SpecCompiler(self.target_spec_path)
            (synthesizable, b, msg) = compiler.compile()
            if not synthesizable:
                self.post_event_parent("INFO", "Compilation failed")
                logging.error("Compilation failed: {}, {}, {}".format(
                    b, msg, self.target_spec_path))
                self.post_event_parent("INFO", "We can't get to %s, log: %s" %
                                       (self.goal_region, msg))
                self.post_event_parent("UNSYNTH", str(self.goal_region))
                self.log_stats("UNSYNTH", msg)
                self.stop()
                return False
            else:
                return True

    def set_init_region(self, region):
        """
        Set the init region by replacing it in the specification file
        """
        # TODO: Do it properly (i.e. not via fiddling with the file)
        # Only needed for simulation with basicSim
        if region is not None:
            # Get only the region of the current level for the init handler
            # but keep the whole path for the abstract handler
            split = region.split(".")
            if len(split) > 1:
                reg = region.split(".")[-(self.level + 1)]
            else:
                reg = region

            config_name = self.find_current_config()
            is_it_the_line = False
            for line in fileinput.input(
                    self.proj.path + "/configs/" + config_name, inplace=True):
                if is_it_the_line:
                    line = re.sub('init_region="(.*)"',
                                  'init_region="' + reg + '"', line)
                    is_it_the_line = False
                if line.startswith("share.MotionControl.AbstractHandler"):
                    line = re.sub('initial_region="(.*)"',
                                  'initial_region="' + region + '"', line)
                if line.startswith("InitHandler:"):
                    is_it_the_line = True

                sys.stdout.write(line)
        else:
            logging.warning("Can't set initial region, it was None")

    def set_goal_region(self):
        """
        Iterates over the specification and changes the goal to the goal region.
        If the goal region is None we delete the entry.
        """
        index = None
        for i, line in enumerate(self.spec_list):
            match = goal_regex.match(line)
            if match:
                index = i

        # If the goal is None, delete the line, otherwise replace the line
        if index is not None and self.goal_region is None:
            del self.spec_list[index]
        elif index is not None and self.goal_region is not None:
            self.spec_list[index] = "go to " + " or ".join(
                self.goal_region) + "\n"

    def avoid_region(self, region_name):
        """
        To avoid a region we insert the line "always not `region`" and set the `had_changes` flag.
        """
        self.spec_list.insert(0, "always not %s\n" % (region_name))
        self.had_changes.set()

    def write_spec_file(self):
        """
        Sets the goal region of the game by changing the specification file.
        Also add the regions to avoid in the end
        """

        # Update the paths to represent the current specification
        hash = self.sha1_of_spec()
        self.target_spec_path = "{}#{}.spec".format(self.path_prefix, hash)
        self.strat_path = "{}#{}.aut".format(self.path_prefix, hash)

        # Since the specCompiler expects the text as multiple lines, convert it
        text = "".join(self.spec_list)
        self.compiler.specText = text
        self.compiler.proj.specText = text
        self.compiler.proj.writeSpecFile(self.target_spec_path)

        # We have changed the specification, so we want to iterate in run() again
        self.game_done.clear()

    def run(self):
        """
        Runs the local game, but checks first if it has to be synthesized.
        It will reiterate, if the game is done, but the `had_changes` flag is set.
        """
        logging.info("Running the game")

        self.had_changes.set()
        while self.had_changes.is_set():
            # The game might change the regions to avoid
            self.had_changes.clear()

            # Set up xmlrpc
            if self.is_dirty():
                self.post_event_parent(
                    "INFO", "Starting to synthesize with goal %s..." %
                    self.goal_region)
                if not self.synthesize():
                    break
                self.post_event_parent("INFO", "Done synthesizing")
            else:
                self.post_event_parent("INFO",
                                       "Strategy was already synthesized")
            self.executor_setup()

            # Wait until the game is done
            self.game_done.wait()
            self.last_outputs = self.executor_proxy.get_current_outputs()

            self.stop_executor()
        logging.info("Run loop is over")

    def stop(self):
        """
        Sets the `game_done` event, so run() continues and tries to stop everything else
        """
        self.game_done.set()
        self.teardown()

    def get_current_region(self):
        return self.current_region

    def executor_setup(self):
        """
        Set up the xmlrpc connection between the executor and the game
        """

        # Start the executor in a new thread
        logging.warning("Current reg: " + str(self.current_region))
        self.exec_thread = threading.Thread(
            target=execute_main,
            args=[
                None, self.target_spec_path, self.strat_path, True,
                self.listen_port, self.last_outputs, self.current_region
            ],
            name="Executor_{}#{}".format(self.strat_path, self.goal_region))
        self.exec_thread.daemon = True
        self.exec_thread.start()

        logging.info("Waiting for the executor to be ready")
        self.executor_ready_event.wait()

        # Set up a proxy for the executor
        self.executor_proxy = xmlrpclib.ServerProxy(
            "http://127.0.0.1:{}".format(self.executor_port), allow_none=True)

        addr = "http://127.0.0.1:{}".format(self.listen_port)
        self.executor_proxy.registerHierarchicalEventTarget(addr)

        self.executor_proxy.postEvent(
            "INFO", "Game on level {}: {} {} {}".format(
                self.level, self.id, self.current_region, self.goal_region))

        self.game_done.clear()

    def pause(self):
        self.executor_proxy.pause()

    def resume(self):
        self.executor_proxy.resume()

    def teardown(self):
        """
        Stopping the executor and XMLrpc server
        """
        self.stop_executor()

        logging.debug("Shutting down serv")
        if self.serv:
            self.serv.shutdown()
            self.serv.server_close()

        logging.debug("Waiting for xmlrpc_server_thread")
        if self.xmlrpc_server_thread:
            self.xmlrpc_server_thread.join()
        logging.info("Waiting for the execution thread to finish")
        logging.info("Local game shut down")

        # Since we've opened the file when initializing, we have to close it now
        if self.level == 0:
            self.position_fd.close()

    def find_current_config(self):
        """
        Find the name of the current config in the specification file
        """
        is_it_the_line = False
        with open(self.spec_path, "r") as file:
            for line in file:
                line = line.strip()

                if is_it_the_line:
                    # keep in sync with hsubConfigObjects.py:835
                    return line.replace(' ', '_') + ".config"

                if line.startswith("CurrentConfigName:"):
                    is_it_the_line = True

        logging.warning("CurrentConfigName not found")

    def stop_executor(self):
        """
        Stops the executor via the proxy and resets all
        related variables.
        """
        self.executor_ready_event.clear()
        if self.executor_proxy is not None:
            try:
                self.executor_proxy.shutdown()
            except:
                pass
            self.executor_proxy = None
        if self.exec_thread is not None:
            self.exec_thread.join()
            self.exec_thread = None
        self.executor_port = None

    def executor_ready(self, port):
        """Is to be called by the executor after it is done setting up its xmlrpc server"""
        self.executor_ready_event.set()
        self.executor_port = port

    def handle_event(self, event_type, event_data):
        """Is called from the execute/executeStrategy on events, like borders crossed"""
        if event_type == "POSE":
            # If we get a pose just pass it to the parent, needed to keep the dots of
            # basicSim in sync, not needed otherwise
            if self.level == 0:
                self.csv_writer.writerow(event_data)
        if event_type == "BORDER":
            self.current_region = event_data
            logging.info("Current reg now: " + str(self.current_region))
            if event_data.startswith("exit"):
                self.post_event_parent(event_type, event_data)
                self.game_done.set()
        elif event_type == "FAIL":
            # We couldn't move somewhere, so try to avoid the region
            # if event_data.startswith("exit"):
            # If it was an exit, tell our parent that we can't go to that region
            # TODO: in case we allow multiple exits, find another exit first and set it as goal
            # (level, fr, to) = regions_from_exit(event_data)
            # self.post_event_parent("FAIL", to)
            #     self.post_event_parent("FAIL", event_data)
            # else:
            logging.warning("We can't go to {}, so try to avoid it".format(
                event_data))
            self.avoid_region(event_data)
            self.log_stats("FAIL", event_data)
            self.write_spec_file()
            self.game_done.set()
        elif event_type == "UNSYNTH":
            logging.warning("We couldn't go to {}, so try to go there later".
                            format(event_data))
            self.executor_proxy.postEvent(
                "INFO",
                "We couldn't go to {}, so try to go there later".format(
                    event_data))

            if not self.move_to_end(event_data):
                self.write_spec_file()
            else:
                logging.error("This was already our last target")
                self.executor_proxy.postEvent(
                    "INFO",
                    "Our target was at the last position already, stopping")
                self.pause()
            self.game_done.set()
        elif event_type == "STATS":
            # STATS from a child, pass it through and append information
            if self.is_toplevel():
                self.stats_logger.info(json.dumps(event_data))
            else:
                self.post_event_parent(event_type, event_data)
        else:
            # logging.debug("Got something else: (%s) %s" %
            # (event_type, event_data))
            self.post_event_parent(event_type, event_data)

    def get_goal_region(self, goal):
        """
        Returns a list of strings with the names of regions to go to
        """
        # In case it is an upper level exit, keep it
        if not goal:
            return None
        elif goal.startswith("exit"):
            return [goal]
        # Else search for all exits to that region
        else:
            return [x.name for x in self.find_exits(goal)]

    def find_exits(self, goal):
        """
        Finds all exits that lead from the current region to the goal
        """
        rfi = regions.RegionFileInterface()
        # self.id might be None, so throw it out
        rfi.readFile(self.region_path)
        exits = []
        pattern = "exit_{}_{}_{}".format(self.level, self.region, goal)
        for reg in rfi.regions:
            if reg.name.startswith(pattern):
                exits.append(reg)
        return exits

    def move_to_end(self, region):
        """
        Tries to reorder the regions to visit, so `region` is in the last line
        and gets visited last.
        Changes specification.
        Returns `True` iff the region was at the last position already
        """
        index = None

        for i, line in enumerate(self.spec_list):
            if "visit " + region in line:
                index = i

        if index == len(self.spec_list) - 1:
            return True

        if index is not None:
            self.spec_list.insert(
                len(self.spec_list), self.spec_list.pop(index))
            self.had_changes.set()
        else:
            logging.warning("Region we wanted to reorder not found")
        return False

    def sha1_of_spec(self):
        """
        Hashes `self.spec_list` and returns its hexdigest
        """
        text = "".join(self.spec_list)
        return hashlib.sha1(text).hexdigest()

    def post_event_parent(self, event_type, event_data):
        """
        Sends the event to its parent (which is typically an AbstractHandler)
        """
        if self.parent is not None:
            self.parent.handle_event(event_type, event_data)

    def is_toplevel(self):
        return self.id is None

    def is_dirty(self):
        """
        Check if there is an automaton for the current specification already
        """
        return not os.path.isfile(self.strat_path)

    def log_stats(self, event_type, reason):
        """
        Tries to log stats by either sending it up to the parent if we're not
        on the top level or by logging it to the file if we are on top
        """
        event = {}
        event['path'] = [self.id],
        event['goal'] = self.goal_region,
        event['time'] = datetime.datetime.now().isoformat()
        event['region'] = self.current_region
        event['reason'] = reason
        event['type'] = event_type

        if self.is_toplevel():
            self.stats_logger.info(json.dumps(event))
        else:
            self.post_event_parent("STATS", event)


# HELPERS
def path_helper(path, name, level, id):
    return path + name + "." + ".".join(str(x) for x in [level, id])


def layer_helper(level, *arg):
    """
    Convert a hierarchy level and a list of integers (ids of games) to a path.
    For example: (0, 1, 3) should return "0.1.3".
    """
    return level + "." + ".".join(str(x) for x in arg)


def exit_helper(level, fromr, to):
    return "exit_" + level + "_" + "_".join([fromr, to])


exit_regex = re.compile("exit_(?P<level>.+?)_(?P<from>.+?)_(?P<to>.+?)(#.*)?",
                        re.I)


def regions_from_exit(ex_region):
    """
    Matches the given string against the exit regex and returns a tuple of two regions (from, to)
    """
    m = exit_regex.match(ex_region)
    return (m.group('level'), m.group('from'), m.group('to'))


if __name__ == "__main__":
    """ When run as script, i.e. for the highest level """
    spec_path = sys.argv[1]
    try:
        (root, filename) = os.path.split(spec_path)
        (name, level, ext) = filename.split(".")
        hier = Hierarchical(name, root + '/', level, "1")
        game = LocalGame(hier, level, None, None)
        game.run()
    except KeyboardInterrupt:
        game.stop()
        raise
