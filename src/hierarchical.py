import sys, os, random
import logging
import globalConfig
import fileinput
import re
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading
from lib.execute import execute_main
import socket
import hashlib
import regions

from lib.specCompiler import SpecCompiler

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

goal_regex = re.compile("^(?P<spec>(Go to))(?P<region>.*)", re.I)


# Highest level
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
        self.level = level
        self.id = id
        self.region = id.split(".")[-1] # the region we represent
        self.path_prefix = path_helper(project.path, project.name, level, id)
        self.spec_path = self.path_prefix + ".spec"
        self.region_path = self.path_prefix + ".regions"
        self.had_changes = threading.Event()

        self.current_region = initial_region
        self.avoid_regions = []
        self.goal_region = self.get_goal_region(goal_region)
        self.game_done = threading.Event()
        self.executor_ready_event = threading.Event()
        self.executor_port = None

        # File gets written when setting the goal
        self.target_spec_path = "{}#{}#{}.spec".format(self.path_prefix,
                                                       goal_region, str(self.avoid_regions))
        self.strat_path = "{}#{}#{}.aut".format(self.path_prefix, goal_region,
                                                str(self.avoid_regions))

        # If there is an automaton file we assume it was synthesized already
        self.dirty = not os.path.isfile(self.strat_path)

        # self.set_init_region(initial_region)
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
            self.parent = xmlrpclib.ServerProxy("http://127.0.0.1:{}".format(
                parent_port))

    def synthesize(self):
        """
        Synthesize the current specification.
        Returns `True` if it was synthesizable, `False` otherwise
        """
        if self.dirty:
            compiler = SpecCompiler(self.target_spec_path)
            (synthesizable, b, msg) = compiler.compile()
            if not synthesizable:
                self.parent.handle_event("INFO", "Compilation failed")
                logging.error("Compilation failed: {}, {}, {}".format(
                    b, msg, self.target_spec_path))
                self.parent.handle_event("INFO", "We can't get to %s, log: %s"
                                         % (self.goal_region, msg))
                self.stop()
                return False
            else:
                self.dirty = False
                return True

    def set_init_region(self, region):
        """ Set the init region by replacing it in the specification file """
        # TODO: Do it properly (i.e. not via fiddling with the file)
        # Only needed for simulation with basicSim
        if region is not None:
            # Get only the region of the current level for the init handler
            # but keep the whole path for the abstract handler
            reg = region.split(".")[-(self.level + 1)]

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

    def write_spec_file(self):
        """
        Sets the goal region of the game by changing the specification file
        TODO: don't do it by fiddling with files
        """
        if self.dirty:
            with open(self.spec_path, 'r') as input, open(
                    self.target_spec_path, 'w') as output:

                lines = input.readlines()
                for line in lines:
                    match = goal_regex.match(line)
                    if match and (self.goal_region is not None):
                        line = match.group('spec') + " " + " or ".join(
                            self.goal_region)
                    elif match and (self.goal_region is None):
                        line = "\n"

                    output.write(line)
                for reg in self.avoid_regions:
                    output.write("always not " + str(reg) + "\n")

    def change_regions(self):
        """
        """
        self.had_changes.set()
        self.avoid_regions.sort()
        self.target_spec_path = "{}#{}#{}.spec".format(self.path_prefix,
                                                       self.goal_region, str(self.avoid_regions))
        self.strat_path = "{}#{}#{}.aut".format(self.path_prefix,
                                                self.goal_region, str(self.avoid_regions))
        self.dirty = True
        self.write_spec_file()

    def run(self):
        """
        Runs the local game, but checks first if it has to be synthesized
        (if regions have changed)
        TODO: cache the synthesized strategies on disk
        """
        logging.info("Running the game")

        self.had_changes.set()
        while self.had_changes.is_set():
            # The game might change the regions to avoid
            self.had_changes.clear()

            # Set up xmlrpc
            if self.dirty:
                self.parent.handle_event(
                    "INFO", "Starting to synthesize with goal %s..." %
                    self.goal_region)
                if not self.synthesize():
                    break
                self.parent.handle_event("INFO", "Done synthesizing")
            else:
                self.parent.handle_event("INFO",
                                         "Strategy was already synthesized")
            self.executor_setup()

            # Wait until the game is done
            self.game_done.wait()

    def stop(self):
        self.game_done.set()
        self.teardown()

    def get_current_region(self):
        return self.current_region

    def executor_setup(self):
        """Set up the xmlrpc connection between the executor and the game"""
        # Find a port for the local server and start it
        while True:
            listen_port = random.randint(10000, 65535)
            try:
                self.serv = SimpleXMLRPCServer(
                    ("127.0.0.1", listen_port),
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
            target=self.serv.serve_forever)
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()

        # Start the executor in a new thread
        self.exec_thread = threading.Thread(
            target=execute_main,
            args=[
                None, self.target_spec_path, self.strat_path, True, listen_port
            ],
            name="LocalGame_{}#{}".format(self.id, self.goal_region))
        self.exec_thread.daemon = True
        self.exec_thread.start()

        logging.info("Waiting for the executor to be ready")
        self.executor_ready_event.wait()

        # Set up a proxy for the executor
        self.executor_proxy = xmlrpclib.ServerProxy(
            "http://127.0.0.1:{}".format(self.executor_port), allow_none=True)
        self.executor_proxy.registerHierarchicalEventTarget(
            "http://127.0.0.1:{}".format(listen_port))

        self.executor_proxy.postEvent(
            "INFO", "Game on level {}: {} {} {}".format(
                self.level, self.id, self.current_region, self.goal_region))

    def pause(self):
        self.executor_proxy.pause()

    def resume(self):
        self.executor_proxy.resume()

    def teardown(self):
        self.executor_proxy.shutdown()

        logging.debug("Shutting down serv")
        self.serv.shutdown()
        self.serv.server_close()

        logging.debug("Waiting for xmlrpc_server_thread")
        self.xmlrpc_server_thread.join()
        logging.info("Waiting for the execution thread to finish")
        self.exec_thread.join()
        logging.info("Local game shut down")

    def find_current_config(self):
        """Find the name of the current config in the specification file"""
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

    def executor_ready(self, port):
        """Is to be called by the executor after it is done setting up its xmlrpc server"""
        self.executor_ready_event.set()
        self.executor_port = port

    def handle_event(self, event_type, event_data):
        """Is called from the execute/executeStrategy on events, like borders crossed"""
        # if event_type == "POSE":
        # If we get a pose just pass it to the parent
        # self.parent.handle_event(event_type, event_data)
        # self.executor_proxy.postEvent(event_type, event_data)
        if event_type == "BORDER":
            self.current_region = event_data
            if event_data.startswith("exit"):
                self.parent.handle_event(event_type, event_data)
                self.game_done.set()
        elif event_type == "FAIL":
            # We couldn't move somewhere, so try to avoid the region
            # if event_data.startswith("exit"):
            # If it was an exit, tell our parent that we can't go to that region
            # TODO: in case we allow multiple exits, find another exit first and set it as goal
            # (level, fr, to) = regions_from_exit(event_data)
            # self.parent.handle_event("FAIL", to)
            #     self.parent.handle_event("FAIL", event_data)
            # else:
            logging.warning("We can't go to {}, so try to avoid it".format(
                event_data))
            self.avoid_regions.append(event_data)
            self.change_regions()

            self.game_done.set()
        else:
            logging.debug("Got something else: (%s) %s" %
                          (event_type, event_data))
            self.parent.handle_event(event_type, event_data)

    def get_goal_region(self, goal):
        """
        Returns a list of strings with the names of regions to go to
        """
        # In case it is an upper level exit, keep it
        if goal.startswith("exit"):
            return [goal]
        # Else search for all exits to that region
        else:
            return [x.name for x in self.find_exits(goal)]

    def find_exits(self, goal):
        rfi = regions.RegionFileInterface()
        # self.id might be None, so throw it out
        rfi.readFile(self.region_path)
        exits = []
        pattern = "exit.{}.{}_{}".format(self.level, self.region, goal)
        for reg in rfi.regions:
            if reg.name.startswith(pattern):
                exits.append(reg)
        return exits


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
    return "exit." + level + "." + "_".join([fromr, to])


exit_regex = re.compile(
    "exit\.(?P<level>.+?)\.(?P<from>.+?)_(?P<to>.+?)(#.*)?", re.I)


def regions_from_exit(ex_region):
    """
    Matches the given string against the exit regex and returns a tuple of two regions (from, to)
    """
    m = exit_regex.match(ex_region)
    return (m.group('level'), m.group('from'), m.group('to'))


def sha1_of_file(path):
    with open(path, 'rb') as f:
        return hashlib.sha1(f.read()).hexdigest()
