import sys, os, random
import subprocess
import logging
import fileinput
import re
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading
from lib.execute import execute_main

from lib.specCompiler import SpecCompiler

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)


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
    def __init__(self, project, level, id, parent_port=None):
        self.proj = project
        self.level = level
        self.id = id
        self.path_prefix = path_helper(project.path, project.name, level, id)
        self.spec_path = self.path_prefix + ".spec"
        self.region_path = self.path_prefix + ".regions"
        self.strat_path = self.path_prefix + ".aut"
        self.dirty = True
        self.goal_region = None
        self.current_region = None
        self.game_done = threading.Event()
        self.executor_ready_event = threading.Event()
        self.executor_port = None

        # Get set up in executor_setup
        self.executor_proxy = None
        self.serv = None
        self.exec_thread = None
        self.xmlrpc_server_thread = None

        if parent_port is None:
            self.parent = None
        else:
            self.parent = xmlrpclib.ServerProxy("http://127.0.0.1:{}".format(
                parent_port))

    def synthesize(self):
        """Synthesize the current specification"""
        compiler = SpecCompiler(self.spec_path)
        (synthesizable, b, c) = compiler.compile()
        if synthesizable:
            self.dirty = False
        else:
            logging.error("Compilation went wrong")

    def set_init_region(self, region):
        """ Set the init region by replacing it in the specification file """
        # TODO: Do it properly (i.e. not via fiddling with the file)
        if region is not None:

            config_name = self.find_current_config()
            logging.warning(self.path_prefix + "/configs/" + config_name)
            is_it_the_line = False
            for line in fileinput.input(
                    self.proj.path + "/configs/" + config_name, inplace=True):
                if is_it_the_line:
                    line = re.sub('init_region="(.*)"',
                                  'init_region="' + region + '"', line)
                if line.startswith("InitHandler:"):
                    is_it_the_line = True

                sys.stdout.write(line)
        else:
            logging.warning("Can't set initial region, it was None")

    def set_goal_region(self, region):
        """Sets the goal region of the game by changing the specification file
        TODO: don't do it by fiddling with files
        """
        goal_regex = re.compile("^(?P<spec>(Go to)|(visit))(?P<region>.*)",
                                re.I)
        if self.goal_region != region:
            self.dirty = True
            self.goal_region = region

            for line in fileinput.input(self.spec_path, inplace=True):

                match = goal_regex.match(line)
                if match:
                    line = match.group('spec') + " " + region

                sys.stdout.write(line)
        return

    def run(self):
        """Runs the local game, but checks first if it has to be synthesized (if goal or initial region changed)
        TODO: cache the synthesized strategies on disk
        """
        logging.info("Running the game")

        # Set up xmlrpc
        if self.dirty:
            self.parent.handle_event("INFO", "Starting to synthesize...")
            self.synthesize()
            self.parent.handle_event("INFO", "Done synthesizing")
        self.executor_setup()

        # Wait until the game is done
        self.game_done.wait()

        logging.info("Stopping myself")
        # Stop xmlrpc
        self.teardown()

    def executor_setup(self):
        """Set up the xmlrpc connection between the executor and the game"""
        executorInitialised = threading.Event()
        # Find a port for the local server and start it
        while True:
            listen_port = random.randint(10000, 65535)
            try:
                self.serv = SimpleXMLRPCServer(
                    ("127.0.0.1", listen_port),
                    logRequests=False,
                    allow_none=True)
            except socket.error as e:
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
            args=[None, self.spec_path, self.strat_path, True, listen_port])
        self.exec_thread.daemon = True
        self.exec_thread.start()

        logging.info("Waiting for the executor to be ready")
        self.executor_ready_event.wait()

        # Set up a proxy for the executor
        self.executor_proxy = xmlrpclib.ServerProxy(
            "http://127.0.0.1:{}".format(self.executor_port), allow_none=True)
        self.executor_proxy.registerHierarchicalEventTarget(
            "http://127.0.0.1:{}".format(listen_port))

    def teardown(self):
        # Clean up on exit
        self.executor_proxy.postEvent("CLOSE", "")

        # self.executor_proxy.pause()
        # self.executor_proxy.shutdown()
        logging.info("Waiting for XML-RPC server to shut down...")
        self.serv.shutdown()
        self.xmlrpc_server_thread.join()
        self.exec_thread.join()
        logging.info("XML-RPC server shutdown complete.  Goodbye.")

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
        if event_type == "POSE":
            # If we get a pose just pass it to the parent
            self.parent.handle_event(event_type, event_data)
            # self.executor_proxy.postEvent(event_type, event_data)
        elif event_type == "BORDER":
            self.current_region = event_data
            if self.goal_region == event_data:
                self.parent.handle_event(event_type, event_data)
                self.game_done.set()

# class AbstractGame(LocalGame):
#     def __init__(self, project, level, id, parent_port=None):
#         super(AbstractGame, self).__init__(project, level, id, parent_port)

#         self.current_game = None

#     def run(self):
#         """ Set up things and wait for events """
#         self.executor_setup()
#         self.game_done.wait()
#         self.teardown()

#     def create_game(self, current_region, next_region, init_region):
#         # TODO: cleverly check for level
#         if self.level == 1:
#             local_game = LocalGame(self.proj, 0, current_region, self)
#             # TODO: How to properly set region?
#             local_game.set_init_region(init_region)
#             local_game.set_goal_region(next_region)
#         else:
#             # TODO: figure out what exactly to do
#             local_game = AbstractGame(self.proj, self.level - 1,
#                                       current_region, self)
#         return local_game

#     def handle_event(self, event_type, event_data):
#         if event_type == "STATE":
#             # If the state changes, stop the current game and create a new one
#             (current_region, next_region) = event_data
#             if current_region != next_region:
#                 # TODO: Assume some init region if there was no game before
#                 init_region = self.proj.init_region
#                 if self.current_game is not None:
#                     init_region = self.current_game.executor_proxy.get_current_region(
#                     )
#                     self.current_game.teardown()

#                 self.current_game = self.create_game(
#                     current_region, exit_helper(current_region,
#                                                 next_region), init_region)
#                 thr = threading.Thread(target=self.current_game.run)
#                 thr.daemon = True
#                 thr.start()
#         if event_type == "BORDER":
#             # TODO: trigger state evaluation/step
#             logging.info("Borders crossed in parent to:{}".format(event_data))
#             self.executor_proxy.set_arrived(True)

#     def teardown(self):
#         if self.current_game is not None:
#             self.current_game.teardown()
#         super(AbstractGame, self).teardown()


# HELPERS
def path_helper(path, name, level, id):
    return path + name + "_" + "_".join(str(x) for x in [level, id])


def layer_helper(level, *arg):
    """Convert a hierarchy level and a list of integers (ids of games) to a path.
    For example: (0, 1, 3) should return "0_1_3".
    """
    return level + "_" + "_".join(str(x) for x in arg)


def exit_helper(fromr, to):
    return "exit_" + "_".join([fromr, to])
