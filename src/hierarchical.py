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


class Hierarchical(object):
    def __init__(self, name, path, layers):
        self.path = path
        self.games = []
        self.initial_pose = 0
        self.name = name
        self.layers = layers

    def set_initial_pose(self):
        self.initial_pose = 0


class LocalGame(object):
    def __init__(self, project, level, id):
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

    def synthesize(self):
        compiler = SpecCompiler(self.spec_path)
        if compiler.compile() is not None:
            self.dirty = False
        else:
            logging.error("Compilation went wrong")
            sys.exit(1)

    def set_init_region(self, region):
        """ Set the init region by replacing it in the specification file """
        # TODO: Do it properly (i.e. not via fiddling with the file)

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
        return

    def set_goal_region(self, region):
        # TODO: don't do it by fiddling with files
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
        if self.dirty:
            self.synthesize()

        executorInitialised = threading.Event()
        while True:
            listen_port = random.randint(10000, 65535)
            try:
                serv = SimpleXMLRPCServer(
                    ("127.0.0.1", listen_port),
                    logRequests=False,
                    allow_none=True)
            except socket.error as e:
                pass
            else:
                break

        serv.register_function(self.executor_ready)
        serv.register_function(self.handle_event)
        XMLRPCServerThread = threading.Thread(target=serv.serve_forever)
        XMLRPCServerThread.daemon = True
        XMLRPCServerThread.start()

        exec_thread = threading.Thread(
            target=execute_main,
            args=[None, self.spec_path, self.strat_path, True, listen_port])
        exec_thread.start()

        logging.info("Waiting for the executor to be ready")
        self.executor_ready_event.wait()

        executorProxy = xmlrpclib.ServerProxy(
            "http://127.0.0.1:{}".format(self.executor_port), allow_none=True)
        executorProxy.registerHierarchicalEventTarget(
            "http://127.0.0.1:{}".format(listen_port))

        self.game_done.wait()

        # Clean up on exit
        executorProxy.shutdown()
        logging.info("Waiting for XML-RPC server to shut down...")
        serv.shutdown()
        XMLRPCServerThread.join()
        exec_thread.join()
        logging.info("XML-RPC server shutdown complete.  Goodbye.")

    def find_current_config(self):
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
        self.executor_ready_event.set()
        self.executor_port = port

    def handle_event(self, eventType, eventData):
        if eventType == "BORDER":
            self.current_region = eventData
            if self.goal_region == eventData:
                logging.info("You have reached your goal")
                self.game_done.set()


def path_helper(path, name, level, id):
    return path + name + "_" + "_".join(str(x) for x in [level, id])


def main():
    hierarch = Hierarchical("Test", "/Users/adrian/Desktop/Test/", 2)
    game0 = LocalGame(hierarch, 0, 1)
    game0.set_init_region("11")
    game0.set_goal_region("exit_2")
    game0.run()

    game1 = LocalGame(hierarch, 0, 2)
    game1.set_init_region("exit_1")
    game1.set_goal_region("22")
    game1.run()

if __name__ == "__main__":
    main()
