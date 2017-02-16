import sys
import lib.globalConfig
from lib.specCompiler import SpecCompiler

def main():
    print("Done")

if __name__=="__main__":
    if len(sys.argv) > 1:
        path = sys.argv[1]
        compiler = SpecCompiler(path)
        (synth, b, msg) = compiler.compile()
        print msg
    else:
        print "No path was given"
        sys.exit(1)
