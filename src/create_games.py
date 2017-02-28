import sys, os
import ConfigParser
import argparse
from lib.regions import Region, RegionFileInterface, Point, Size, reg_RECT, pointLineIntersection
import lib.project as project
from math import floor

DEFAULT = 'DEFAULT'
CFG_REGS = 'Regions'
CFG_DIM = 'Dimensions'
CFG_SPEC = 'Spec_path'
CFG_REG = 'Region_path'
CFG_LEV = 'Level'
CFG_NAME = 'Name'

class GameCreator():
    def __init__(self, config_file):
        config = ConfigParser.SafeConfigParser()
        config.read(config_file)
        spec_path = config.get(DEFAULT, CFG_SPEC)
        region_path = config.get(DEFAULT, CFG_REG)
        self.name = config.get(DEFAULT, CFG_NAME)

        # Get regions from regions_file
        self.top_rfi = RegionFileInterface()
        if not self.top_rfi.readFile(region_path):
            print("Could not read region file")
            sys.exit(-1)

        for region in self.top_rfi.regions:
            section = region.name if config.has_section(
                region.name) else DEFAULT
            dimensions = map(int, config.get(section, CFG_DIM).split(','))
            regs = config.get(section, CFG_REGS).split(',')
            level = config.get(section, CFG_LEV)
            self.create_game(region, level, dimensions[0], dimensions[1],
                             regs)

    def create_game(self, region, level, rows, cols, regs):
        """
        Creates a game by creating the spec and region files as well as filling the regions with a grid and exits.

        :param region: region the game corresponds to
        :param level: level of the current layer
        :param rows: number of rows the grid has
        :param cols: number of columns the grid has
        :param regs: list of the region names the grid should use
        :return:
        """
        # e.g. Flat.0.Kitchen.spec
        spec_path = "%s.%s.%s.spec" % (self.name, level, region.name)
        regions_path = "%s.%s.%s.regions" % (self.name, level, region.name)

        # check if the file exists, so the manual changes won't be overwritten
        if not os.path.exists(spec_path) and not os.path.exists(regions_path):
            print("Creating game for region %s on level %s (%d x %d)" %
                  (region.name, level, rows, cols))

            proj = project.Project()
            proj.compile_options["decompose"] = False

            rfi = RegionFileInterface()

            # Origin and sizes for the grid
            orig_x = region.position.x
            orig_y = region.position.y
            width = region.size.GetWidth()/cols
            height = region.size.GetHeight()/rows
            size = Size(width, height)

            # Create regions
            regs.reverse()
            for i in range(0, rows):
                for j in range(0, cols):
                    n = regs.pop()
                    reg = Region(name=n, type=reg_RECT, position=Point(orig_x + j * width, orig_y + i * height), size=size)
                    rfi.regions.append(reg)

            self.create_exits(cols, height, region, rfi, rows, size, width)

            rfi.recalcAdjacency()
            rfi.writeFile(regions_path)
            proj.rfi = rfi
            proj.writeSpecFile(spec_path)
        else:
            print("Skipping game for region %s" % region.name)

    def create_exits(self, cols, height, region, rfi, rows, size, width):
        """
        Creates the exits by searching through the transitions of the higher game.

        :param cols: number of columns the game has
        :param height: height of a grid cell
        :param region: the region this game corresponds to
        :param rfi: region file interface of the game
        :param rows: number of rows the game has
        :param size: size the grid cell has
        :param width: width of a grid cell
        :return:
        """
        index = self.top_rfi.indexOfRegionWithName(region.name)
        for (i, trans) in enumerate(self.top_rfi.transitions[index]):
            # check if there is a adjacent region
            if trans:
                # line between the two regions from p1 to p2
                # FIXME: will fail on Polygons?
                p1, p2 = trans[0]
                adj_region = self.top_rfi.regions[i]

                # check if we need to place it above/beneath
                if p1.x == p2.x:
                    pos_y = min(p1.y, p2.y) + floor(rows / 2) * height
                    if region.position.x > adj_region.position.x:
                        pos_x = p1.x - width
                    else:
                        pos_x = p1.x
                # check if we need to place it to the right/left
                else:
                    pos_x = min(p1.x, p2.x) + floor(cols / 2) * width
                    if region.position.y > adj_region.position.y:
                        pos_y = p1.y - height
                    else:
                        pos_y = p1.y

                # FIXME: adding a point doesn't make it a transition
                self.add_point(pos_x, pos_y, rfi)
                self.add_point(pos_x+width, pos_y+height, rfi)
                self.add_point(pos_x, pos_y+height, rfi)
                self.add_point(pos_x+width, pos_y, rfi)

                reg = Region(name=exit_name(0, region.name, adj_region.name),
                             type=reg_RECT,
                             position=Point(pos_x, pos_y),
                             size=size)
                rfi.regions.append(reg)

    def add_point(self, pos_x, pos_y, rfi):
        """
        Tries to find a region where the point pos_x, pos_y is on a face and if so, add the point to that region

        :param pos_x: x coordinate of the point to be added
        :param pos_y: y coordinate of the point to be added
        :param rfi: the region file interface of the game, to iterate over the regions
        :return: None
        """
        # find region we could add a point to
        for reg in rfi.regions:
            if reg.objectContainsPoint(pos_x, pos_y):
                # Figure out where we would add a new point
                points = [x for x in reg.getPoints()]

                for pidx in range(len(points)):
                    if pidx > 0:
                        p1 = points[pidx - 1]
                        p2 = points[pidx]
                        padd = Point(pos_x, pos_y)

                        (on_seg, d, p) = pointLineIntersection(p1, p2, padd)
                        # If the distance between the point and the line is zero, add the point to the region
                        # don't add, if it is a corner point
                        if d == 0.0 and p1 != padd and p2 != padd:
                            reg.addPoint(Point(pos_x - reg.position.x, pos_y - reg.position.y), pidx)
                            break


def exit_name(level, fr, to):
    """
    Formats the string for an exit name

    :param level: level the exit corresponds to
    :param fr: name of the `from` region
    :param to: name of the `to` region
    :return: string with the exit name
    """
    return "exit_%d_%s_%s" % (level, fr, to)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Create subgames of a hierarchical game.')
    parser.add_argument(
        '-cfg',
        dest='config_file',
        help='.cfg file that defines the lower levels')

    args = parser.parse_args()
    GameCreator(args.config_file)
