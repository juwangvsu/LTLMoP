import sys, os
import ConfigParser
import argparse
import lib.regions as regions
import lib.project as project

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
        rfi = regions.RegionFileInterface()
        if not rfi.readFile(region_path):
            print("Could not read region file")
            sys.exit(-1)

        for region in rfi.regions:
            section = region.name if config.has_section(
                region.name) else DEFAULT
            dimensions = map(int, config.get(section, CFG_DIM).split(','))
            regs = config.get(section, CFG_REGS).split(',')
            level = config.get(section, CFG_LEV)
            self.create_game(region.name, level, dimensions[0], dimensions[1],
                             regs)

    def add_region(self):
        pass

    def create_game(self, region, level, rows, cols, regs):
        # TODO Create .spec
        # TODO Create .regions
        # e.g. Flat.0.Kitchen.spec
        spec_path = "%s.%s.%s.spec" % (self.name, level, region)
        regions_path = "%s.%s.%s.regions" % (self.name, level, region)

        print("Creating game for region %s on level %s (%d x %d)" %
              (region, level, rows, cols))

        # if not os.path.exists(spec_path):
        proj = project.Project()
        proj.compile_options["decompose"] = False
        proj.writeSpecFile(spec_path)
        # proj.loadRegionFile() # doesn't know path

        # if not os.path.exists(regions_path):
        rfi = regions.RegionFileInterface()

        # Create regions
        for i in rows:
            for j in cols:
                regions.Region()

        rfi.writeFile(regions_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Create subgames of a hierarchical game.')
    parser.add_argument(
        '-cfg',
        dest='config_file',
        help='.cfg file that defines the lower levels')

    args = parser.parse_args()

    GameCreator(args.config_file)
