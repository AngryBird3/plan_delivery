"""
    This module is your primary workspace. Add whatever helper functions, classes, data structures, imports... etc here.

    We expect most results will utilize more than just dumping code into the plan_paths()
        function, that just serves as a meaningful entry point.

    In order for the rest of the scoring to work, you need to make sure you have correctly
        populated the DeliverySite.path for each result you produce.
"""
import typing
from nest_info import NestInfo, Coordinate, DeliverySite
from Astar import Astar
import numpy as np


class PathPlanner:
    def __init__(self, nest_info: NestInfo, delivery_sites: typing.List["DeliverySite"]):
        self.nest_info: NestInfo = nest_info
        self.delivery_sites: typing.List["DeliverySite"] = delivery_sites
        self.astar = Astar(nest_info.nest_coord, nest_info.maximum_range, nest_info.risk_zones)

    def plan_paths(self):
        """
        This is the function you should re-write. It is expected to mutate the list of
        delivery_sites by calling each DeliverySite's set_path() with the resulting
        path as an argument.

        The default construction shows this format, and should produce 10 invalid paths.
        """
        # path_coords = self.astar.find_path_to(self.delivery_sites[0].coord)
        # for p in path_coords:
        #     print(p, "  ", )
        # print("\n")

        for site in self.delivery_sites:
            print("[{}]".format(site.coord))
            path_coords = self.astar.find_path_to(site.coord)
            # Once you have a solution for the site - populate it like this:
            site.set_path(path_coords)
