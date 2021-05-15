"""
This file implements Astar for finding path between nest to deliver site
"""
import math

import numpy
from nest_info import Coordinate


class Node:
    """
    For Astar we're using this class to calculate to store node as we visit them into our open/close list
    """

    def __init__(self, coord: Coordinate, parent):
        self.coord = coord
        self.parent = parent
        self.g_score = 0
        self.h_score = 0
        self.length = 0

    # This is not only calculating F score but also considering length
    def get_score(self):
        return self.g_score + self.h_score

    def prettyPrint(self):
        print(
            "coord: {}, g_score: {}, h_score: {}, length: {}".format(self.coord, self.g_score, self.h_score,
                                                                     self.length))


class Astar:
    """
    This is our typical Astar algorithm implemented in only public method: find_path_to
    """
    def __init__(self, nest_coord: Coordinate, max_range: int, risk_map: numpy.ndarray):
        self.start = Node(nest_coord, None)
        self.grid = risk_map
        # Direction can be 8 or 4, but for now I'm hard-coding it to 8
        self.direction = [[0, 1], [1, 0], [0, -1], [-1, 0], [-1, -1], [1, 1], [-1, 1], [1, -1]]
        """
        We want to weight cost as well as dist and to do that I'm using some score here
        I'm giving slightly higher weight to length of the path
        Look at g_score
        """
        self.l_weight = 0.4
        self.max_range = max_range
        self.grid_max_x, self.grid_max_y = self.grid.shape

    def __find_node_with_least_Fscore(self, open_list: list[Node]) -> (int, Node):
        cur = open_list[0]
        index = 0

        for i, n in enumerate(open_list):
            if n.get_score() < cur.get_score():
                cur = n
                index = i

        return index, cur

    def __find_node_in_list(self, open_list: list[Node], neighborXY: list[int]) -> Node:
        node = None
        for n in open_list:
            if n.coord == Coordinate(neighborXY[0], neighborXY[1]):
                return n
        return node

    def __safe(self, neighborXY: list[int, int]) -> bool:
        x, y = neighborXY
        # check if it is outside of our risk map
        if x >= self.grid_max_x or y >= self.grid_max_y or x < 0 or y < 0:
            return False
        # check if it is KEEP_OUT zone
        if self.grid[x][y] == 2:
            return False
        return True

    def __build_path(self, current: Node) -> list[Coordinate]:
        path = list()
        while current:
            path.insert(0, current.coord)
            current = current.parent

        return path

    def __heuristic(self, neighborXY: list[int], dest: list[int], risk: int) -> float:
        """
        Here we're using euclidean cause that's what we used at g_score (for distance)
        """
        x_delta = abs(neighborXY[0] - dest[0])
        y_delta = abs(neighborXY[1] - dest[1])
        euclidean = self.l_weight * (math.sqrt(pow(x_delta, 2) + pow(y_delta, 2))) + (1 - self.l_weight) * risk
        return euclidean

    def euclidean(self, p1: Coordinate, p2: Coordinate):
        x_delta = abs(p1.e - p2.e)
        y_delta = abs(p1.n - p2.n)
        return math.sqrt(pow(x_delta, 2) + pow(y_delta, 2))

    def find_path_to(self, dest: Coordinate):
        open = list()  # list of Node in open
        closed = list()  #

        open.append(self.start)  # always start from nest
        while len(open) > 0:
            # get node with least F score
            index, current = self.__find_node_with_least_Fscore(open)
            # if it is the destination then hooray!
            if current.coord == dest:
                return self.__build_path(current)

            # remove cur from open list
            closed.append(current)
            open.pop(index)

            # now add 8 neighbors to our open set IF its total cost is less with this path
            for d in self.direction:
                neighborXY = [current.coord.e + d[0], current.coord.n + d[1]]
                # check if this wasn't visited before and it's safe
                if not self.__safe(neighborXY) or self.__find_node_in_list(closed, neighborXY):
                    continue
                dist = current.length + self.euclidean(current.coord, Coordinate(neighborXY[0], neighborXY[1]))
                total_cost = dist * self.l_weight + (1 - self.l_weight) * self.grid[neighborXY[0]][neighborXY[1]]

                # no can't go out side of max_range
                if dist > self.max_range:
                    continue

                # let's see if this node is already in open list
                next = self.__find_node_in_list(open, neighborXY)
                if next:
                    if next.g_score > total_cost:
                        # update node
                        next.g_score = total_cost
                        next.length = dist
                        next.parent = current
                        # h_score doesn't change!
                else:
                    next = Node(Coordinate(neighborXY[0], neighborXY[1]), current)
                    next.g_score = total_cost
                    next.h_score = self.__heuristic(neighborXY, [dest.e, dest.n],
                                                    self.grid[neighborXY[0]][neighborXY[1]])
                    next.length = dist
                    open.append(next)

        # we shouldn't be here
        raise ValueError("No path found")
