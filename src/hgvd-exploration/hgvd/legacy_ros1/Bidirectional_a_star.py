"""
Bidirectional_a_star 2D
@author: huiming zhou
"""

import math
import heapq
from typing import Tuple
from utils import Position
import numpy as np


class BidirectionalAStar:
    """
    双向A*算法
    """

    def __init__(self, map: np.ndarray):  # , info, test_pub
        """
        双向A*算法

        Parameters:
        -----------

        map: np.ndarray
            地图，-1为未知，0为空闲，100为障碍物
        """
        self.u_set = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]  # feasible input set
        self.map = map.copy()  # position of obstacles
        """地图，-1为未知，0为空闲，100为障碍物"""
        # 降低地图分辨率
        self.map = self.map[::2, ::2]

    def __init_search(
        self,
        s_start: Position,
        s_goal: Position,
    ):
        """
        初始化搜索参数
        """
        self.s_start = (s_start.x//2, s_start.y//2)
        self.s_goal = (s_goal.x//2, s_goal.y//2)

        self.OPEN_fore = []  # OPEN set for forward searching
        self.OPEN_back = []  # OPEN set for backward searching
        self.CLOSED_fore = []  # CLOSED set for forward
        self.CLOSED_back = []  # CLOSED set for backward
        self.PARENT_fore = dict()  # recorded parent for forward
        self.PARENT_back = dict()  # recorded parent for backward
        self.g_fore = dict()  # cost to come for forward
        self.g_back = dict()  # cost to come for backward

        self.g_fore[self.s_start] = 0.0
        self.g_fore[self.s_goal] = math.inf
        self.g_back[self.s_goal] = 0.0
        self.g_back[self.s_start] = math.inf
        self.PARENT_fore[self.s_start] = self.s_start
        self.PARENT_back[self.s_goal] = self.s_goal
        heapq.heappush(
            self.OPEN_fore, (self.__f_value_fore(self.s_start), self.s_start)
        )
        heapq.heappush(self.OPEN_back, (self.__f_value_back(self.s_goal), self.s_goal))

    def search(
        self,
        s_start: Position,
        s_goal: Position,
    ):
        """
        搜索

        Parameters:
        -----------
        s_start: Position
            起始点
        s_goal: Position
            终点

        Returns:
        --------
        path: List[Position]
            路径
        """
        
        self.__init_search(s_start, s_goal)
        s_meet = self.s_start

        while self.OPEN_fore and self.OPEN_back:
            # solve foreward-search
            _, s_fore = heapq.heappop(self.OPEN_fore)

            if s_fore in self.PARENT_back:
                s_meet = s_fore
                break

            self.CLOSED_fore.append(s_fore)

            for s_n in self.__get_neighbor(s_fore):
                new_cost = self.g_fore[s_fore] + self.__cost(s_fore, s_n)

                if s_n not in self.g_fore:
                    self.g_fore[s_n] = math.inf

                if new_cost < self.g_fore[s_n]:
                    self.g_fore[s_n] = new_cost
                    self.PARENT_fore[s_n] = s_fore
                    heapq.heappush(self.OPEN_fore, (self.__f_value_fore(s_n), s_n))

            # solve backward-search
            _, s_back = heapq.heappop(self.OPEN_back)

            if s_back in self.PARENT_fore:
                s_meet = s_back
                break

            self.CLOSED_back.append(s_back)

            for s_n in self.__get_neighbor(s_back):
                new_cost = self.g_back[s_back] + self.__cost(s_back, s_n)

                if s_n not in self.g_back:
                    self.g_back[s_n] = math.inf

                if new_cost < self.g_back[s_n]:
                    self.g_back[s_n] = new_cost
                    self.PARENT_back[s_n] = s_back
                    heapq.heappush(self.OPEN_back, (self.__f_value_back(s_n), s_n))

        return self.__extract_path(s_meet)

    def __get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def __extract_path(self, s_meet: Tuple[int, int]):
        """
        extract path from start and goal
        :param s_meet: meet point of bi-direction a*
        :return: path
        """

        # extract path for foreward part
        path_fore = [Position.of(s_meet)]
        s = s_meet

        while True:
            s = self.PARENT_fore[s]
            path_fore.append(Position.of(s))
            if s == self.s_start:
                break

        # extract path for backward part
        path_back = []
        s = s_meet

        while True:
            if s not in self.PARENT_back:
                # np.save("map.npy", self.map)
                # np.save("points.npy", [self.s_start, self.s_goal])
                # breakpoint()
                return math.inf
            s = self.PARENT_back[s]
            path_back.append(Position.of(s))
            if s == self.s_goal:
                break

        return len(path_fore) + len(path_back)

    def __f_value_fore(self, s):
        """
        forward searching: f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_fore[s] + self.__h(s, self.s_goal)

    def __f_value_back(self, s):
        """
        backward searching: f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_back[s] + self.__h(s, self.s_start)

    def __h(self, s, goal):
        """
        Calculate heuristic value.
        :param s: current node (state)
        :param goal: goal node (state)
        :return: heuristic value
        """
        return math.hypot(goal[0] - s[0], goal[1] - s[1])

    def __cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.__is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def __is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.

        Parameters:
        -----------
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """
        if s_start[0] >= self.map.shape[0] or s_start[1] >= self.map.shape[1] or s_start[0] < 0 or s_start[1] < 0:
            print("start point out of map:", s_start, self.map.shape)
            return False
        if s_end[0] >= self.map.shape[0] or s_end[1] >= self.map.shape[1] or s_end[0] < 0 or s_end[1] < 0:
            print("end point out of map:", s_end, self.map.shape)
            return False
        if self.map[s_start[0], s_start[1]] == 100 or self.map[s_end[0], s_end[1]] == 100:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if self.map[s1[0], s1[1]] == 100 or self.map[s2[0], s2[1]] == 100:
                return True

        return False
