import math
from typing import List, Optional, Tuple, Union
from cachetools import LRUCache
import numpy as np
import cv2
from topo.msg import Node


class Position:
    def __init__(self, x: Union[int, float], y: Union[int, float]):
        self.x = int(x)
        self.y = int(y)

    @staticmethod
    def of(t: Union[Tuple, np.ndarray]):
        return Position(t[0], t[1])

    @staticmethod
    def from_node(node: Node):
        return Position(node.x, node.y)

    def __sub__(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def __eq__(self, other: "Position"):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __str__(self):
        return f"({self.x}, {self.y})"

    def __repr__(self):
        return f"({self.x}, {self.y})"

    def __getitem__(self, index: int):
        if index == 0:
            return self.x
        elif index == 1:
            return self.y
        else:
            raise IndexError

    def __setitem__(self, index: int, value: int):
        if index == 0:
            self.x = value
        elif index == 1:
            self.y = value
        else:
            raise IndexError

    def __len__(self):
        return 2

    def __iter__(self):
        return iter([self.x, self.y])

    def __add__(self, other: "Position"):
        return Position(self.x + other.x, self.y + other.y)

    def __abs__(self):
        return Position(abs(self.x), abs(self.y))

    def __neg__(self):
        return Position(-self.x, -self.y)

    def __lt__(self, other: "Position"):
        return self.x < other.x and self.y < other.y

    def __le__(self, other: "Position"):
        return self.x <= other.x and self.y <= other.y

    def __gt__(self, other: "Position"):
        return self.x > other.x and self.y > other.y


class ConnectivityCache:
    def __init__(self, map, samples):
        self.map = map
        self.samples = samples
        self.samples_connectivity = LRUCache(maxsize=4 * 1024)
        self.points_connectivity = LRUCache(maxsize=6 * 1024)
        self.points_samples_connectivity = LRUCache(maxsize=8 * 1024)

    def update_map(self, map):
        self.map = map
        print(
            "已缓存连通性：",
            len(self.samples_connectivity),
            "组采样点，",
            len(self.points_connectivity),
            "组点，",
            len(self.points_samples_connectivity),
            "组点与采样点",
        )

    def update_samples(self, samples):
        self.samples = samples

    def samples_connectable(self, idx1, idx2) -> Optional[bool]:
        if idx1 > idx2:
            idx1, idx2 = idx2, idx1
        connectable = self.samples_connectivity.get((idx1, idx2), None)
        if connectable is not None:
            return connectable
        connectable = self.__check_connectivity(
            [self.samples[idx1].x, self.samples[idx1].y],
            [self.samples[idx2].x, self.samples[idx2].y],
        )
        if connectable is not None:
            self.samples_connectivity[(idx1, idx2)] = connectable
            self.points_connectivity[
                (
                    self.samples[idx1].x,
                    self.samples[idx1].y,
                    self.samples[idx2].x,
                    self.samples[idx2].y,
                )
            ] = connectable
            self.points_samples_connectivity[
                (self.samples[idx1].x, self.samples[idx1].y, idx2)
            ] = connectable
            self.points_samples_connectivity[
                (self.samples[idx2].x, self.samples[idx2].y, idx1)
            ] = connectable
        return connectable

    def points_connectable(self, point1, point2) -> Optional[bool]:
        x1, y1 = point1
        x2, y2 = point2
        if x1 > x2:
            x1, y1, x2, y2 = x2, y2, x1, y1
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        connectable = self.points_connectivity.get((x1, y1, x2, y2), None)
        if connectable is not None:
            return connectable
        connectable = self.__check_connectivity([x1, y1], [x2, y2])
        if connectable is not None:
            self.points_connectivity[(x1, y1, x2, y2)] = connectable
        return connectable

    def point_sample_connectable(self, point, sample_idx):
        x, y = point
        x, y = int(x), int(y)
        connectable = self.points_samples_connectivity.get((x, y, sample_idx), None)
        if connectable is not None:
            return connectable
        connectable = self.__check_connectivity(
            [x, y], [self.samples[sample_idx].x, self.samples[sample_idx].y]
        )
        if connectable is not None:
            self.points_samples_connectivity[(x, y, sample_idx)] = connectable
        return connectable

    def __check_connectivity(self, start_node, end_node):
        """
        检查两个节点是否连通
        """
        try:
            x1, y1 = int(start_node[1]), int(start_node[0])
            x2, y2 = int(end_node[1]), int(end_node[0])
            dx = x2 - x1
            dy = y2 - y1
            if dx == 0:
                for y in range(min(y1, y2), max(y1, y2)):
                    if self.map[x1][y] == 100:
                        return False
                    if self.map[x1][y] == -1:
                        return None
            elif dy == 0:
                for x in range(min(x1, x2), max(x1, x2)):
                    if self.map[x][y1] == 100:
                        return False
                    if self.map[x][y1] == -1:
                        return None
            else:
                k = dy / dx
                if abs(k) < 1:
                    b = y1 - k * x1
                    for x in range(min(x1, x2), max(x1, x2)):
                        y = int(k * x + b)
                        if self.map[x][y] == 100:
                            return False
                        if self.map[x][y] == -1:
                            return None
                else:
                    k = dx / dy
                    b = x1 - k * y1
                    for y in range(min(y1, y2), max(y1, y2)):
                        x = int(k * y + b)
                        if self.map[x][y] == 100:
                            return False
                        if self.map[x][y] == -1:
                            return None
            return True
        except Exception as e:
            print(str(e))
            return None

    # def node_sample_connectable(self, node, sample):


def check_connectivity(
    map_data, start_node: Position, end_node: Position, include_unknown=False
):
    """
    检查两个节点是否连通

    Parameters:
    -----------
    - map_data: np.ndarray
        地图数据，-1为未知，0为空闲，100为障碍物
    - start_node: Tuple[int, int]
        起始节点坐标
    - end_node: Tuple[int, int]
        终点节点坐标
    - include_unknown: bool
        是否包含未知区域，若为True，则未知区域也视为连通，默认为False
    """
    x1, y1 = start_node.y, start_node.x
    x2, y2 = end_node.y, end_node.x
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0:
        for y in range(min(y1, y2), max(y1, y2)):
            if map_data[x1][y] == 100:
                return False
            if map_data[x1][y] == -1 and not include_unknown:
                return False
    elif dy == 0:
        for x in range(min(x1, x2), max(x1, x2)):
            if map_data[x][y1] == 100:
                return False
            if map_data[x][y1] == -1 and not include_unknown:
                return False
    else:
        k = dy / dx
        if abs(k) < 1:
            b = y1 - k * x1
            for x in range(min(x1, x2), max(x1, x2)):
                y = int(k * x + b)
                if map_data[x][y] == 100:
                    return False
                if map_data[x][y] == -1 and not include_unknown:
                    return False
        else:
            k = dx / dy
            b = x1 - k * y1
            for y in range(min(y1, y2), max(y1, y2)):
                x = int(k * y + b)
                if map_data[x][y] == 100:
                    return False
                if map_data[x][y] == -1 and not include_unknown:
                    return False
    return True


def crop(map: np.ndarray, border=0):
    """
    自动裁剪地图

    Parameters:
    -----------
    - map: np.ndarray
        读取的地图，-1为未知，0为空闲，100为障碍物
    - border: int
        裁剪边框，默认为0

    Returns:
    --------
    - x0, y0, x1, y1: int
        裁剪后的地图边界
    """
    img = map.copy()
    img[img == -1] = 254
    img[img == 0] = 205
    img[img == 100] = 1
    img = img.astype(np.uint8)
    # 使用Canny边缘检测算法
    edges = cv2.Canny(img, 50, 150)
    # 寻找轮廓
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, 0, img.shape[1], img.shape[0]
    max_contour = np.concatenate(contours)
    # 确定对象的边框并进行裁剪
    x, y, w, h = cv2.boundingRect(max_contour)
    x0 = max(0, x - border)
    y0 = max(0, y - border)
    x1 = min(img.shape[1], x + w + border)
    y1 = min(img.shape[0], y + h + border)
    return x0, y0, x1, y1


def denoise(map: np.ndarray, old_map: np.ndarray):
    """
    对地图进行降噪处理。

    参数
    ----
    map: numpy数组，表示地图，-1为未知，0为空闲，100为障碍物
    old_map: numpy数组，表示上一次的地图

    返回值
    -----
    numpy数组，表示降噪处理后的地图图像。
    """
    radius = 2
    """降噪半径"""
    if old_map is None or old_map.shape != map.shape:
        changed_map = np.ones(map.shape)
    else:
        changed_map = map - old_map
    denoised_map = map.copy()
    left, top, right, bottom = crop(map)
    for i in range(top, bottom):
        for j in range(left, right):
            if map[i, j] == -1:
                if changed_map[i, j] != 0:
                    x1 = max(0, i - radius)
                    x2 = min(map.shape[0], i + radius)
                    y1 = max(0, j - radius)
                    y2 = min(map.shape[1], j + radius)
                    if np.sum(map[x1 : x2 + 1, y1 : y2 + 1] == 0) >= 3:
                        denoised_map[i, j] = 0
    return denoised_map


def find(condition):
    a = np.argwhere(condition)
    a1 = a.T
    a2 = np.lexsort(a1)
    a3 = a[a2].T
    return a3


def calculate_angle(pos1: Union[Position, Node], pos2: Union[Position, Node]):
    """
    计算两点之间的夹角

    Parameters
    ----------
    pos1 : Position | Node
        第一个点
    pos2 : Position | Node
        第二个点

    Returns
    -------
    out : float
        两点之间的夹角（单位：弧度）
    """
    # 计算两点之间的水平和垂直距离
    dx = pos2.x - pos1.x
    dy = pos2.y - pos1.y
    # 计算夹角弧度
    angle_radians = math.atan2(dy, dx)

    return angle_radians
