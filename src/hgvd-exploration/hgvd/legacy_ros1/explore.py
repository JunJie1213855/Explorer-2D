#!/usr/bin/env python3

from functools import lru_cache
import os
from typing import List, Tuple, Union

import cv2
from sklearn.neighbors import NearestNeighbors
import rospy
from mean_shift3 import mean_shift
from utils import ConnectivityCache, Position, calculate_angle
from topo.msg import Node, NodeArray
from copy import copy
import tf2_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import networkx as nx
from math import sqrt
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from Bidirectional_a_star import BidirectionalAStar
from utils import check_connectivity
from sklearn.neighbors import NearestNeighbors

# MS_BANDWIDTH = 50
# MS_BANDWIDTH = 60
MS_BANDWIDTH = 100
TOPO_NEIGHBORS = 10
"""拓扑邻居数，连接子拓扑图时计算的邻居数"""
THRESHOLD = 80
"""目标切换阈值"""


def info(*kwargs):
    """带文件名的loginfo"""
    rospy.loginfo(
        "[{}]: {}".format(os.path.basename(__file__), " ".join(list(map(str, kwargs))))
    )


def warn(*kwargs):
    """带文件名的logwarn"""
    rospy.logwarn(
        "[{}]: {}".format(os.path.basename(__file__), " ".join(list(map(str, kwargs))))
    )


def get_nodes_marker():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "nodes"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.color.a = 1.0
    marker.color.g = 0.0
    marker.color.r = 0.0
    marker.color.b = 1.0
    marker.lifetime = rospy.Duration()
    return marker


class ExploreNode:
    def __init__(self) -> None:
        rospy.init_node("explore", anonymous=False)
        info("探索节点启动")
        self.map = None
        """地图，-1为未知，0为空闲，100为障碍物"""
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

        self.samples: List[Node] = []
        self.samples_sub = rospy.Subscriber(
            "samples", NodeArray, self.samples_callback, queue_size=1
        )
        self.G = nx.Graph()
        self.samples_topo = nx.Graph()
        self.labels = []

        self.connectivity_cache = ConnectivityCache(self.map, self.samples)

        self.path_length = {}

        # MoveBase
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.current_goal = None
        self.move_base_goal = MoveBaseGoal()
        self.move_base_goal.target_pose.header.frame_id = "map"
        self.move_base_goal.target_pose.header.stamp = rospy.Time.now()
        self.explored_pub = rospy.Publisher("nodes_explored", Marker, queue_size=10)
        self.unexplored_pub = rospy.Publisher("nodes_unexplored", Marker, queue_size=10)

        self.nodes_pub = rospy.Publisher("topo_nodes", Marker, queue_size=10)
        self.nodes_marker = get_nodes_marker()
        self.edges_pub = rospy.Publisher("topo_edges", Marker, queue_size=10)
        self.edges_pub2 = rospy.Publisher("topo_edges2", Marker, queue_size=10)
        self.test_pub = rospy.Publisher("test", Marker, queue_size=10)
        self.clusters_pub = rospy.Publisher("clusters", Marker, queue_size=10)
        self.topo_distance_pub = rospy.Publisher(
            "topo_distance", MarkerArray, queue_size=10
        )

    def run(self):
        rate = rospy.Rate(1)
        # 等待地图
        while self.map is None:
            info("等待地图……")
            rate.sleep()
        while len(self.G) == 0:
            info("等待节点……")
            rate.sleep()
        while self.client.wait_for_server() == False:
            info("等待move_base服务器……")
            rate.sleep(1)
        self.threshold = 10
        while not rospy.is_shutdown():
            G = self.G.copy()
            nodes = self.samples.copy()
            labels = self.labels.copy()
            current_pos, yaw = self.current_pos_yaw()
            # 首先更新各节点探索状态
            for i in range(len(nodes)):
                node = nodes[i]
                if node.explored:
                    continue
                distance = self.get_distance(current_pos, node)
                if distance < self.threshold or check_explored(node, self.map):
                    node.explored = True
                    # info(f"节点({node.x}, {node.y})已探索")
                self.display_samples(nodes)
            # 选择最近的未探索节点
            nearest, min_distance = self.get_nearest(G, nodes, labels, current_pos, yaw)
            try:
                if nodes[nearest].explored:
                    info("暂无边界点")
                    self.display_samples(nodes)
                    rate.sleep()
                    continue
            except:
                continue
            state = self.client.get_state()
            # info("当前状态：", state)
            # 如果当前目标点已丢失或未开始，则直接探索最近点
            if state != GoalStatus.ACTIVE or self.current_goal is None:
                self.current_goal = nearest
                self.send_goal(nodes[self.current_goal])
                rospy.sleep(1)
                continue
            else:
                if nodes[self.current_goal].explored:
                    # info("目标点已探索")
                    self.current_goal = None
                    continue
                current_goal_distance = self.topo_distance(
                    nodes[self.current_goal], G, nodes, labels, current_pos, yaw
                )[0]
                # info(
                #     "当前目标点与最近点距离：",
                #     # current_goal_distance,
                #     # min_distance,
                #     current_goal_distance - min_distance,
                # )
                if current_goal_distance - min_distance > THRESHOLD:
                    self.current_goal = nearest
                    self.send_goal(nodes[self.current_goal])

            rospy.sleep(0.2)
            # info(
            #     "未探索："
            #     + "".join(
            #         [f"({node.x}, {node.y})" for node in nodes if not node.explored]
            #     )
            # )

    def real_pos_from_pixel(self, x, y):
        """
        将地图坐标转换为真实坐标，用于导航
        """
        return (
            x * self.info.resolution + self.info.origin.position.x,
            y * self.info.resolution + self.info.origin.position.y,
        )

    def send_goal(self, target: Node):
        """
        发送探索目标点

        Parameters
        ----------
        target : Node
            目标节点
        """
        # info(f"前往({x}, {y})")
        x, y = self.real_pos_from_pixel(target.x, target.y)
        self.move_base_goal.target_pose.pose.position.x = x
        self.move_base_goal.target_pose.pose.position.y = y
        self.move_base_goal.target_pose.pose.orientation.x = 0.0
        self.move_base_goal.target_pose.pose.orientation.y = 0.0
        self.move_base_goal.target_pose.pose.orientation.z = 0.0
        self.move_base_goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.move_base_goal)

    # TODO
    def topo_distance(
        self,
        target_node: Node,
        G: nx.Graph,
        nodes: List[Node],
        labels: List[int],
        robot_pos: Position,
        yaw: float,
    ):
        """
        计算机器人当前拓扑节点到目标点的拓扑距离

        Parameters:
        -----------
        target_node : Node
            目标节点
        G : nx.Graph
            拓扑图
        nodes : List[Node]
            节点列表
        labels : List[int]
            节点所属拓扑节点
        robot_pos : Position
            机器人当前位置
        yaw : float
            机器人航向
        """
        if check_connectivity(self.map, robot_pos, target_node):
            # 直线距离
            distance = self.get_distance(robot_pos, target_node)
            angle = calculate_angle(robot_pos, target_node) - yaw
            if angle > math.pi:
                angle -= 2 * math.pi
            elif angle < -math.pi:
                angle += 2 * math.pi
            distance += 0.1 * distance * abs(angle)
            return [distance, 0, 0, 0, 0]

        min_node_distance = float("inf")
        for tmp_node in nodes:
            node_distance = self.get_distance(tmp_node, robot_pos)
            if node_distance < min_node_distance:
                min_node_distance = node_distance
                curr_node_idx = labels[tmp_node.index]
        distance = 0

        # 拓扑距离，即经过的拓扑路径的长度
        has_path = nx.has_path(G, curr_node_idx, labels[target_node.index])
        if has_path:
            topo_distance = 0
            path = nx.shortest_path(
                G, source=curr_node_idx, target=labels[target_node.index]
            )
            for i in range(len(path) - 1):
                topo_distance += self.get_distance(
                    Position.of(G.nodes[path[i]]["pos"]),
                    Position.of(G.nodes[path[i + 1]]["pos"]),
                )
        else:
            path = [0] * 3
            topo_distance = 2 * self.get_distance(
                Position.of(G.nodes[curr_node_idx]["pos"]),
                Position.of(G.nodes[labels[target_node.index]]["pos"]),
            )
        distance += topo_distance

        # 末端距离（目标点到所属拓扑节点的距离）
        end_distance = self.get_distance(
            Position.of(G.nodes[labels[target_node.index]]["pos"]),
            target_node,
        )
        distance += end_distance

        # 0.1*拓扑路径大小*拓扑距离
        distance += 0.1 * len(path) * topo_distance

        # 0.5*末端距离*角度
        angle = calculate_angle(robot_pos, target_node) - yaw
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        distance += 1.0 * end_distance * abs(angle)

        """
        距离
        拓扑距离
        末端距离
        0.1*拓扑路径大小*拓扑距离
        0.5*末端距离*角度
        """
        return [
            distance,  # 距离
            topo_distance,  # 拓扑距离
            end_distance,  # 末端距离
            0.1 * len(path) * topo_distance,  # 0.1*拓扑路径大小*拓扑距离
            1.0 * end_distance * abs(angle),  # 0.5*末端距离*角度
        ]

    def get_nearest(
        self,
        G: nx.Graph,
        nodes: List[Node],
        labels: List[int],
        robot_pos: Position,
        yaw: float,
    ):
        """
        获取最近的点的索引
        
        Parameters
        ----------
        G : nx.Graph
            拓扑图
        nodes : List[Node]
            节点列表
        labels : List[int]
            节点所属拓扑节点
        robot_pos : Position
            机器人当前位置
        yaw : float
            机器人航向

        Returns
        -------
        out : Tuple[int, float]
            最近的点的索引和距离
        """
        nearest = 0
        min_distance = float("inf")
        marker_array = MarkerArray()
        for node in nodes:
            if not node.explored:
                distances = self.topo_distance(node, G, nodes, labels, robot_pos, yaw)
                if distances[0] < min_distance:
                    min_distance = distances[0]
                    nearest = node.index
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "topo_distance"
                marker.id = node.index
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = (
                    node.x * self.info.resolution + self.info.origin.position.x
                )
                marker.pose.position.y = (
                    node.y * self.info.resolution + self.info.origin.position.y + 0.2
                )
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1.0
                # 缩放
                marker.scale.z = 0.4
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.text = "|".join(map(lambda x: str(int(x)), distances))
                marker_array.markers.append(marker)
        self.topo_distance_pub.publish(marker_array)
        # info(
        #     f"距离{int(distance)},拓扑{int(topo_distance)},末端{int(node_distance)}角度{angle}"
        # )
        return nearest, min_distance

    def current_position(self):
        """
        获取机器人在地图中的像素坐标

        Returns
        -------
        out: Position
            机器人在地图中的位置
        """
        # 指定目标坐标系和源坐标系
        target_frame = "/map"  # 目标坐标系
        source_frame = "/base_link"  # 机器人本体坐标系
        cond = 0
        while cond == 0:
            try:
                trans = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rospy.Time()
                )
                position = trans.transform.translation
                # orientation = trans.transform.rotation
                cond = 1
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                cond == 0
        return Position(
            (position.x - self.info.origin.position.x) / self.info.resolution,
            (position.y - self.info.origin.position.y) / self.info.resolution,
        )

    def current_pos_yaw(self) -> Tuple[Position, float]:
        """
        获取机器人在地图中的位置和航向

        Returns
        -------
        out: Tuple[Position, float]
            机器人在地图中的位置和航向
        """
        # 指定目标坐标系和源坐标系
        target_frame = "map"  # 目标坐标系
        source_frame = "base_link"  # 机器人本体坐标系
        cond = 0
        while cond == 0:
            try:
                trans = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rospy.Time()
                )
                position = trans.transform.translation
                orientation = trans.transform.rotation
                cond = 1
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                cond == 0
        # 从四元数中提取航向
        yaw = euler_from_quaternion(
            [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        )[2]
        # info(f"当前航向：{yaw}")
        return (
            Position(
                int((position.x - self.info.origin.position.x) / self.info.resolution),
                int((position.y - self.info.origin.position.y) / self.info.resolution),
            ),
            yaw,
        )

    def samples_callback(self, msg: NodeArray):
        info(f"采样点更新：{len(msg.nodes)}")
        if len(msg.nodes) == 0:
            return
        if len(msg.nodes) == len(self.samples):
            nodes_pos = np.array([self.G.nodes[node]["pos"] for node in self.G.nodes])
            self.display_topo_nodes(nodes_pos)
            self.display_topo_edges(self.G)
        
        start_time = rospy.Time.now()
        samples = self.samples.copy()
        self.display_samples(samples)

        for i in range(len(samples), len(msg.nodes)):
            samples.append(msg.nodes[i])
            self.samples_topo.add_node(i, pos=(msg.nodes[i].x, msg.nodes[i].y))
        info("开始更新拓扑图")
        self.connectivity_cache.update_samples(samples)
        # 更新连通性
        for i in range(len(samples)):
            for j in range(i + 1, len(samples)):
                if self.connectivity_cache.samples_connectable(i, j):
                    self.samples_topo.add_edge(
                        i,
                        j,
                        length=self.get_distance(
                            samples[i],
                            samples[j],
                        ),
                    )
        info("采样点连通性", (rospy.Time.now() - start_time).to_sec())
        if len(samples) == 0:
            return
        G = nx.Graph()
        samples_pos = np.array([Position(sample.x, sample.y) for sample in samples])
        nodes_pos, labels = mean_shift(
            X=samples_pos,
            map=self.map,
            connectivity_cache=self.connectivity_cache,
            bandwidth=MS_BANDWIDTH,
            n_jobs=4,
        )
        info("聚类", (rospy.Time.now() - start_time).to_sec())
        self.display_clusters(nodes_pos, labels, samples)
        samples_topo = self.samples_topo.copy()
        # 添加聚类中心
        for i in range(len(nodes_pos)):
            samples_topo.add_node(
                i + len(samples), pos=(nodes_pos[i][0], nodes_pos[i][1])
            )
        # 添加边
        for i in range(len(nodes_pos)):
            for j in range(len(samples)):
                if self.connectivity_cache.point_sample_connectable(
                    nodes_pos[i],
                    j,
                ):
                    samples_topo.add_edge(
                        i + len(samples),
                        j,
                        length=self.get_distance(
                            Position.of(nodes_pos[i]),
                            samples[j],
                        ),
                    )
        # 生成拓扑图
        for i in range(len(nodes_pos)):
            G.add_node(i, pos=(nodes_pos[i][0], nodes_pos[i][1]))
        for i in G.nodes:
            for j in range(i + 1, len(G.nodes)):
                if self.connectivity_cache.points_connectable(
                    G.nodes[i]["pos"], G.nodes[j]["pos"]
                ):
                    G.add_edge(i, j)
        self.display_topo_nodes(nodes_pos)
        self.G = G
        self.update_samples(samples)
        self.labels = labels
        info("连通性耗时", (rospy.Time.now() - start_time).to_sec())
        #####

        bastar = None
        while True:
            min_sub_graph = min(nx.connected_components(G), key=len)
            """最小联通子图"""
            # 如果只有一个联通子图，即最小联通子图就是全部节点，则无需连接
            if len(min_sub_graph) == len(nodes_pos):
                break

            min_graph_nodes_idx = [i for i in min_sub_graph]
            """最小联通子图的节点索引"""
            min_graph_nodes = [nodes_pos[i] for i in min_graph_nodes_idx]
            """最小联通子图的节点坐标"""
            remain_nodes_idx = [
                i for i in range(len(nodes_pos)) if i not in min_sub_graph
            ]
            """最小联通子图外其余节点的索引"""
            remain_nodes = [nodes_pos[i] for i in remain_nodes_idx]
            """最小联通子图外其余节点的坐标"""
            # 计算最小联通子图与其余节点的拓扑距离（基于samples_topo）
            min_length = math.inf
            for i in min_graph_nodes_idx:
                for j in remain_nodes_idx:
                    if nx.has_path(samples_topo, i + len(samples), j + len(samples)):
                        length = nx.shortest_path_length(
                            samples_topo,
                            source=i + len(samples),
                            target=j + len(samples),
                            weight="length",
                        )
                    else:
                        length = math.inf
                    if length < min_length:
                        min_length = length
                        min_i, min_j = i, j
            if min_length != math.inf:
                G.add_edge(min_i, min_j)
                # info(f"基于GVD添加边：{min_i} - {min_j}")
            else:
                warn("无法基于GVD添加边，尝试基于A*添加边")
                # 计算最小联通子图中的节点与其余节点的最近的TOPO_NEIGHBORS个距离
                n_neighbors = (
                    TOPO_NEIGHBORS
                    if len(remain_nodes) > TOPO_NEIGHBORS
                    else len(remain_nodes)
                )
                nbrs = NearestNeighbors(n_neighbors=n_neighbors).fit(remain_nodes)
                distances, indices = nbrs.kneighbors(min_graph_nodes)
                if not bastar:
                    bastar = BidirectionalAStar(self.map)
                min_length = math.inf
                for i in range(len(min_graph_nodes_idx)):
                    for j in range(n_neighbors):
                        length = bastar.search(
                            Position.of(min_graph_nodes[i]),
                            Position.of(remain_nodes[indices[i][j]]),
                        )
                        if length < min_length:
                            min_length = length
                            min_i, min_j = i, indices[i][j]
                if min_length == math.inf:
                    # 添加距离最短的边
                    i, d = min(enumerate(distances[i]), key=lambda x: x[1])
                    G.add_edge(min_graph_nodes_idx[i], remain_nodes_idx[indices[i][j]])
                else:
                    G.add_edge(min_graph_nodes_idx[min_i], remain_nodes_idx[min_j])
            self.G = G
            self.update_samples(samples)
            self.labels = labels

        end_time = rospy.Time.now()
        info("拓扑图更新完成，耗时：", (end_time - start_time).to_sec())
        self.display_topo_edges2(samples_topo)
        self.display_topo_edges(G)

    def update_samples(self, samples):
        self.samples = samples
        self.connectivity_cache.update_samples(samples)

    def map_callback(self, data):
        # self.old_map = self.map
        self.info = data.info
        self.map_width = data.info.width
        self.map_height = data.info.height
        if self.map_width!=data.info.width or self.map_height!=data.info.height:
            rospy.logerr(f"地图尺寸由{self.map_width}*{self.map_height}更新为{data.info.width}*{data.info.height}")
        self.map = np.asarray(data.data).reshape((self.map_height, self.map_width))
        self.connectivity_cache.update_map(self.map)
        # info("地图已更新")

    def check_path(self, G: nx.Graph, idx1, idx2):
        """检测两点之间是否有拓扑路径"""
        if not G.has_node(idx1) or not G.has_node(idx2):
            return False
        return nx.has_path(G, idx1, idx2)

    def get_distance(
        self, pos1: Union[Position, Node], pos2: Union[Position, Node]
    ) -> float:
        """
        计算两点之间的欧式距离

        Parameters
        ----------
        pos1 : Position | Node
            第一个点
        pos2 : Position | Node
            第二个点

        Returns
        -------
        out : float
            两点之间的欧式距离
        """
        return np.linalg.norm(np.array([pos1.x, pos1.y]) - np.array([pos2.x, pos2.y]))

    def unknown_length(self, map, pos1: Position, pos2: Position):
        """
        计算从 pos1 到 pos2 路径上未知区域的长度，并结合欧几里得距离计算总长度

        Parameters
        ----------
        map : np.ndarray
            地图，-1为未知，0为空闲，100为障碍物
        pos1 : Position
            起始位置
        pos2 : Position
            结束位置

        Returns
        -------
        out : float
            路径上未知区域的长度加上两点间的欧几里得距离
        """
        # 如果起始位置和结束位置相同，未知长度为 0
        if pos1 == pos2:
            return 0
        # 提取起始位置的 x 和 y 坐标
        x1, y1 = pos1.y, pos1.x
        # 提取结束位置的 x 和 y 坐标
        x2, y2 = pos2.y, pos2.x

        # 计算 x 方向的差值
        dx = x2 - x1
        # 计算 y 方向的差值
        dy = y2 - y1
        # 初始化未知区域的长度
        length = 0
        # 初始化斜率
        k = 1
        # 如果 x 方向差值为 0，即垂直路径
        if dx == 0:
            # 遍历垂直路径上的每个点
            for y in range(min(y1, y2), max(y1, y2)):
                # 如果该点在地图上为未知区域
                if map[x1][y] == -1:
                    # 未知区域长度加 1
                    length += 1
        # 如果 y 方向差值为 0，即水平路径
        elif dy == 0:
            # 遍历水平路径上的每个点
            for x in range(min(x1, x2), max(x1, x2)):
                # 如果该点在地图上为未知区域
                if map[x][y1] == -1:
                    # 未知区域长度加 1
                    length += 1
        # 其他情况，即斜向路径
        else:
            # 计算斜率
            k = dy / dx
            # 计算截距
            b = y1 - k * x1
            # 如果斜率的绝对值小于 1
            if abs(k) < 1:
                # 计算截距
                b = y1 - k * x1
                # 遍历斜向路径上的每个 x 坐标
                for x in range(min(x1, x2), max(x1, x2)):
                    # 计算对应的 y 坐标
                    y = int(k * x + b)
                    # 如果该点在地图上为未知区域
                    if map[x][y] == -1:
                        # 未知区域长度加 1
                        length += 1
            # 如果斜率的绝对值大于等于 1
            else:
                # 计算斜率的倒数
                k = dx / dy
                # 计算截距
                b = x1 - k * y1
                # 遍历斜向路径上的每个 y 坐标
                for y in range(min(y1, y2), max(y1, y2)):
                    # 计算对应的 x 坐标
                    x = int(k * y + b)
                    # 如果该点在地图上为未知区域
                    if map[x][y] == -1:
                        # 未知区域长度加 1
                        length += 1
        # 计算未知区域长度乘以斜率绝对值的结果
        unknown = length * abs(k)
        # 返回未知区域长度加上两点间的欧几里得距离
        return unknown + sqrt(dx**2 + dy**2)

    def display_topo_nodes(self, centers: np.ndarray):
        point = Point()
        point.z = 0.0
        self.nodes_marker.points.clear()
        for center in centers:
            point.x = center[0] * self.info.resolution + self.info.origin.position.x
            point.y = center[1] * self.info.resolution + self.info.origin.position.y
            self.nodes_marker.points.append(copy(point))
        self.nodes_pub.publish(self.nodes_marker)

    def display_topo_edges(self, graph: nx.Graph):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edges"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.06
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        i = 0
        marker.id = i
        edges_point = Point()
        for u, v in graph.edges:
            i += 1
            edges_point.x = (
                graph.nodes[u]["pos"][0] * self.info.resolution
                + self.info.origin.position.x
            )
            edges_point.y = (
                graph.nodes[u]["pos"][1] * self.info.resolution
                + self.info.origin.position.y
            )
            marker.points.append(copy(edges_point))
            edges_point.x = (
                graph.nodes[v]["pos"][0] * self.info.resolution
                + self.info.origin.position.x
            )
            edges_point.y = (
                graph.nodes[v]["pos"][1] * self.info.resolution
                + self.info.origin.position.y
            )
            marker.points.append(copy(edges_point))
            # markers.markers.append(marker)
        self.edges_pub.publish(marker)
    def display_topo_edges2(self, graph: nx.Graph):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "edges2"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.1
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        i = 0
        marker.id = i
        edges_point = Point()
        for u, v in graph.edges:
            i += 1
            edges_point.x = (
                graph.nodes[u]["pos"][0] * self.info.resolution
                + self.info.origin.position.x
            )
            edges_point.y = (
                graph.nodes[u]["pos"][1] * self.info.resolution
                + self.info.origin.position.y
            )
            marker.points.append(copy(edges_point))
            edges_point.x = (
                graph.nodes[v]["pos"][0] * self.info.resolution
                + self.info.origin.position.x
            )
            edges_point.y = (
                graph.nodes[v]["pos"][1] * self.info.resolution
                + self.info.origin.position.y
            )
            marker.points.append(copy(edges_point))
            # markers.markers.append(marker)
        self.edges_pub2.publish(marker)

    def display_clusters(
        self, centers: np.ndarray, labels: np.ndarray, samples: List[Node]
    ):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "clusters"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 0.1
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        i = 0
        marker.id = i
        edges_point = Point()
        for i in range(len(labels)):
            edges_point.x = (
                centers[labels[i]][0] * self.info.resolution
                + self.info.origin.position.x
            )
            edges_point.y = (
                centers[labels[i]][1] * self.info.resolution
                + self.info.origin.position.y
            )
            marker.points.append(copy(edges_point))
            edges_point.x = (
                samples[i].x * self.info.resolution + self.info.origin.position.x
            )
            edges_point.y = (
                samples[i].y * self.info.resolution + self.info.origin.position.y
            )
            marker.points.append(copy(edges_point))
        self.clusters_pub.publish(marker)

    def display_samples(self, samples):
        explored = Marker()
        unexplored = Marker()
        explored.header.frame_id = unexplored.header.frame_id = "map"
        explored.header.stamp = unexplored.header.stamp = rospy.Time.now()
        explored.ns = unexplored.ns = "nodes"
        explored.id = 0
        unexplored.id = 1
        explored.type = unexplored.type = Marker.POINTS
        explored.action = unexplored.action = Marker.ADD
        explored.pose.orientation.w = unexplored.pose.orientation.w = 1.0
        explored.scale.x = unexplored.scale.x = 0.5
        explored.scale.y = unexplored.scale.y = 0.5
        explored.color.a = unexplored.color.a = 1.0
        explored.color.b = unexplored.color.b = 0.0
        explored.color.g = 1.0
        unexplored.color.g = 0.0
        explored.color.r = 0.0
        unexplored.color.r = 1.0
        explored.lifetime = unexplored.lifetime = rospy.Duration()
        point = Point()
        point.z = 0.0
        explored.points.clear()
        unexplored.points.clear()
        for sample in samples:
            point.x = sample.x * self.info.resolution + self.info.origin.position.x
            point.y = sample.y * self.info.resolution + self.info.origin.position.y
            if sample.explored:
                explored.points.append(copy(point))
            else:
                unexplored.points.append(copy(point))
        self.explored_pub.publish(explored)
        self.unexplored_pub.publish(unexplored)


def check_explored(node: Node, map: np.ndarray) -> bool:
    """
    检测节点是否已探索

    Parameters
    ----------
    node : Node
        节点
    map : np.ndarray
        地图，-1为未知，0为空闲，100为障碍物
    """
    if node.explored:
        return True
    local_map = map[
        int((node.y - node.radius)) : int((node.y + node.radius) + 1),
        int((node.x - node.radius)) : int((node.x + node.radius) + 1),
    ]
    # 使用opencv检测空闲区域和未知区域是否相邻
    # 未知区域的8邻域
    unknown = np.where(local_map == -1)
    if len(unknown[0]) == 0:
        return True
    unknown_neighbour = np.zeros_like(local_map, dtype=np.uint8)
    unknown_neighbour[unknown] = 1
    unknown_neighbour = cv2.dilate(unknown_neighbour, np.ones((3, 3)))
    free = np.where(local_map == 0)
    free_neighbour = np.zeros_like(local_map, dtype=np.uint8)
    free_neighbour[free] = 1
    free_unknown_coincide = free_neighbour * unknown_neighbour
    free_unknown_coincide_sum = free_unknown_coincide.sum()
    if free_unknown_coincide_sum < 5:
        return True
    wall = np.where(local_map == 100)
    wall_neighbour = np.zeros_like(local_map, dtype=np.uint8)
    wall_neighbour[wall] = 1
    wall_neighbour = cv2.dilate(wall_neighbour, np.ones((5, 5)))
    coincide_in_wall = wall_neighbour * free_unknown_coincide
    in_wall_sum = coincide_in_wall.sum()
    if in_wall_sum / free_unknown_coincide_sum > 0.8:
        return True
    else:
        return False


if __name__ == "__main__":
    node = ExploreNode()
    try:
        node.run()
    except Exception as e:
        print(str(e))