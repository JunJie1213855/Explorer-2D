#!/usr/bin/env python3

import os
from math import atan2, cos, sin
import rospy
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
from utils import crop, denoise, find
from topo.msg import Node, NodeArray
from copy import copy
import tf2_ros

LOCAL_RANGE = 50
"""局部搜索范围"""
# RESOLUTION = 5
RESOLUTION = 100
"""局部搜索分辨率，每 `RESOLUTION` 个点搜索一次"""
GLOBAL_FACTOR = 300000
"""全局搜索因子，全局每 `RESOLUTION` * `GLOBAL_FACTOR` 个点搜索一次"""
# MIN_RADIUS = 2
MIN_RADIUS = 10
"""最小GVD半径"""


def info(m):
    """带文件名的loginfo"""
    rospy.loginfo("[{}]: {}".format(os.path.basename(__file__), m))


def debug(m):
    """带文件名的logdebug"""
    rospy.logdebug("[{}]: {}".format(os.path.basename(__file__), m))


class GVDNode:
    def __init__(self) -> None:
        rospy.init_node("GVD", anonymous=False)
        info("GVD节点启动")
        self.gvd_publisher = rospy.Publisher("gvd", MarkerArray, queue_size=10)
        gvd_markers = MarkerArray()
        gvd_list = []
        self.gvd_length = 0
        self.nodes_radius = []

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        rate = rospy.Rate(1)

        self.old_map = None
        self.map = None
        """地图，-1为未知，0为空闲，100为障碍物"""
        """地图话题"""
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

        self.samples = rospy.Publisher("samples", NodeArray, queue_size=10)

        # 等待地图
        while self.map is None:
            info("等待地图……")
            rate.sleep()

        M = self.map.shape[0]
        """全局地图高度"""
        N = self.map.shape[1]
        """全局地图宽度"""
        search_step = 2
        """搜索步长"""
        cost_length = np.zeros((M, N))
        map_center_x = np.zeros(cost_length.shape, dtype=np.int64) - 1
        map_center_y = np.zeros(cost_length.shape, dtype=np.int64) - 1
        feature = np.zeros(self.map.shape)
        map_cover = np.ones(self.map.shape)
        # map_cover[self.map == 205] = 0
        # map_cover[self.map == 0] = 0
        map_cover[self.map == 100] = 0
        # map_cover[self.map == 100] = 0
        nodes = []

        while not rospy.is_shutdown():
            map = self.map.copy()
            map = denoise(map, self.old_map)
            self.old_map = map.copy()
            (
                local_map_start_x,
                local_map_start_y,
                local_map_end_x,
                local_map_end_y,
            ) = crop(map)
            local_map = map[
                local_map_start_y:local_map_end_y, local_map_start_x:local_map_end_x
            ]
            """裁剪后的局部地图，-1为未知，0为空闲，100为障碍物"""

            # local_map = denoise(local_map, old_local_map)
            local_map_cover = map_cover[
                local_map_start_y:local_map_end_y, local_map_start_x:local_map_end_x
            ]
            local_free_x, local_free_y = find(local_map == 0)
            """空闲的点在局部地图中的坐标"""
            # 打乱顺序
            rand_index = np.random.permutation(local_free_x.shape[0])
            local_free_x = local_free_x[rand_index]
            local_free_y = local_free_y[rand_index]
            info("开始计算GVD")
            position = self.getPosition()
            if position is None or not np.any(local_map == 100):
                rate.sleep()
                continue
            local_height, local_width = local_map.shape
            local_start_x = max(
                0, int(position[0] - LOCAL_RANGE / self.info.resolution)
            )
            local_end_x = min(M, int(position[0] + LOCAL_RANGE / self.info.resolution))
            local_start_y = max(
                0, int(position[1] - LOCAL_RANGE / self.info.resolution)
            )
            local_end_y = min(N, int(position[1] + LOCAL_RANGE / self.info.resolution))
            # 遍历所有未被覆盖的点
            for i in range(0, local_free_x.shape[0], RESOLUTION):
                if local_map_cover[local_free_x[i], local_free_y[i]] == 0:
                    continue
                if (
                    local_free_x[i] + local_map_start_y < local_start_x
                    or local_free_x[i] + local_map_start_y > local_end_x
                    or local_free_y[i] + local_map_start_x < local_start_y
                    or local_free_y[i] + local_map_start_x > local_end_y
                ):
                    if (i / RESOLUTION) % GLOBAL_FACTOR != 0:
                        continue
                matrix = np.array([0])
                """搜索矩阵"""
                search_radius = MIN_RADIUS
                """搜索半径"""
                # 扩大搜索范围，直到找到附近有障碍物的点（100）
                while 100 not in matrix:
                    start_x = np.max([0, local_free_x[i] - search_radius])
                    end_x = np.min([local_height - 1, local_free_x[i] + search_radius])
                    start_y = np.max([0, local_free_y[i] - search_radius])
                    end_y = np.min([local_width - 1, local_free_y[i] + search_radius])
                    matrix = local_map[start_x : end_x + 1, start_y : end_y + 1]
                    search_radius += search_step
                matrix_free_xi = local_free_x[i] - start_x
                """local_free_x[i]在搜索矩阵中的坐标"""
                matrix_free_yi = local_free_y[i] - start_y
                """local_free_y[i]在搜索矩阵中的坐标"""
                obstacle_matrix_x, obstacle_matrix_y = find(matrix == 100)
                """搜索矩阵中的障碍物坐标"""
                dist = np.abs(obstacle_matrix_x - matrix_free_xi) + np.abs(
                    obstacle_matrix_y - matrix_free_yi
                )
                """dist为所有障碍物到matrix_free_xi, matrix_free_yi的距离"""
                # 重新确定搜索半径，为最近障碍物的距离+1
                search_radius = np.min(dist) + 1
                start_x = np.max([0, local_free_x[i] - search_radius])
                """搜索范围左边界"""
                end_x = np.min([local_height - 1, local_free_x[i] + search_radius])
                """搜索范围右边界"""
                start_y = np.max([0, local_free_y[i] - search_radius])
                """搜索范围上边界"""
                end_y = np.min([local_width - 1, local_free_y[i] + search_radius])
                """搜索范围下边界"""
                matrix = local_map[start_x : end_x + 1, start_y : end_y + 1]
                matrix_free_xi = local_free_x[i] - start_x
                matrix_free_yi = local_free_y[i] - start_y
                obstacle_matrix_x, obstacle_matrix_y = find(matrix == 100)
                dist = np.abs(obstacle_matrix_x - matrix_free_xi) + np.abs(
                    obstacle_matrix_y - matrix_free_yi
                )
                value = np.min(dist)
                # nearest_obstacles = np.argwhere(dist < value + 2).flatten()
                nearest_obstacles = np.argwhere(dist < value + 3).flatten()
                """距离local_free_x, local_free_y最近的障碍物的坐标"""
                # a = 5
                a = 5
                # if nearest_obstacles.shape[0] > 4:
                if nearest_obstacles.shape[0] > 1:
                    for j in range(1, nearest_obstacles.shape[0]):
                        if (
                            np.abs(
                                obstacle_matrix_x[nearest_obstacles[0]]
                                - obstacle_matrix_x[nearest_obstacles[j]]
                            )
                            > a
                            or np.abs(
                                obstacle_matrix_y[nearest_obstacles[0]]
                                - obstacle_matrix_y[nearest_obstacles[j]]
                            )
                            > a
                        ):
                            center_x = local_map_start_y + local_free_x[i]
                            center_y = local_map_start_x + local_free_y[i]
                            r = np.sqrt(
                                (
                                    obstacle_matrix_x[nearest_obstacles[j]]
                                    - matrix_free_xi
                                )
                                ** 2
                                + (
                                    obstacle_matrix_y[nearest_obstacles[j]]
                                    - matrix_free_yi
                                )
                                ** 2
                            )
                            if r > MIN_RADIUS:
                                cost_length[center_x, center_y] = r
                            break
            while np.sum(cost_length > 0) > 0:
                max_cost_length = cost_length.max()
                """cost_length最大值"""
                max_x, max_y = find(cost_length == max_cost_length)
                """cost_length最大的点的坐标"""
                # 遍历半径为max_cost_length的圆
                r = np.ceil(max_cost_length) + 1
                feature[max_x[0], max_y[0]] = r - 1
                outer_start_x = max(0, max_x[0] - r)
                """外圆左边界"""
                outer_end_x = min(M - 1, max_x[0] + r)
                """外圆右边界"""
                for outer_x in range(int(outer_start_x), int(outer_end_x + 1)):
                    outer_y_range = np.sqrt(r**2 - (outer_x - max_x[0]) ** 2)
                    outer_start_y = max(0, max_y[0] - outer_y_range)
                    outer_end_y = min(N - 1, max_y[0] + outer_y_range)
                    for outer_y in range(int(outer_start_y), int(outer_end_y + 1)):
                        r_inner = np.ceil(cost_length[outer_x, outer_y]) + 1
                        pos1 = [outer_x, outer_y]
                        if r_inner > 0:
                            inner_start_x = max(0, outer_x - r_inner)
                            inner_end_x = min(M - 1, outer_x + r_inner)
                            for inner_x in range(
                                int(inner_start_x), int(inner_end_x + 1)
                            ):
                                inner_y_range = np.sqrt(
                                    r_inner**2 - (inner_x - outer_x) ** 2
                                )
                                inner_start_y = max(0, outer_y - inner_y_range)
                                inner_end_y = min(N - 1, outer_y + inner_y_range)
                                for inner_y in range(
                                    int(inner_start_y + 1), int(inner_end_y - 1 + 1)
                                ):
                                    map_cover[inner_x, inner_y] = 0
                                    map_center_x[inner_x, inner_y] = max_x[0]
                                    map_center_y[inner_x, inner_y] = max_y[0]
                            cost_length[outer_x, outer_y] = 0
                        map_cover[outer_x, outer_y] = 0

            # 发布GVD
            self.publish_gvd(gvd_markers, feature, gvd_list)
            info(f"已发布：{len(gvd_list)}")
            if self.gvd_length != len(gvd_list):
                self.gvd_length = len(gvd_list)
                nodes = []
                for i in range(len(gvd_list)):
                    nodes.append(
                        Node(
                            index=i,
                            x=int(gvd_list[i][0]),
                            y=int(gvd_list[i][1]),
                            radius=self.nodes_radius[i],
                        )
                    )
                # 发布节点
                self.samples.publish(nodes)
                info(f"GVD已发布：{len(gvd_list)}")
            # info(gvd_list)
            rate.sleep()

    def map_callback(self, data: OccupancyGrid):
        # self.old_map = self.map
        self.info = data.info
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map = np.asarray(data.data).reshape((self.map_height, self.map_width))
        # info("地图已更新")

    def getPosition(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        position = transform.transform.translation
        origin = self.info.origin
        q = origin.orientation
        yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        dx, dy = position.x - origin.position.x, position.y - origin.position.y
        row = (-sin(yaw) * dx + cos(yaw) * dy) / self.info.resolution
        col = (cos(yaw) * dx + sin(yaw) * dy) / self.info.resolution
        return row, col

    def publish_gvd(self, markers: MarkerArray, cost_length, gvd_list):
        N = self.map.shape[1]
        x, y = find(cost_length != 0)
        for i in range(0, x.shape[0]):
            center_x_pixel, center_y_pixel = y[i], x[i]
            if [center_x_pixel, center_y_pixel] in gvd_list:
                continue
            gvd_list.append([center_x_pixel, center_y_pixel])
            center_x_real = y[i] * self.info.resolution + self.info.origin.position.y
            center_y_real = x[i] * self.info.resolution + self.info.origin.position.x
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "gvd"
            marker.id = x[i] * N + y[i]
            marker.action = Marker.ADD
            marker.type = Marker.LINE_STRIP
            marker.lifetime = rospy.Duration(0)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 0.5
            r = cost_length[x[i], y[i]]
            self.nodes_radius.append(int(r))
            r = r * self.info.resolution
            for theta in np.arange(0, 2 * 3.24, 0.1):
                point = Point()
                point.x = center_x_real + r * np.cos(theta)
                point.y = center_y_real + r * np.sin(theta)
                point.z = 0
                marker.points.append(point)
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.scale.x = 0.1
            markers.markers.append(marker)
        self.gvd_publisher.publish(markers)


if __name__ == "__main__":
    GVDNode()
