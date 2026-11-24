#!/usr/bin/env python3

import os
import subprocess

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
import csv
from datetime import datetime
from PIL import Image

def info(m):
    """带文件名的loginfo"""
    rospy.loginfo("[{}]: {}".format(os.path.basename(__file__), m))


class RecordNode:
    """保存地图、实时位置和地图已知面积的节点"""

    def __init__(self) -> None:
        rospy.init_node("save_node")
        info("保存节点启动")
        prefix = rospy.get_param("~prefix", "default")
        # 创建CSV文件并写入表头
        self.name = prefix + "_" + datetime.now().strftime("%Y%m%d-%H%M%S")
        self.csv_file = open(
            "/home/mean/catkin_ws/src/topo/data/" + self.name + ".csv", "w"
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "x", "y", "known_area"])
        self.known_area = 0
        self.map = None
        self.map_sub = rospy.Subscriber(
            "/map", OccupancyGrid, self.map_callback, queue_size=1
        )
        self.map_sub = rospy.Subscriber(
            "/tb3_0/map", OccupancyGrid, self.map_callback, queue_size=1
        )
        # 订阅/odom话题
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.odom_sub = rospy.Subscriber("/tb3_0/odom", Odometry, self.odom_callback)
        rospy.on_shutdown(self.save_map)

    def map_callback(self, data: OccupancyGrid):
        self.map = data
        map = np.asarray(data.data)
        self.known_area = np.sum(map != -1)

    def odom_callback(self, msg: Odometry):
        # 提取位置信息和时间戳
        time = msg.header.stamp.to_sec()
        position = msg.pose.pose.position
        # 将数据写入CSV文件
        self.csv_writer.writerow([time, position.x, position.y, self.known_area])

    def save_map(self):
        # 关闭CSV文件
        self.csv_file.close()
        # 保存地图到文件
        map_file_path = "/home/mean/catkin_ws/src/topo/map/" + self.name
        try:
            #  保存地图信息
            map_info_file = open(map_file_path + ".yaml", "w")
            map_info_file.write("image: " + self.name + ".pgm\n")
            map_info_file.write("resolution: " + format(self.map.info.resolution,".3f") + "\n")
            map_info_file.write("origin: [" + str(self.map.info.origin.position.x) + ", " +
                                 str(self.map.info.origin.position.y) + ", " +
                                 str(self.map.info.origin.orientation.z) + "]\n")
            map_info_file.close()
            # 保存地图
            map = np.asarray(self.map.data).reshape(
                self.map.info.height, self.map.info.width
            )
            map = np.flipud(map) # 翻转以适应图像坐标系
            map=np.uint8((map == -1) * 205 + (map == 0) * 254 + (map == 100) * 0)  # 设置灰度值
            image = Image.fromarray(map)
            image.save(map_file_path + ".pgm")
            info("地图保存在：" + map_file_path)
        except subprocess.CalledProcessError as e:
            rospy.logerr("地图保存失败：%s", e)

    def run(self):
        # 保持节点运行
        rospy.spin()


if __name__ == "__main__":
    try:
        save_node = RecordNode()
        save_node.run()
    except rospy.ROSInterruptException:
        pass
