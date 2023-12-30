#!/usr/bin/env python
# -*- coding: utf-8 -*-
################################################################################
#
# 将使用`kitti2bag`将KITTI数据集转化为ROS `bag`之后的激光雷达topic接收之后，将所有的`i`乘
# 以255之后，并将名字改为“intensity”，之后换个topic名字发出去，以进行Rviz反射率可视化渲染。
#
# [kitti2bag](https://github.com/tomas789/kitti2bag)
#
################################################################################
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

def callback(pointcloud_msg):
    # 创建一个新的点云数据列表
    new_points = []
    for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z", "i"), skip_nans=True):
        # 将'intensity'值乘以255
        new_i = point[3] * 255
        # 添加修改后的点数据
        new_points.append((point[0], point[1], point[2], new_i))

    # 创建一个新的PointCloud2消息
    new_cloud = pc2.create_cloud(pointcloud_msg.header, pointcloud_msg.fields, new_points)
    # 更新'intensity'字段的名称
    for field in new_cloud.fields:
        if field.name == 'i':
            field.name = 'intensity'
    # 发布新的点云数据
    pub.publish(new_cloud)

def listener():
    rospy.init_node('pointcloud_intensity_modifier', anonymous=True)
    # 订阅'/kitti/velo/pointcloud'话题
    rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, callback)
    # 保持程序持续运行直到被手动终止
    rospy.spin()

if __name__ == '__main__':
    # 初始化发布者，发布到新的话题上
    pub = rospy.Publisher('/modified/velo/pointcloud', PointCloud2, queue_size=10)
    listener()
