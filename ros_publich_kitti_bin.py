'''
读取KITTI的点云bin文件，并按照文件顺序，每隔0.1s发出一帧topic

本程序需要使用ROS，在melodic下验证过,python版本2.7

用法：
1. 运行程序
```
python ros_publich_kitti_bin.py /home/bit202/bin
```
2. 打开Rviz观察点云
```
rviz -d ros_publich_kitti_bin.rviz
```
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os
import sys
import glob
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

def read_points(file_path):
    points = np.fromfile(file_path, dtype=np.float32)
    return points.reshape(-1, 4)  # 假设点云为[x,y,z,intensity]

def create_point_cloud_msg(points, frame_id):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # 假设点云为[x,y,z,intensity]
    # 创建点云字段
    fields = [pcl2.PointField('x', 0, pcl2.PointField.FLOAT32, 1),
              pcl2.PointField('y', 4, pcl2.PointField.FLOAT32, 1),
              pcl2.PointField('z', 8, pcl2.PointField.FLOAT32, 1),
              pcl2.PointField('intensity', 12, pcl2.PointField.FLOAT32, 1)]

    return pcl2.create_cloud(header, fields, points)

def publish_points(points, publisher, frame_id):
    cloud_msg = create_point_cloud_msg(points, frame_id)
    publisher.publish(cloud_msg)

def main():
    rospy.init_node('kitti_point_cloud_publisher')
    pub = rospy.Publisher("/kitti/points", PointCloud2, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz, 每隔0.1秒

    if len(sys.argv) < 2:
        print("Usage: kitti_publisher.py <path_to_kitti_bin_files>")
        sys.exit(1)

    path = sys.argv[1]
    files = sorted(glob.glob(os.path.join(path, '*.bin')))
    total_files = len(files)  # 获取文件总数

    for index, file_path in enumerate(files):
        if rospy.is_shutdown():
            break
        points = read_points(file_path)
        publish_points(points, pub, "kitti_frame")
        rate.sleep()
        print("Processed {}/{} files.".format(index + 1, total_files))  # 使用.format()代替f-string
        # print(f"Processed {index+1}/{total_files} files.")  # 打印进度

if __name__ == '__main__':
    main()
