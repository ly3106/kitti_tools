'''
本程序接收使用[kitti2bag]()生成的`bag`发出的点云，并打印出点云`i`字段的最大值和最小值

本程序依赖于ROS，曾在melodic版本下进行过验证

用法:
```
python listen_kittibag_print_i.py
```
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Initialize variables to store the maximum and minimum intensity values
max_intensity = float('-inf')  # Start with the lowest possible number
min_intensity = float('inf')   # Start with the highest possible number

def callback(pointcloud_msg):
    global max_intensity, min_intensity  # Indicate that we'll use the global variables
    # 遍历点云数据
    for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z", "i"), skip_nans=True):
        # point 是一个包含x, y, z, 和 intensity 的元组
        i = point[3]  # 假设'intensity'字段代表'i'
        
        # Update the max and min intensity values
        max_intensity = max(max_intensity, i)
        min_intensity = min(min_intensity, i)

    # Print the current max and min intensity values
    print("Current Max Intensity: ", max_intensity)
    print("Current Min Intensity: ", min_intensity)

def listener():
    rospy.init_node('pointcloud_listener', anonymous=True)
    # 订阅'/kitti/velo/pointcloud'话题
    rospy.Subscriber("/kitti/velo/pointcloud", PointCloud2, callback)
    # 保持程序持续运行直到被手动终止
    rospy.spin()

if __name__ == '__main__':
    listener()