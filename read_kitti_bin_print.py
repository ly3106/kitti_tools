
import numpy as np

# 替换为你的KITTI点云文件路径
file_path = '/home/bit202/KITTI/object/velodyne/testing/velodyne/000000.bin'

# 定义每个点的数据结构
point_dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)])

# 读取文件并将其转换为numpy数组
points = np.fromfile(file_path, dtype=point_dtype)

# 打印前500个点以检查
print(points[:500])

# 找出所有点中强度的最大值和最小值
intensities = points['intensity']  # 获取所有点的强度
max_intensity = intensities.max()  # 计算最大强度
min_intensity = intensities.min()  # 计算最小强度

# 打印最大强度和最小强度
print("最大强度: ", max_intensity)
print("最小强度: ", min_intensity)