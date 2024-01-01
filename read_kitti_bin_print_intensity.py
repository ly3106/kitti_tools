"""
功能描述: 本程序用于读取KITTI `bin`点云文件，并且将每个`bin`的`intensity`的最大值和最小值打印出来，并最终打印出最大值的最大值和最小值的最小值。

用法:
1. 修改下面`directory_path`指到你的`bin`文件所在路径，注意路径结尾没有“/”
2. 运行本程序
```
python3 read_kitti_bin_print_intensity.py
```
"""
import numpy as np
import os
import glob

# 设置KITTI点云数据的目录路径
directory_path = '/media/bit202/bin'

# 定义每个点的数据结构
point_dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)])

# 获取目录下所有的.bin文件并按文件名排序
bin_files = sorted(glob.glob(os.path.join(directory_path, '*.bin')))

# 初始化全局最大强度和最小强度
global_max_intensity = -np.inf
global_min_intensity = np.inf

# 遍历每个文件
for file_path in bin_files:
    # 读取文件并将其转换为numpy数组
    points = np.fromfile(file_path, dtype=point_dtype)

    # 获取所有点的强度
    intensities = points['intensity']

    # 计算并打印最大强度和最小强度
    max_intensity = intensities.max()
    min_intensity = intensities.min()
    
    # 更新全局最大强度和最小强度
    global_max_intensity = max(global_max_intensity, max_intensity)
    global_min_intensity = min(global_min_intensity, min_intensity)

    # 打印当前文件的信息
    print(f"文件: {os.path.basename(file_path)}")
    print(f"最大强度: {max_intensity}")
    print(f"最小强度: {min_intensity}")
    print("-" * 30)  # 打印分隔线以区分不同文件的结果

# 打印所有文件中的最大的最大强度和最小的最小强度
print(f"所有文件中的最大强度值: {global_max_intensity}")
print(f"所有文件中的最小强度值: {global_min_intensity}")
