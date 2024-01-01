#####################################################################################
# 要批量将某个文件夹下的文件按照顺序重命名为
# “000000”
# “000001”
# “000002”
# 这样的KITTI点云文件名的格式
#
#####################################################################################
# 用法：
# 1. 修改下面程序中的`folder_path`指向自己的文件夹，注意最后没有“/”
# 2. 直接运行该程序，例如
# ```
# python3 rename_to_kitti_bin.py
# ```
# 3. 打印日志最后为这样，表示完成
# ```
# 原文件名: 943925227.258200000.bin -> 新文件名: 002273.bin | 进度: 2274/2275 | 估算剩余时间: 0.00秒
# 原文件名: 943925227.308200000.bin -> 新文件名: 002274.bin | 进度: 2275/2275 | 估算剩余时间: 0.00秒
# 文件重命名完成。
# ```
#####################################################################################


import os
import time

# 设定你的文件夹路径
folder_path = '/home/bit202/bin'

# 获取该文件夹内所有文件的名称，并按名字排序
files = sorted(os.listdir(folder_path))

# 计数器用于生成数字序列
counter = 0

# 开始时间
start_time = time.time()

for file in files:
    # 获取文件扩展名
    extension = os.path.splitext(file)[1]

    # 生成新文件名，格式为六位数加扩展名，如 "000001.jpg"
    new_filename = f'{counter:06}{extension}'

    # 获取原文件的完整路径
    old_file = os.path.join(folder_path, file)

    # 获取新文件的完整路径
    new_file = os.path.join(folder_path, new_filename)

    # 重命名文件
    os.rename(old_file, new_file)

    # 更新计数器
    counter += 1

    # 计算并打印进度
    elapsed_time = time.time() - start_time
    files_left = len(files) - counter
    if counter > 0:
        estimated_time_left = (elapsed_time / counter) * files_left
    else:
        estimated_time_left = 0

    # 打印转换日志，包括原始文件名和新文件名
    print(f'原文件名: {file} -> 新文件名: {new_filename} | 进度: {counter}/{len(files)} | 估算剩余时间: {estimated_time_left:.2f}秒')

print("文件重命名完成。")
