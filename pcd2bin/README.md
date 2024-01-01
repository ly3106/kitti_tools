# 1. 修改介绍
本程序在原始项目[pcd2bin](https://github.com/leofansq/Tools_RosBag2KITTI/tree/master/pcd2bin)的基础上修改了主程序,
使的程序使用可以更灵活，修改日期2024-01-01。

# 2.使用步骤

## 2.1 编译
```
mkdir build
cd build
cmake ..
make
```

## 2.2 用法：
1. 将pcd文件复制到本文件同级目录下的`pcd`文件夹下，运行
```
./pcd2bin
```
文件会保存在同级目录下的`bin`文件夹下。

2. 指定包含`pcd`文件的路径和`bin`文件的路径
```
./pcd2bin /home/bit202/pcd /home/bit202/bin
```
生成的`bin`文件在`/home/bit202/bin`中

# 3. Create files_list.txt ###
The KITTI dataset has txt files like train.txt trainval.txt val.txt, which contains a subset of all data files.  So we need to get the files_list.txt

* **Get files_list:** The result will be saved into [bin](/pcd2bin/bin) named *list.txt*.

	`cd bin`; `ls -1 | grep ".bin$" > list.txt`

* **Create the final .txt:** The txt file obtained in the previous step contains a file suffix such as .bin. This requires further processing. After this step, you will get the final txt file named *files_list.txt*.

	`python get_list.py`

