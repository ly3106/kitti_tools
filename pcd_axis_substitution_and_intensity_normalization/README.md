# 用法
## 1. 进入到当前路径
## 2. 编译
```
mkdir build
cd build
cmake ..
make
```
## 3. 运行
运行语法为：
```
pcd_axis_substitution_and_intensity_normalization [source folder] [optional: target folder] [optional: encoding (ascii/binary/compressed)]
```
实例：
```
./pcd_axis_substitution_and_intensity_normalization /home/bit202/pcd
```
或
```
./pcd_axis_substitution_and_intensity_normalization /home/bit202/pcd /home/bit202/pcd_trans
```
或
```
./pcd_axis_substitution_and_intensity_normalization /home/bit202/pcd /home/bit202/pcd_trans compressed
```
### ps
当有第3个参数的时候第2个参数不可省略