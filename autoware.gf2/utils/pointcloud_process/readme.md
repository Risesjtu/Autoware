# transform_pointcloud
## 编译
```bash
pointcloud_process$ mkdir -p build && cd build
pointcloud_process/build$ cmake .. && make
```
## 运行
```bash
$ chmod +x transform_pointcloud # 添加可执行权限
# 对点云 input.pcd 进行坐标变换，变换矩阵在文件 transform.txt 中写作 4*4 矩阵，同一行数字间用空格隔开
# 点云转换完成后程序将自动在输入点云文件的同级目录下创建新文件夹，用于保存输出点云文件
$ ./transform_pointcloud input.pcd transform.txt 
# 使用 transform.txt 中矩阵的逆矩阵对输入点云进行坐标变换
$ ./transform_pointcloud input.pcd transform.txt -i
```

# split_pointcloud
## 编译
```bash
pointcloud_process$ mkdir -p build && cd build
pointcloud_process/build$ cmake .. && make
```
## 运行
```bash
$ chmod +x split_pointcloud # 添加可执行权限
# 对点云 input.pcd 进行分块, 并自动在同级目录下创建文件夹，用于存放分块后的点云文件
# 代码中默认按照 X 轴和 Y 轴 分割成 100m*100m 的方块状点云
$ ./split_pointcloud input.pcd 
```