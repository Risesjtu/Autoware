# 估计变换矩阵
当前文件夹下的 3 个 Python 文件是由 Pix 提供的程序, 用于解析 rosbag 数据包中的 /gnss_pose 和 /ndt_pose 两个话题上的数据, 
从而估计出点云地图坐标系和GNSS参考点坐标系之间的变换矩阵, 以便后续将点云地图统一坐标系.

```
geo_extract_bag.py
para_compute.py
rotation.py
```

# 运行
```bash
python geo_extract_bag.py [*].bag
```

# 样例输出
```
utils/gnss_conv$ python geo_extract_bag.py autoware-20200918105939.bag 
extract bag: autoware-20200918105939.bag
r matrix:

[[-0.99967592  0.022874    0.01117303]
 [-0.02374769 -0.9960464  -0.08560155]
 [ 0.0091708  -0.08583914  0.9962668 ]]
t matrix:

[  493.76232472  1274.67780532 -1160.19819068]


a-aa mean error:

[0.03057933 0.06692708 0.03959106]
('c-test_b:\n', array([0.03942313, 0.08924367, 0.04630217]))
```

# 后续
手动将旋转矩阵 r matrix 和平移向量 t matrix 组合成 4*4 的变换矩阵，写入 transform.txt 文件

```plain transform.txt
-0.99967592  0.022874    0.01117303 493.76232472
-0.02374769 -0.9960464  -0.08560155 1274.67780532
0.0091708  -0.08583914  0.9962668 -1160.19819068
0   0   0   1
```

在对点云地图进行坐标变换时，使用上述矩阵的逆矩阵，即
```
$ ./transform_pointcloud input.pcd transform.txt -i
```

