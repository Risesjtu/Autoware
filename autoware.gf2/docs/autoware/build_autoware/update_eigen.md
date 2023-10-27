# 升级Eigen3
系统原有的eigen3版本是3.3.4,需要卸载再重装eigen3.3.7

# 卸载旧版本
## 查看当前版本
```bash
$ pkg-config --modversion eigen3
3.3.4
```
## 删除eigen3相关文件
### 查看eigen3位置
```bash
sudo updatedb
locate eigen3
```
### 删除eigen3相关文件
```bash
sudo rm -rf /usr/include/eigen3
sudo rm -rf /usr/lib/cmake/eigen3
sudo rm -rf /usr/local/include/eigen3
sudo rm -rf /usr/share/doc/libeigen3-dev 
sudo rm -rf /usr/local/share/pkgconfig/eigen3.pc /usr/share/pkgconfig/eigen3.pc /var/lib/dpkg/info/libeigen3-dev.list /var/lib/dpkg/info/libeigen3-dev.md5sums
sudo rm -rf /usr/local/lib/pkgconfig/eigen3.pc
sudo rm -rf /usr/local/share/eigen3
```
### 刷新查看是否删除彻底
```bash
sudo updatedb
locate eigen3
pkg-config --modversion eigen3
```
# 安装需要的版本
## 官网下载需要的版本的压缩包解压
- [https://gitlab.com/libeigen/eigen/-/releases](https://gitlab.com/libeigen/eigen/-/releases)
- [Eigen 3.3.7 Source code (tar.gz) ](https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz)

# 编译安装
```bash
cd 3.3.7/eigen-eigen-323c052e1731 && mkdir build && cd build && cmake .. && make
sudo make install
sudo cp -r /usr/local/include/eigen3 /usr/include 
cd && rm -rf 3.3.7.tar.gz && rm -rf 3.3.7 #(也可以不执行此删除命令)
pkg-config --modversion eigen3 # 查看当前版本
```

从相关头文件中的宏定义也可以看出当前eigen库的版本
```bash
$ cat /usr/local/include/eigen3/Eigen/src/Core/util/Macros.h | grep _VERSION
#define EIGEN_WORLD_VERSION 3
#define EIGEN_MAJOR_VERSION 3
#define EIGEN_MINOR_VERSION 7
...
```
# 参考
- [Eigen3卸载与重装](https://blog.csdn.net/qq_45401419/article/details/118358687)


