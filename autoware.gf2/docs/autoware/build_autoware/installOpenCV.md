# 安装OpenCV
由于之前安装了`libopencv-dev3.2.0`，因此安装的OpenCV版本必须一致，否则会冲突.
新建`installOpencv320.sh`文件，内容如下:   
```bash
echo "** Install requirement"
sudo apt-get update
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install -y python2.7-dev
sudo apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
sudo apt-get install -y libv4l-dev v4l-utils qv4l2 v4l2ucp
sudo apt-get install -y curl
sudo apt-get update
echo "** Download opencv-3.2.0"
curl -L https://github.com/opencv/opencv/archive/3.2.0.zip -o opencv-3.2.0.zip
unzip opencv-3.2.0.zip
cd opencv-3.2.0/
echo "** Building..."
mkdir build
cd build/
cmake -D WITH_CUDA=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF -D CMAKE_BUILD_TYPE=RELEASE -D ENABLE_PRECOMPILED_HEADERS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
echo "** Install opencv-3.2.0 successfully"
```

执行脚本下载并安装`OpenCV 3.2.0`
```bash
chmod +x installOpencv320.sh
./installOpencv320.sh
```