# rviz_plugin
这是ros自带的一个库文件，但是由于在无人车的巡检功能中需要用到新的功能于是对其中的plantflag进行了二次开发。增加的功能是：当鼠标单击RVIZ中的plantflag功能，并且在界面上选择一个点后，会将这个点的坐标发布到 /inspection_target 话题上。

# 部署方法

在根目录下寻找这个文件：
    sudo find / | grep librviz_plugin_tutorials.so

目录一般为 :
    /opt/ros/melodic/lib/librviz_plugin_tutorials.so

而后需要将这个库文件备份

    sudo mv /opt/ros/melodic/lib/librviz_plugin_tutorials.so /opt/ros/melodic/lib/librviz_plugin_tutorials.so.bak

拷贝同目录下的库文件：

    sudo cp -f ~/autoware.gf2/catkin_ws/ros_planning/rviz_plugins/librviz_plugin_tutortials.so /opt/ros/melodic/lib/

而后在rviz中可以直接使用。（位于rivz中工具栏的“+”内，与 “2D estimate pose” 同一行）
