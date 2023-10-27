# Perception Engine's Continental ARS408 Driver

## How to compile

```
$ git clone https://gitlab.com/perceptionengine/pe-drivers/ars408_ros.git pe_ars408_ws/src && cd pe_ars408_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin build
```

# Radar status
Obtain the radar status
```
roslaunch pe_ars408_ros ars408_status.launch
```

# Radar setup

If the sensor does not match the `OBJECTS` mode. Use the setup node.
**Avoid to execute the setup node continously. The Radar lifespan memory gets reduced on every write.**  

Example using SocketCAN to **get** the configuration status of the device. 
```
roslaunch pe_ars408_ros ars408_status.launch
```

Example using Kvaser Interface to **set** the device configuration.
```
roslaunch pe_ars408_ros ars408_setup_kvaser.launch can_device:=can0 can_topic:=can_raw perform_setup:=True can_circuit:=3 can_hardware_id:=11111 
```
This will:
1. Obtain the current status of the device.
1. If the current setting is not set to `OBJECTS` mode. It will send a CAN message through Kvaser Interface to the sensor.
1. Verify the configuration was set. IF it was set it will end. Otherwise it will repeat.

# Driver

The driver works directly on CAN Frames, so this driver can work with Kvaser, SocketCAN or 
any other interface that publishes CAN Frame msgs (http://docs.ros.org/en/melodic/api/can_msgs/html/msg/Frame.html).

## How to run 

Example with SocketCAN Bridge.  

1. Enable can port
`sudo ip link set can0 up type can bitrate 500000`

2. Launch the driver
`roslaunch pe_ars408_ros continental_ars408.launch`
   
## Parameters

|Parameter|Description|Default|
|---|---|---|
|`can_device`|Device name of the can interface|`can0`|
|`can_topic`|Topic on which socketcan will publish the can raw msg|`can_raw`|

## Launch file

The launch file will initiate two nodes:
1. socketcan_bridge to read from `can0` and publish the CAN msg in `can_raw`
1. Continental ARS408 driver will read the `can_raw`, parse and publish the detected objects using the Autoware.IV
   `autoware_perception_msgs/DynamicObjectArray` in the topic `/detection/radar/objects`.
   
If needed change the launch file to match kvaser_interface.

# Visualization

To visualize the objects the Autoware's `dynamic_object_visualizer` needs to be launched and subscribe to `/detection/radar/objects`.

i.e.`roslaunch dynamic_object_visualization dynamic_object_visualizer.launch with_feature:=False input:=/detection/radar/objects`