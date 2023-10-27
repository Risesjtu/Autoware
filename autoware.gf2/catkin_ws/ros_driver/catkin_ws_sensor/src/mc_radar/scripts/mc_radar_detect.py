#!/usr/bin/env python
#coding:utf-8
import can
import rospy
from std_msgs.msg import Bool 
from std_msgs.msg import Int16MultiArray 

# 12*[0] --> [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# 乘法有复制的功能
channel = 12*[0]
channel_detect = 12*[0]

can_type = rospy.get_param("can_type_1")
can_channel = rospy.get_param("can_channel_1")
bus = can.interface.Bus(bustype=can_type, channel=can_channel, bitrate=500000)

bus.set_filters([
    {
        'can_id': 0x611,
        'can_mask': 0x7ff,
        'extended': False
    },
    {
        'can_id': 0x612,
        'can_mask': 0x7ff,
        'extended': False
    },
    {
        'can_id': 0x613,
        'can_mask': 0x7ff,
        'extended': False
    }
])

def decimalize(hex_data):
    hex_str = hex(hex_data)[2:]
    if len(hex_str) > 1:
        return hex_str
    else:
        return '0'+hex_str
    
def decode_range(byte_0, byte_1):
    return int(decimalize(byte_0)+decimalize(byte_1))

def update_range(message):
    # 0 1 2 3  号探头的障碍物距离设置，超过距离的障碍物的数据
    if message.arbitration_id == 0x611:
        channel[0] = decode_range(message.data[0], message.data[1])
        channel[1] = decode_range(message.data[2], message.data[3])
        channel[2] = decode_range(message.data[4], message.data[5])
        channel[3] = decode_range(message.data[6], message.data[7])
    # 4 5 6 7  号探头的障碍物距离设置，超过距离的障碍物的数据
    elif message.arbitration_id == 0x612:
        channel[4] = decode_range(message.data[0], message.data[1])
        channel[5] = decode_range(message.data[2], message.data[3])
        channel[6] = decode_range(message.data[4], message.data[5])
        channel[7] = decode_range(message.data[6], message.data[7])
    # 8 9 10 11 号探头的障碍物距离设置，超过距离的障碍物的数据
    elif message.arbitration_id == 0x613:
        channel[8] = decode_range(message.data[0], message.data[1])
        channel[9] = decode_range(message.data[2], message.data[3])
        channel[10] = decode_range(message.data[4], message.data[5])
        channel[11] = decode_range(message.data[6], message.data[7])
    
def detect(gear):
    # 前进档位下-----------------------------------------
    if gear == 1:
        for i in range(12):
            # 0 1 2 3 号探头
            if i<=3:
                # 障碍物距离 -- 1200mm内
                if channel[i] <= 1200:
                    channel_detect[i] = 1
                else:
                    channel_detect[i] = 0 
            # 4 5 6 7 号探头  
            if i>3 and i<=7:
                # 障碍物距离 -- 1000mm内
                if channel[i] <= 1000:
                    channel_detect[i] = 1
                else:
                    channel_detect[i] = 0   
            # 8 9 10 11 号探头    
            if i>7:
                # 障碍物距离 -- 1200mm内
                if channel[i] <= 1200:
                    channel_detect[i] = 1
                else:
                    channel_detect[i] = 0

    # 后退档位下-------------------------------------------
    elif gear == 3:
        for i in range(12):
            # 0 1 2 号探头
            if i<=2:
                if channel[i] <= 50:
                    channel_detect[i] = 1
                else:
                    channel_detect[i] = 0
            # 3 4 5 6 7 8 号探头
            if i>2 and i<=8:
                if channel[i] <= 50:
                    channel_detect[i] = 1
                else:
                    channel_detect[i] = 0
            # 9 10 11 号探头  
            if i>9:
                if channel[i] <= 200:
                    channel_detect[i] = 1
                else:
                    channel_detect[i] = 0
    # 累加和大于0，说明至少有一个探头，探测的障碍物距离小于安全距离
    sum_detect = sum(channel_detect)
    if sum_detect > 0:
        return True
    else:
        return False


def mc_radar_detect():

    rospy.init_node('mc_radar_detect_node', anonymous=True)
    pub_detect = rospy.Publisher('mc_radar_detect', Bool, queue_size=1)
    pub_range = rospy.Publisher('mc_radar_range', Int16MultiArray, queue_size=1)
    try:
        activate_message = can.Message(
            arbitration_id=0x601,
            data=[0xb2, 0x1f, 0xff],
            is_extended_id=False
        )
        # bus.send(activate_message)
    except:
        return 0 
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
        bus.send(activate_message)
        for i in range(3):
            message = bus.recv()
            update_range(message)
        pub_range.publish(Int16MultiArray(data=channel))
        # 判断是几号探头没有工作
        for i,j in enumerate(channel):
            if j == 5005:
                if i == 0 or i==1:
                    continue
                    print("{}号探头没有工作，请重新插拔".format(i+1))
        pub_detect.publish(detect(1))
        r.sleep()


if __name__ == '__main__':
    mc_radar_detect()


