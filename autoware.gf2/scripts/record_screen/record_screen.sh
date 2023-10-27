#! /bin/bash

# 设置变量
dirdate=`date +%Y%m%d`
mkdir $dirdate
OUTPUT_FILE="./$dirdate/output.mp4"  # 设置输出文件名ss
DURATION=10              # 设置每段录制时长（秒）

while true; do
    # 开始录制
    ffmpeg -video_size 1920x1080 -framerate 30 -f x11grab -i $DISPLAY -t $DURATION $OUTPUT_FILE

    # 在录制完成后进行处理（你可以根据需要添加其他处理步骤）
    echo "Recording completed. Performing further processing..."

    # 重命名输出文件
    mv $OUTPUT_FILE "${OUTPUT_FILE%.*}_$(date +%Y%m%d%H%M%S).mp4"

    # 等待一段时间后开始下一段录制
    # sleep 0.1
done
