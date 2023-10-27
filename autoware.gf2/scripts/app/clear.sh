#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"
cd ../

# 在杀死所有终端前，先启动一个app（这个sh中有2s的延迟，能保证在下面杀死命令后，再启动app节点）
gnome-terminal -t "app" -- bash -c "bash app_client.sh"
# gnome-terminal -t "roscore" -- bash -c "bash app/roscore.sh"

# 杀死所有终端进程
ps -ef | grep bash | grep -v client | cut -c 9-15 | xargs kill -s 9




