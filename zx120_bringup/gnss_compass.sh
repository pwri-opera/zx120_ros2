#!/bin/bash
#--------------------------------------------------------------------
#バックグラウンド実行用のスクリプト
#--------------------------------------------------------------------

#変数の設定
SCRIPTDIR=/home/zx120/ros2_ws/src/zx120_ros2/zx120_bringup
LOGDIR=$SCRIPTDIR/log
ENVFILE=/home/zx120/ros2_ws/install/setup.bash
DISPLAY=:1

#実行
if [ -f ${ENVFILE} ]; then
    #環境変数読み込み
    echo "Loading ROS2 Env..."
    source /home/zx120/ros2_ws/install/setup.bash
    if [ -d ${LOGDIR} ]; then
        echo "ROS2 Launching..."
        #roslaunch実行
	exec gnome-terminal -- ros2 launch zx120_bringup zx120_gnss.launch.py >> ${LOGDIR}/gnss_compass.log 2>&1
    else
        echo "There is no ${LOGDIR}"
    fi
else
    echo "There is no ${ENVFILE}"
fi
