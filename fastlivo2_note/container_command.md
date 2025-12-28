# 1. 允許 Docker 使用本地顯示資源
xhost +local:docker

# 2. 啟動容器
docker run -it \
    --net=host \
    --runtime=nvidia \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    -v ~/fastlivo2_ws:/root/catkin_ws \  #這邊接fastlivo2會用到的的東西，裝在本機映射過去容器裡面的
    -v /dev:/dev \
    --name [cintainer name that u like] \
    ros:noetic-ros-base /bin/bash


# 1. 安裝環境 (一次性)
apt-get update
apt-get install -y ros-noetic-desktop libpcl-dev libpcap-dev libffi-dev
ln -sf /usr/lib/aarch64-linux-gnu/libffi.so.7 /usr/lib/aarch64-linux-gnu/libffi.so.8
ldconfig

# 2. 執行
source /root/catkin_ws/devel/setup.bash
rosrun rviz rviz
