xhost +local:docker

docker run -it --rm \
    --name slamesh \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume $XAUTHORITY:/root/.Xauthority \
    --volume=/home/jun/ros1_convert:/root/dataset/ \
    -p 5900:5900 \
    -p 2222:22 \
    -e RESOLUTION=1920x1080 \
    junhyeok/slamesh:latest

