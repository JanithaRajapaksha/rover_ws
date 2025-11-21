docker run -it \
  --name ros_humble \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev/shm:/dev/shm \
  -v /home/rover1/ros_humble:/rover \
  --privileged \
  --user root \
  -w /rover/rover_ws \
  $(for d in /dev/ttyACM* /dev/ttyUSB* /dev/input/event* /dev/input/js*; do \
      [ -e "$d" ] && echo --device $d:$d; \
    done) \
  ros_humble

