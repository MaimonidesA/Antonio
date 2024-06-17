


docker run \
 --name ros2_humble -h ros2dk \
 -v $PWD/antonio_ws:/home/antonio/antonio_ws \
  --network host \
 --ipc=host \
 -v /dev/bus/usb:/dev/bus/usb -v /tmp/.X11-unix:/tmp/.X11-unix \
 --device-cgroup-rule='c 189:* rmw' --device-cgroup-rule='c 81:* rmw'\
 -e DISPLAY=$DISPLAY \
 --privileged -it --rm antonio_image #\
 #ros2 launch antonio_description display.launch.py

 # -v /dev/bus/usb:/dev/bus/usb 

#--user ros \

#'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc
#'echo 'export CYCLONEDDS_URI=$(pwd)/cyclonedds-profile.xml' >> /root/.bashrc
# --user antonio \
# 

