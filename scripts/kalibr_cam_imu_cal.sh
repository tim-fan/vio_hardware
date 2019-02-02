#!/bin/bash 
BAG_FILE="$1"
rosrun kalibr kalibr_calibrate_imu_camera \
--imu /home/tfanselo/projects/20190126_vio_hardware/catkin_ws/src/vio_hardware/config/kalibr/imu.yaml \
--cam /home/tfanselo/projects/20190126_vio_hardware/catkin_ws/src/vio_hardware/config/kalibr/camchain.yaml \
--target /home/tfanselo/projects/20190126_vio_hardware/catkin_ws/src/vio_hardware/config/kalibr/aprilgrid.yaml \
--bag "$1"
