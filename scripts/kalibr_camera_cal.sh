#!/bin/bash 
rosrun kalibr kalibr_calibrate_cameras \
--models pinhole-equi \
--target /home/tfanselo/projects/20190126_vio_hardware/catkin_ws/src/vio_hardware/config/kalibr/aprilgrid.yaml \
--bag /home/tfanselo/projects/20190126_vio_hardware/logs/vio_calibration_log_2019-02-01-18-13-28.bag  \
--topics /vio_hardware/image_timestamp_corrected

