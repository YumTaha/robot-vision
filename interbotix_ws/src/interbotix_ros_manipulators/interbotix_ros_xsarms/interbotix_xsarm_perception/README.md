# interbotix_xsarm_perception

[![docs](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_packages/perception_pipeline_configuration.html)

## Overview

This package contains the necessary config and launch files to get any of the many Interbotix X-Series arms working with the [perception pipeline](https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/Building-a-Perception-Pipeline.html) and [YOLO](https://github.com/YumTaha/robot-vision/tree/main/yolo_ros)(https://github.com/mgonzs13/yolo_ros). The end result allows for an arm to pick up any small, non-reflective object from a tabletop-type surface that is within a RealSense color/depth camera's field of view (perception pipeline) OR pick up any object YOLO is trained for from any surface within camera view. While any Intel RealSense color/depth camera can be used, this package was mainly tested with the [D415i](https://www.intelrealsense.com/depth-camera-d415i/) camera.
