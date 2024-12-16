# **Documentation for RealSense and YOLO Setup**

## **1. RealSense Installation**

To integrate RealSense with **ROS 2**, follow the steps provided in the [Intel RealSense GitHub repository](https://github.com/IntelRealSense/realsense-ros).

**Command for launching the RealSense camera:**
```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true
```
If the "_frame not published within 5sec_" error pops up, you can add `initial_reset:=true` to the command.

### **Ensure the following arguments are enabled:**
- **PointCloud**: Enables depth to 3D point cloud mapping.
- **Align Depth**: Aligns the depth image with the color image for accurate overlays (dimensions).

> [!NOTE]
> Ensure these topics match the RealSense camera configuration to avoid misalignments.

---

## **2. YOLO Installation/Configuration**

### **i. installing YOLO**

To integrate YOLO with **ROS 2**, follow the steps provided in the [YOLO Ros GitHub repository](https://github.com/mgonzs13/yolo_ros?tab=readme-ov-file#installation).

### **ii. NVIDIA Driver Installation for YOLO**

To use YOLO with your GPU, ensure the **NVIDIA drivers** are correctly installed.
Follow the official [Ubuntu NVIDIA Driver Installation Guide](https://ubuntu.com/server/docs/nvidia-drivers-installation).

**Verify CUDA availability in the terminal:**
```python
~$ python3
>>> import torch
>>> print(torch.cuda.is_available())
```
If the output is `True`, CUDA is successfully installed and functional.

> [!IMPORTANT]
> Verify CUDA and PyTorch compatibility to avoid runtime errors.
> 
> <details>
> <summary>Example CUDA Check</summary>
> 
> ```bash
> nvidia-smi
> ```
> </details>

### **iii. Training YOLO**

To train a YOLO model for object detection:

#### **Dataset Preparation:**
- Use [RoboFlow](https://roboflow.com/) to upload, annotate, and augment your dataset.

#### **Model Training:**
- Access [Google Colab](https://colab.research.google.com/) through RoboFlow for training.
- Export the trained model for deployment in **YOLO ROS**.

> [!TIP]
> Use data augmentation in RoboFlow to improve model performance.
> 
> <details>
> <summary>Example Dataset Workflow</summary>
> 
> 1. Open an augmentation workspace in RoboFlow.
> 2. Upload images.
> 3. Annotate objects of interest.
> 4. Train model online using Google Colab.
> 5. Export in YOLO format (choose your YOLO version). <sub> YOLOv11 </sub>
> </details>

### **iv. YOLO ROS Configuration**

#### **Editing the YOLO Launch File**
Navigate to `yolo_ros/yolo_bringup/launch` and modify the `yolo.launch.py` file with the following parameters:

| **Parameter**                   | **Value**                                                    |
|---------------------------------|------------------------------------------------------------|
| `model_type`                    | Path to your trained YOLO model (e.g., `/yolo_ros/best.pt`) |
| `input_image_topic`             | `/camera/camera/color/image_raw`                           |
| `image_reliability`             | `1`                                                        |
| `input_depth_topic`             | `/camera/camera/aligned_depth_to_color/image_raw`          |
| `depth_image_reliability`       | `1`                                                        |
| `input_depth_info_topic`        | `/camera/camera/aligned_depth_to_color/camera_info`        |
| `depth_info_reliability`        | `1`                                                        |
| `target_frame`                  | `camera_link`                                              |
| `maximum_detection_threshold`   | `0.05`                                                     |


> [!WARNING]
> Ensure these topics match the RealSense camera configuration to avoid misalignments.

> [!NOTE]
> If hard coding parameters is not something you like, you can just put them as arguments in the launch command.
> 
> <details>
> <summary>Launch Command</summary>
> 
> ```bash
> ros2 launch yolo_bringup yolov11.launch.py model:="../yolo_ros/best.pt" use_3d:=True input_image_topic:="/camera/camera/color/image_raw" image_reliability:=1 input_depth_topic:="/camera/camera/aligned_depth_to_color/image_raw" depth_image_reliability:=1 input_depth_info_topic:="/camera/camera/aligned_depth_to_color/camera_info" depth_info_reliability:=1 target_frame:="camera_link" maximum_detection_threshold:=0.05
> ```
> </details>

### **v. Command Execution**
To launch the YOLO ROS node with the configured parameters, execute:

#### **With 2D detection:**
```bash
ros2 launch yolo_bringup yolov11.launch.py
```
#### **With 3D detection:**
```bash
ros2 launch yolo_bringup yolov11.launch.py use_3d:=True
```

> [!TIP]
> Adjust `maximum_detection_threshold` for optimizing detection confidence.

<details>
<summary>Command Execution Example</summary>

```bash
ros2 launch yolo_bringup yolov11.launch.py use_3d:=True maximum_detection_threshold:=0.05
```
</details>

---

## **3. RVIZ Configuration**
To create a visualization, configure the following parameters:

| **Parameter**           | **Value**                                     |
|-------------------------|-----------------------------------------------|
| `Fixed Frame`           | `camera_link`                                |
| `Image Topic`           | `/yolo/dbg_image`                            |
| `PointClouds2 Topic`    | `/camera/camera/depth/color/points`          |
| `MarkerArray Topic`     | `/yolo/dgb_bb_markers`                       |


> [!NOTE]
> - Ensure these topics match the configuration to avoid misalignments.
> - You can always edit them in rviz and save them to an rviz file to be loaded.
> - The Rviz config file for yolo is in `yolo_ros/src/yolo_ros/yolo_bringup/rviz/yolo.rviz`

---

## **Tips and Recommendations**
- Regularly test the RealSense camera and YOLO ROS integration to ensure compatibility.
- Monitor system performance using commands like `top` or `nvidia-smi` to optimize resource usage.

---

## **Other Useful Commands**
Launch RealSense with specific configurations:
```bash
ros2 launch realsense2_camera rs_launch.py initial_reset:=true pointcloud.enable:=true rgb_camera.color_profile:=848x480x30
```
Launch YOLO with 3D detection and adjusted depth inputs:
```bash
ros2 launch yolo_bringup yolov11.launch.py use_3d:=True input_depth_topic:="/camera/camera/depth/image_rect_raw" input_depth_info_topic:="/camera/camera/depth/camera_info"
```

---


## **5. Live Demos**
Below are placeholders for demo videos showcasing the setup and functionality:

### **Demo 1: YOLO pick and place (all objects)**
Path: `/home/iras/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/demos/yolo_demos/yolo_pick_place.py`

[yolo pick and place](https://github.com/user-attachments/assets/d70c9688-d756-4792-b1cb-8a27c9746c45)

### **Demo 2: Live follow object**
Path: `/home/iras/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/demos/yolo_demos/live_follow.py`

[live follow](https://github.com/user-attachments/assets/5c856ebd-8666-4d57-9b7d-e06c957f0cc9)

### **Demo 3: YOLO pick and place by color**
Path: `/home/iras/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/demos/yolo_demos/color_pick_place.py`

[color pick and place](https://github.com/user-attachments/assets/8c594d41-111f-4873-812f-b732c657cfc2)

### **Demo 4: Debug of pointcloud and yolo systems**
Path: `/home/iras/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/demos/yolo_demos/debug.py`

[debug](https://github.com/user-attachments/assets/e1856100-eb18-4dc4-af27-744f6f22f7b7)

---

## **6. Troubleshooting**
Below are some common error messages, warnings, or issues you might see, and how to go about fixing them.

### **Camera lag/ low fps**
The QoS (Quality of Service) settings for the camera publisher and subscriber are mismatched.

   Use the following command to inspect the QoS settings for your camera topic:
   ```bash
   ros2 topic info -v /camera/camera/color/image_raw
   ```
   - Look for the **Reliability** setting under both the **Publisher** and **Subscriber** sections.
   - If the **Publisher** uses `RELIABLE` but the **Subscriber** (e.g., YOLO node) uses `BEST_EFFORT`, a mismatch occurs. Make sure they match.

### **3D BoudingBox stretching**

Try to reduce the `maximum_detection_threshold`. You can try values near your object box size.

---
