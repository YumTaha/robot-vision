#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from yolo_msgs.msg import DetectionArray
import numpy as np

# Robot Configurations
ROBOT_MODEL = 'vx300'
ROBOT_NAME = ROBOT_MODEL
REF_FRAME = 'camera_color_optical_frame'
ARM_TAG_FRAME = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'


class DebugNode(Node):
    def __init__(self, armtag, pcl):
        super().__init__('debug_node')
        self.armtag: InterbotixArmTagInterface = armtag
        self.pcl: InterbotixPointCloudInterface = pcl
        self.detections: list = []
        
        # YOLO Detections Subscriber
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.yolo_callback,
            10
        )
        self.get_logger().info("Debug Node Initialized: Listening for YOLO detections...")

    def yolo_callback(self, msg):
        """Callback to process YOLO 3D detections."""
        self.detections.clear()  # Clear previous detections

        self.armtag.find_ref_to_arm_base_transform()
        # Retrieve the transform between camera and base frame
        # if not self.armtag.find_ref_to_arm_base_transform():
        #     self.get_logger().warn("Failed to find ArmTag transform.")
        #     return
        
        camera_base_trans = self.armtag.get_transform(
            self.armtag.tfBuffer,
            target_frame=ARM_BASE_FRAME,
            source_frame=REF_FRAME
        )

        for detection in msg.detections:
            class_name = detection.class_name
            bbox3d = detection.bbox3d
            x_cam, y_cam, z_cam = bbox3d.center.position.x, bbox3d.center.position.y, bbox3d.center.position.z

            # Transform coordinates
            xyz_unaligned = np.array([[0.015 - y_cam], [-z_cam], [x_cam], [1]])
            xyz_aligned = np.matmul(camera_base_trans, xyz_unaligned)

            # Format and log the unaligned detection
            x, y, z = xyz_unaligned[:3, 0]
            self.get_logger().info(
                f"YOLO Detection Unaligned: {class_name}, x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

            # Append transformed coordinates
            self.detections.append({
                'class_name': class_name,
                'x': round(xyz_aligned[0, 0], 3),
                'y': round(xyz_aligned[1, 0], 3),
                'z': round(xyz_aligned[2, 0], 3)
            })

    def get_pointcloud_clusters(self):
        """Retrieve and log cluster positions using PointCloudInterface."""
        success, clusters = self.pcl.get_cluster_positions(
            ref_frame=ARM_BASE_FRAME, sort_axis='x', reverse=True
        )
        if success:
            for i, cluster in enumerate(clusters):
                x, y, z = cluster['position']
                self.get_logger().info(f"PointCloud Cluster {i + 1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        else:
            self.get_logger().warn("No PointCloud clusters found.")


def main():
    rclpy.init()
    global_node = create_interbotix_global_node()

    try:
        # Initialize interfaces
        armtag = InterbotixArmTagInterface(
            ref_frame=REF_FRAME,
            arm_tag_frame=ARM_TAG_FRAME,
            arm_base_frame=ARM_BASE_FRAME,
            node_inf=global_node,
        )
        pcl = InterbotixPointCloudInterface(node_inf=global_node)
        debug_node = DebugNode(armtag, pcl)

        # Start the robot
        robot_startup(global_node)

        print("\n--- Retrieving PointCloud Positions using Interbotix ---")
        debug_node.get_pointcloud_clusters()

        print("\n--- Waiting for YOLO Detections ---")
        for _ in range(20):  # ~10 seconds
            rclpy.spin_once(debug_node, timeout_sec=0.5)
            if debug_node.detections:
                break

        # # Log transformed YOLO detections
        if debug_node.detections:
            print("\n--- Transformed YOLO Detections ---")
            for i, detection in enumerate(debug_node.detections):
                print(f"YOLO Detection {i + 1}: {detection}")
        else:
            print("No YOLO detections received.")

    finally:
        robot_shutdown(global_node)
        if rclpy.ok():
            rclpy.shutdown()
        print("Debug Node Shutdown.")


if __name__ == '__main__':
    main()
