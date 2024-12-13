#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from yolo_msgs.msg import DetectionArray
import numpy as np

# Robot Configurations
ROBOT_MODEL = 'vx300'
ROBOT_NAME = ROBOT_MODEL
REF_FRAME = 'camera_color_optical_frame'
ARM_TAG_FRAME = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'

# Global Variable for Target Color
COLOR = "red"  # Specify the color to pick (e.g., "red", "blue", "green")

class YoloPickPlace(Node):
    def __init__(self, bot, armtag):
        super().__init__('yolo_pick_place')
        self.bot: InterbotixManipulatorXS = bot
        self.armtag: InterbotixArmTagInterface = armtag
        self.detections = []

        # YOLO Detections Subscriber
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.yolo_callback,
            10
        )
        self.get_logger().info("Listening for YOLO detections...")

    def yolo_callback(self, msg):
        """Callback to process YOLO detections and transform coordinates."""
        self.detections.clear()  # Clear previous detections

        # Retrieve the camera-to-arm transform
        self.armtag.find_ref_to_arm_base_transform()
        camera_base_trans = self.armtag.get_transform(
            self.armtag.tfBuffer, 
            target_frame=ARM_BASE_FRAME, 
            source_frame=REF_FRAME
        )

        for detection in msg.detections:
            class_name = detection.class_name
            if class_name != COLOR:  # Only process detections matching the global COLOR
                continue

            bbox3d = detection.bbox3d
            x_cam, y_cam, z_cam = bbox3d.center.position.x, bbox3d.center.position.y, bbox3d.center.position.z

            # Transform coordinates
            xyz_unaligned = np.array([[0.015 - y_cam], [-z_cam], [x_cam], [1]])
            xyz_aligned = np.matmul(camera_base_trans, xyz_unaligned)

            # Append transformed coordinates
            self.detections.append({
                'class_name': class_name,
                'x': round(xyz_aligned[0, 0], 3),
                'y': round(xyz_aligned[1, 0], 3),
                'z': round(xyz_aligned[2, 0], 3)
            })
        self.get_logger().info("Filtered YOLO Detections Transformed and Ready.")

    def pick_objects(self):
        """Pick and place objects based on YOLO detections."""
        if not self.detections:
            self.get_logger().warn(f"No '{COLOR}' objects detected.")
            return

        for detection in self.detections:
            x, y, z = detection['x'], detection['y'], detection['z']
            self.get_logger().info(f"Picking '{detection['class_name']}' object at x={x}, y={y}, z={z}")

            # Move arm above the object
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.1, pitch=0.5)
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.03, pitch=0.5)
            
            # Grasp the object
            self.bot.gripper.grasp()
            self.bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.1, pitch=0.5)

            # Place the object in a predefined location
            self.bot.arm.set_ee_pose_components(x=-0.3, y=-0.2, z=0.2)
            self.bot.gripper.release()

def main():
    rclpy.init()
    global_node = create_interbotix_global_node()

    try:
        # Initialize robot interfaces
        bot = InterbotixManipulatorXS(robot_model=ROBOT_MODEL, robot_name=ROBOT_NAME, node=global_node)
        armtag = InterbotixArmTagInterface(
            ref_frame=REF_FRAME, 
            arm_tag_frame=ARM_TAG_FRAME, 
            arm_base_frame=ARM_BASE_FRAME, 
            node_inf=global_node
        )
        yolo_node = YoloPickPlace(bot, armtag)

        # Start robot
        robot_startup(global_node)

        # Set initial arm and gripper pose
        bot.arm.go_to_home_pose()
        bot.arm.go_to_sleep_pose()
        bot.gripper.release()
        yolo_node.armtag.find_ref_to_arm_base_transform()
        bot.arm.set_ee_pose_components(x=0.3, z=0.2)

        # Wait for YOLO detections
        print(f"\n--- Waiting for '{COLOR}' YOLO Detections ---")
        for _ in range(20):  # Timeout ~10 seconds
            rclpy.spin_once(yolo_node, timeout_sec=0.5)
            if yolo_node.detections:
                break

        # Pick and place objects of specified color
        print(f"\n--- Picking '{COLOR}' Objects Based on YOLO Detections ---")
        yolo_node.pick_objects()

    finally:
        bot.arm.go_to_home_pose()
        bot.arm.go_to_sleep_pose()
        robot_shutdown(global_node)
        if rclpy.ok():
            rclpy.shutdown()
        print("Pick and Place Node Shutdown.")

if __name__ == '__main__':
    main()
