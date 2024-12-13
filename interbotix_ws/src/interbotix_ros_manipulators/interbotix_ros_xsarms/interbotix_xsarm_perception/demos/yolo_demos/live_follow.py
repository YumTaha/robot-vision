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
import time

# Robot Configurations
ROBOT_MODEL = 'vx300'
ROBOT_NAME = ROBOT_MODEL
REF_FRAME = 'camera_color_optical_frame'
ARM_TAG_FRAME = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'


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

        # self.armtag.find_ref_to_arm_base_transform()
        camera_base_trans = self.armtag.get_transform(
            self.armtag.tfBuffer, 
            target_frame=ARM_BASE_FRAME, 
            source_frame=REF_FRAME
        )

        for detection in msg.detections:
            bbox3d = detection.bbox3d
            x_cam, y_cam, z_cam = bbox3d.center.position.x, bbox3d.center.position.y, bbox3d.center.position.z

            # Transform coordinates
            xyz_unaligned = np.array([[0.015 - y_cam], [-z_cam], [x_cam], [1]])
            xyz_aligned = np.matmul(camera_base_trans, xyz_unaligned)

            # Append transformed coordinates
            self.detections.append({
                'x': round(xyz_aligned[0, 0], 3),
                'y': round(xyz_aligned[1, 0], 3),
                'z': round(xyz_aligned[2, 0], 3)
            })
    
    def live_follow_object(self):
        """Continuously follow the detected object."""
        self.get_logger().info("Starting live follow mode...")

        # self.bot.arm.set_trajectory_time(moving_time=0.5, accel_time=0.2)
        
        last_detection_time = time.time()  # Record current time

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if not self.detections:
                if time.time() - last_detection_time > 5:
                    self.get_logger().warn("\n--- No Objects Detected for 5 Seconds. Exiting Live Follow Mode. ---")
                    return  # Exit the function after 5 seconds of no detections
                
                self.get_logger().warn("No detections, holding position...")
                time.sleep(0.1)
                continue
            
            # Get the latest block position
            latest_detection = self.detections[0]
            x, y, z = latest_detection['x'], latest_detection['y'], latest_detection['z']
            last_detection_time = time.time()

            # Set the robot end-effector to the smoothed position
            self.bot.arm.set_ee_pose_components(
                x=x,
                y=y,
                z=z + 0.05,  # Slight offset above the block
                pitch=0.5,
                moving_time=1,
                # accel_time=0.2
                blocking=False
            )

            self.get_logger().info(f"Following: x={x:.3f}, y={y:.3f}, z={z:.3f}")


def main():
    rclpy.init()
    global_node = create_interbotix_global_node()

    try:
        bot = InterbotixManipulatorXS(robot_model=ROBOT_MODEL, robot_name=ROBOT_NAME, node=global_node)
        armtag = InterbotixArmTagInterface(
            ref_frame=REF_FRAME, 
            arm_tag_frame=ARM_TAG_FRAME, 
            arm_base_frame=ARM_BASE_FRAME, 
            node_inf=global_node
        )
        yolo_node = YoloPickPlace(bot, armtag)

        robot_startup(global_node)
        bot.arm.go_to_home_pose()
        bot.gripper.release()

        armtag.find_ref_to_arm_base_transform()

        # Start live follow mode
        print("\n--- Live Following Block ---")
        yolo_node.live_follow_object()

    finally:
        bot.arm.go_to_home_pose(moving_time=2)
        bot.arm.go_to_sleep_pose()
        robot_shutdown(global_node)
        if rclpy.ok():
            rclpy.shutdown()
        print("Pick and Place Node Shutdown.")

if __name__ == '__main__':
    main()
