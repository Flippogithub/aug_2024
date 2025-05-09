#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # For velocity commands
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ImageAnnotator(Node):
    def __init__(self):
        super().__init__('image_annotator')
        
        # Create a subscriber for the input image
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',  # Input image topic
            self.image_callback,
            10)
            
        # Create a subscriber for velocity commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel',  # Velocity commands topic
            self.cmd_vel_callback,
            10)
            
        # Create a publisher for the annotated image
        self.publisher = self.create_publisher(
            Image,
            '/annotated_image',  # Output topic
            10)
            
        self.bridge = CvBridge()
        
        # Store the latest velocity command
        self.latest_linear_x = 0.0
        self.latest_angular_z = 0.0
        
        # Counter for demonstration purposes
        self.counter = 0
        
        # Store the latest image (so we can annotate it with new velocity data)
        self.latest_image = None
        self.latest_image_header = None
        
        # Create sci-fi UI overlay elements
        self.create_ui_elements()
        
    def create_ui_elements(self):
        """Create sci-fi UI overlay elements"""
        # Create a futuristic HUD frame
        self.frame_size = (640, 480)  # Default size, will be resized to match camera
        self.create_hud_overlay()
        
    def create_hud_overlay(self):
        """Create a sci-fi HUD overlay"""
        # Create a transparent overlay with sci-fi elements
        self.hud_overlay = np.zeros((self.frame_size[1], self.frame_size[0], 4), dtype=np.uint8)
        
        # Draw a header area
        #cv2.rectangle(self.hud_overlay, (0, 0), (self.frame_size[0], 70), (20, 20, 40, 180), -1)
        
        # Draw corner brackets
        bracket_size = 50
        thickness = 2
        
        # Top-left bracket
        cv2.line(self.hud_overlay, (0, bracket_size), (0, 0), (0, 255, 255, 255), thickness)
        cv2.line(self.hud_overlay, (0, 0), (bracket_size, 0), (0, 255, 255, 255), thickness)
        
        # Top-right bracket
        cv2.line(self.hud_overlay, (self.frame_size[0]-bracket_size, 0), (self.frame_size[0], 0), (0, 255, 255, 255), thickness)
        cv2.line(self.hud_overlay, (self.frame_size[0], 0), (self.frame_size[0], bracket_size), (0, 255, 255, 255), thickness)
        
        # Bottom-left bracket
        cv2.line(self.hud_overlay, (0, self.frame_size[1]-bracket_size), (0, self.frame_size[1]), (0, 255, 255, 255), thickness)
        cv2.line(self.hud_overlay, (0, self.frame_size[1]), (bracket_size, self.frame_size[1]), (0, 255, 255, 255), thickness)
        
        # Bottom-right bracket
        cv2.line(self.hud_overlay, (self.frame_size[0]-bracket_size, self.frame_size[1]), (self.frame_size[0], self.frame_size[1]), (0, 255, 255, 255), thickness)
        cv2.line(self.hud_overlay, (self.frame_size[0], self.frame_size[1]-bracket_size), (self.frame_size[0], self.frame_size[1]), (0, 255, 255, 255), thickness)
        
        # Draw a semi-transparent data panel
        cv2.rectangle(self.hud_overlay, (20, 425), (400, 470), (20, 20, 40, 160), -1)
        #cv2.rectangle(self.hud_overlay, (20, 100), (300, 200), (0, 255, 255, 255), 1)
        
        # Crosshairs
        cv2.line(self.hud_overlay, (self.frame_size[0]//2, 0), (self.frame_size[0]//2, self.frame_size[1]), (0, 255, 255, 80), 1)
        cv2.line(self.hud_overlay, (0, self.frame_size[1]//2), (self.frame_size[0], self.frame_size[1]//2), (0, 255, 255, 80), 1)
        
        
        
    def cmd_vel_callback(self, msg):
        """Callback for velocity command messages"""
        # Store the latest velocity values
        self.latest_linear_x = msg.linear.x
        self.latest_angular_z = msg.angular.z
        
        # If we have an image, re-annotate and publish with the new velocity data
        if self.latest_image is not None:
            self.annotate_and_publish()
        
    def image_callback(self, msg):
        """Callback for camera image messages"""
        # Convert ROS Image message to OpenCV image
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image_header = msg.header
        
        # Check if we need to resize our overlay based on the image size
        if self.latest_image.shape[1] != self.frame_size[0] or self.latest_image.shape[0] != self.frame_size[1]:
            self.frame_size = (self.latest_image.shape[1], self.latest_image.shape[0])
            self.create_hud_overlay()
        
        # Annotate the image and publish
        self.annotate_and_publish()
    
    def overlay_ui(self, image, ui_overlay):
        """Overlay UI elements on the image with transparency"""
        # Resize overlay if needed
        if ui_overlay.shape[0] != image.shape[0] or ui_overlay.shape[1] != image.shape[1]:
            ui_overlay = cv2.resize(ui_overlay, (image.shape[1], image.shape[0]))
            
        # Create a 3-channel image with the same size as input
        result = image.copy()
        
        # Extract alpha channel
        alpha = ui_overlay[:, :, 3] / 255.0
        
        # Apply the overlay for each color channel
        for c in range(0, 3):
            result[:, :, c] = (1.0 - alpha) * result[:, :, c] + alpha * ui_overlay[:, :, c]
            
        return result
        
    def add_sharp_text(self, image, text, position, font_scale=0.5, color=(0, 255, 255), thickness=2):
        """Add sharp text directly to the image"""
        # Draw text with a black outline for better readability
        cv2.putText(
            image,
            text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 0),  # Black outline
            thickness + 1,
            cv2.LINE_AA
        )
        
        # Draw the text in the specified color
        cv2.putText(
            image,
            text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            color,
            thickness,
            cv2.LINE_AA
        )

        return image    
    
    def annotate_and_publish(self):
        """Annotate the latest image with all available data and publish it"""
        if self.latest_image is None:
            return
            
        # Create a copy to avoid modifying the original
        annotated_image = self.latest_image.copy()
        
        # Create a dynamic UI overlay that updates with data
        dynamic_overlay = self.hud_overlay.copy()


        
        # Visualize velocity with vector indicator
        center_x = self.frame_size[0] // 2
        center_y = self.frame_size[1] // 2
        
        # Scale for visualization
        linear_scale = 50  # pixels per m/s
        
        # Calculate arrow endpoint
        end_x = center_x + int(self.latest_linear_x * linear_scale)
        end_y = center_y
        
        # Draw dynamic elements on the overlay
        if abs(self.latest_linear_x) > 0.05 or abs(self.latest_angular_z) > 0.05:
            # Draw the vector arrow
            cv2.arrowedLine(
                dynamic_overlay,
                (center_x, center_y),
                (end_x, center_y),
                (0, 255, 255, 255),
                2
            )
            
            # Draw rotation indicator
            if abs(self.latest_angular_z) > 0.05:
                radius = int(abs(self.latest_angular_z) * 40)
                color = (0, 255, 255, 255) if self.latest_angular_z > 0 else (255, 0, 255, 255)
                cv2.circle(
                    dynamic_overlay,
                    (center_x, center_y),
                    max(radius, 20),
                    color,
                    2
                )
        # Velocity data - add directly to image for sharpness
        self.add_sharp_text(
            annotated_image,
            f"LINEAR: {self.latest_linear_x:.2f} m/s",
            (20, 440),
            0.3,  # Reduced font size
            (0, 255, 255),
            1
        )
        
        self.add_sharp_text(
            annotated_image,
            f"ANGULAR: {self.latest_angular_z:.2f} rad/s",
            (20, 460),
            0.3,  # Reduced font size
            (0, 255, 255),
            1
        )

        
        # Combine image with the dynamic overlay
        annotated_image = self.overlay_ui(annotated_image, dynamic_overlay)
        
         
        # Increment counter for next frame
        self.counter += 1
        
        # Convert back to ROS Image message and publish
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        annotated_msg.header = self.latest_image_header  # Preserve the original header
        self.publisher.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageAnnotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()