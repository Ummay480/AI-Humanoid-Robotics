#!/usr/bin/env python3
"""
Simple Object Detection Example

Demonstrates basic object detection using the perception module.
Subscribes to camera feed and publishes detected objects.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.perception.computer_vision.object_detector import YOLODetector


class SimpleDetectionNode(Node):
    """Simple object detection node."""

    def __init__(self):
        super().__init__('simple_detection')

        # Declare parameters
        self.declare_parameter('model_path', 'models/yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_topic', '/camera_front/image_raw')
        self.declare_parameter('output_topic', '/detections')

        # Get parameters
        model_path = self.get_parameter('model_path').value
        confidence = self.get_parameter('confidence_threshold').value
        camera_topic = self.get_parameter('camera_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Create detector
        self.get_logger().info(f'Loading YOLO model from {model_path}...')
        self.detector = YOLODetector(
            model_path=model_path,
            confidence_threshold=confidence
        )
        self.get_logger().info('YOLO model loaded successfully')

        # Setup ROS 2
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            output_topic,
            10
        )

        self.detection_count = 0
        self.get_logger().info(f'Subscribed to {camera_topic}')
        self.get_logger().info(f'Publishing to {output_topic}')

    def image_callback(self, msg):
        """Process incoming images."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Run detection
            detections = self.detector.detect(cv_image)

            # Publish results
            results = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame_id': msg.header.frame_id,
                'count': len(detections),
                'objects': [
                    {
                        'class': det.object_type.value,
                        'confidence': float(det.confidence),
                        'position': det.position.tolist() if det.position is not None else None,
                        'bbox': det.bbox.tolist() if det.bbox is not None else None
                    }
                    for det in detections
                ]
            }

            msg_out = String()
            msg_out.data = json.dumps(results)
            self.publisher.publish(msg_out)

            self.detection_count += 1
            if self.detection_count % 30 == 0:  # Log every second at 30 FPS
                self.get_logger().info(
                    f'Processed {self.detection_count} frames, '
                    f'current detections: {len(detections)}'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = SimpleDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
