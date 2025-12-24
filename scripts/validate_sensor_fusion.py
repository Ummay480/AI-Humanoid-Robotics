#!/usr/bin/env python3
"""
Validation script for sensor fusion components.

This script validates the sensor fusion foundation by:
- Checking topic availability
- Confirming message flow
- Verifying fusion runs without error
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import Dict, Any, Optional


class SensorFusionValidator(Node):
    """
    Validation node for sensor fusion components.
    """

    def __init__(self):
        """Initialize the validation node."""
        super().__init__('sensor_fusion_validator')

        # Validation results
        self.validation_results = {
            'topics_available': False,
            'message_flow': False,
            'fusion_running': False,
            'errors': [],
            'warnings': [],
            'success_count': 0
        }

        # Subscriptions to validate
        self.topics_to_check = [
            '/perception/fused_state',
            '/perception/fusion_status'
        ]

        # Messages received
        self.received_messages: Dict[str, Any] = {}
        self.message_counts: Dict[str, int] = {}

        # Set up subscriptions
        for topic in self.topics_to_check:
            self.create_subscription(
                String,
                topic,
                lambda msg, t=topic: self._generic_callback(msg, t),
                10
            )
            self.message_counts[topic] = 0

        self.get_logger().info('Sensor Fusion Validator initialized')

    def _generic_callback(self, msg: String, topic: str):
        """
        Generic callback to capture messages from any topic.

        Args:
            msg: Message received
            topic: Topic name
        """
        try:
            # Try to parse as JSON to validate format
            data = json.loads(msg.data)
            self.received_messages[topic] = data
            self.message_counts[topic] = self.message_counts.get(topic, 0) + 1

            self.get_logger().debug(f'Received message on {topic}, count: {self.message_counts[topic]}')

            # Validate message structure based on topic
            if topic == '/perception/fused_state':
                self._validate_fused_state_message(data)
            elif topic == '/perception/fusion_status':
                self._validate_fusion_status_message(data)

        except json.JSONDecodeError:
            self.validation_results['errors'].append(f'Invalid JSON on topic {topic}')
        except Exception as e:
            self.validation_results['errors'].append(f'Error processing message on {topic}: {e}')

    def _validate_fused_state_message(self, data: Dict[str, Any]):
        """
        Validate fused state message structure.

        Args:
            data: Parsed JSON data
        """
        required_fields = ['id', 'timestamp', 'fused_payload', 'source_sensor_ids', 'confidence_score', 'coordinate_frame']

        for field in required_fields:
            if field not in data:
                self.validation_results['errors'].append(f'Missing field {field} in fused_state message')

        if 'confidence_score' in data and not (0.0 <= data['confidence_score'] <= 1.0):
            self.validation_results['warnings'].append(f'Confidence score out of range: {data["confidence_score"]}')

    def _validate_fusion_status_message(self, data: Dict[str, Any]):
        """
        Validate fusion status message structure.

        Args:
            data: Parsed JSON data
        """
        required_fields = ['node', 'timestamp', 'fusion_stats', 'latest_sensor_data', 'overall_status']

        for field in required_fields:
            if field not in data:
                self.validation_results['errors'].append(f'Missing field {field} in fusion_status message')

        if 'overall_status' in data and data['overall_status'] not in ['OK', 'WARNING', 'DEGRADED']:
            self.validation_results['warnings'].append(f'Invalid overall_status: {data["overall_status"]}')

    def run_validation(self, timeout: float = 10.0) -> Dict[str, Any]:
        """
        Run the validation process.

        Args:
            timeout: Validation timeout in seconds

        Returns:
            Dictionary with validation results
        """
        self.get_logger().info(f'Running sensor fusion validation for {timeout} seconds...')

        start_time = time.time()
        last_message_count = 0

        while time.time() - start_time < timeout:
            current_total = sum(self.message_counts.values())

            # Check if messages are flowing
            if current_total > last_message_count:
                self.validation_results['message_flow'] = True
                self.validation_results['success_count'] += 1
                last_message_count = current_total

            # Check if fusion is running (we can see status messages)
            if self.message_counts.get('/perception/fusion_status', 0) > 0:
                self.validation_results['fusion_running'] = True

            # Check if topics are available by seeing if we receive any messages
            if any(count > 0 for count in self.message_counts.values()):
                self.validation_results['topics_available'] = True

            time.sleep(0.1)

        # Final validation results
        results = self.validation_results.copy()
        results['final_message_counts'] = self.message_counts.copy()
        results['validation_time'] = timeout

        return results

    def print_validation_report(self, results: Dict[str, Any]):
        """
        Print a formatted validation report.

        Args:
            results: Validation results dictionary
        """
        print("\n" + "="*60)
        print("SENSOR FUSION VALIDATION REPORT")
        print("="*60)

        print(f"Validation Time: {results['validation_time']} seconds")
        print(f"Messages Received: {sum(results['final_message_counts'].values())}")
        print(f"Success Count: {results['success_count']}")

        print("\nTopic Availability:")
        for topic, count in results['final_message_counts'].items():
            status = "✓ Available" if count > 0 else "✗ Not Available"
            print(f"  {topic}: {status} ({count} messages)")

        print(f"\nCore Validation:")
        print(f"  Topics Available: {'✓' if results['topics_available'] else '✗'}")
        print(f"  Message Flow: {'✓' if results['message_flow'] else '✗'}")
        print(f"  Fusion Running: {'✓' if results['fusion_running'] else '✗'}")

        if results['errors']:
            print(f"\nErrors ({len(results['errors'])}):")
            for error in results['errors']:
                print(f"  ❌ {error}")

        if results['warnings']:
            print(f"\nWarnings ({len(results['warnings'])}):")
            for warning in results['warnings']:
                print(f"  ⚠️  {warning}")

        # Overall status
        all_checks_passed = all([
            results['topics_available'],
            results['message_flow'],
            results['fusion_running'],
            not results['errors']
        ])

        print(f"\nOverall Status: {'✅ PASSED' if all_checks_passed else '❌ FAILED'}")
        print("="*60)


def main(args=None):
    """Main entry point for the validation script."""
    rclpy.init(args=args)

    validator = SensorFusionValidator()

    try:
        # Run validation for 10 seconds
        results = validator.run_validation(timeout=10.0)
        validator.print_validation_report(results)

        # Exit with appropriate code
        all_checks_passed = all([
            results['topics_available'],
            results['message_flow'],
            results['fusion_running'],
            not results['errors']
        ])

        return 0 if all_checks_passed else 1

    except KeyboardInterrupt:
        validator.get_logger().info('Validation interrupted by user')
        return 0
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    exit_code = main()
    exit(exit_code)