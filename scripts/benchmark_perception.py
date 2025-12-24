#!/usr/bin/env python3
"""
Perception Performance Benchmarking Script
Implements T017: Implement Performance Benchmarking (90%+ Accuracy)
"""
import os
import sys
import numpy as np
import cv2
import json
from typing import Dict, List, Tuple, Any
from pathlib import Path
import argparse
from datetime import datetime

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.isaac_ros.perception.detection_system import ObjectDetectionSystem
from src.isaac_ros.perception.pipeline import PerceptionPipelineOrchestrator
from src.common.models.perception_models import create_default_perception_pipeline


class PerceptionBenchmark:
    """
    Benchmarking system for perception accuracy and performance.
    Validates perception accuracy against ground truth labels.
    """

    def __init__(self, ground_truth_path: str = None):
        """
        Initialize the benchmark system.

        Args:
            ground_truth_path: Path to ground truth labels
        """
        self.ground_truth_path = ground_truth_path
        self.detection_system = ObjectDetectionSystem(confidence_threshold=0.5)
        self.results = {
            'benchmark_start_time': datetime.now().isoformat(),
            'total_samples': 0,
            'detection_results': [],
            'accuracy_metrics': {},
            'performance_metrics': {},
            'benchmark_config': {
                'confidence_threshold': self.detection_system.confidence_threshold,
                'iou_threshold': 0.5,
                'object_classes': self.detection_system.object_classes
            }
        }

    def load_ground_truth(self, ground_truth_path: str) -> Dict[str, Any]:
        """
        Load ground truth labels for benchmarking.

        Args:
            ground_truth_path: Path to ground truth JSON file

        Returns:
            Dictionary containing ground truth data
        """
        if not ground_truth_path or not os.path.exists(ground_truth_path):
            print(f"Ground truth file not found: {ground_truth_path}")
            print("Generating synthetic ground truth for testing...")
            return self._generate_synthetic_ground_truth()

        with open(ground_truth_path, 'r') as f:
            return json.load(f)

    def _generate_synthetic_ground_truth(self, num_samples: int = 100) -> Dict[str, Any]:
        """
        Generate synthetic ground truth data for testing.

        Args:
            num_samples: Number of samples to generate

        Returns:
            Dictionary with synthetic ground truth
        """
        ground_truth = {
            'samples': [],
            'object_classes': self.detection_system.object_classes
        }

        for i in range(num_samples):
            sample = {
                'image_id': f'synthetic_{i:06d}',
                'width': 640,
                'height': 480,
                'objects': []
            }

            # Generate random objects for this sample
            num_objects = np.random.randint(1, 5)  # 1-4 objects per image
            for _ in range(num_objects):
                class_id = np.random.choice(list(self.detection_system.object_classes.keys())[1:])  # Skip background
                class_name = self.detection_system.object_classes[class_id]

                # Generate random bounding box
                x = np.random.randint(0, 500)
                y = np.random.randint(0, 380)
                width = np.random.randint(50, 150)
                height = np.random.randint(50, 150)

                sample['objects'].append({
                    'class_id': class_id,
                    'class_name': class_name,
                    'bbox': [x, y, x + width, y + height],
                    'area': width * height
                })

            ground_truth['samples'].append(sample)

        return ground_truth

    def calculate_iou(self, bbox1: List[int], bbox2: List[int]) -> float:
        """
        Calculate Intersection over Union between two bounding boxes.

        Args:
            bbox1: First bounding box [x_min, y_min, x_max, y_max]
            bbox2: Second bounding box [x_min, y_min, x_max, y_max]

        Returns:
            IoU value
        """
        x1_min, y1_min, x1_max, y1_max = bbox1
        x2_min, y2_min, x2_max, y2_max = bbox2

        # Calculate intersection area
        inter_x_min = max(x1_min, x2_min)
        inter_y_min = max(y1_min, y2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_max = min(y1_max, y2_max)

        if inter_x_max < inter_x_min or inter_y_max < inter_y_min:
            return 0.0

        inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)

        # Calculate union area
        area1 = (x1_max - x1_min) * (y1_max - y1_min)
        area2 = (x2_max - x2_min) * (y2_max - y2_min)
        union_area = area1 + area2 - inter_area

        return inter_area / union_area if union_area > 0 else 0.0

    def calculate_precision_recall_f1(self, detections: List, ground_truth: List, iou_threshold: float = 0.5) -> Dict[str, float]:
        """
        Calculate precision, recall, and F1 metrics.

        Args:
            detections: List of detected objects
            ground_truth: List of ground truth objects
            iou_threshold: IoU threshold for matching

        Returns:
            Dictionary with precision, recall, and F1 scores
        """
        if len(ground_truth) == 0:
            # If no ground truth, precision is 1 if no detections, 0 if there are detections
            return {
                'precision': 1.0 if len(detections) == 0 else 0.0,
                'recall': 1.0 if len(detections) == 0 else 0.0,
                'f1_score': 1.0 if len(detections) == 0 else 0.0
            }

        if len(detections) == 0:
            # If no detections but there's ground truth, all metrics are 0
            return {
                'precision': 0.0,
                'recall': 0.0,
                'f1_score': 0.0
            }

        # Match detections to ground truth
        matched = [False] * len(ground_truth)
        true_positives = 0

        for detection in detections:
            best_match_idx = -1
            best_iou = 0.0

            for i, gt_obj in enumerate(ground_truth):
                if matched[i]:
                    continue

                iou = self.calculate_iou(detection.bbox, gt_obj['bbox'])
                if iou > best_iou and iou >= iou_threshold:
                    best_iou = iou
                    best_match_idx = i

            if best_match_idx != -1:
                matched[best_match_idx] = True
                true_positives += 1

        false_positives = len(detections) - true_positives
        false_negatives = len(ground_truth) - true_positives

        precision = true_positives / (true_positives + false_positives) if (true_positives + false_positives) > 0 else 0.0
        recall = true_positives / (true_positives + false_negatives) if (true_positives + false_negatives) > 0 else 0.0
        f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0

        return {
            'precision': precision,
            'recall': recall,
            'f1_score': f1_score
        }

    def calculate_mean_average_precision(self, all_detections: List, all_ground_truth: List, iou_threshold: float = 0.5) -> float:
        """
        Calculate mean Average Precision (mAP).

        Args:
            all_detections: List of all detections across all samples
            all_ground_truth: List of all ground truth across all samples
            iou_threshold: IoU threshold for matching

        Returns:
            mAP score
        """
        if len(all_detections) == 0 or len(all_ground_truth) == 0:
            return 0.0

        # For simplicity, we'll calculate a simplified version of mAP
        # In a real implementation, this would be more complex
        total_precision = 0.0
        total_samples = len(all_detections)

        for detections, ground_truth in zip(all_detections, all_ground_truth):
            metrics = self.calculate_precision_recall_f1(detections, ground_truth, iou_threshold)
            total_precision += metrics['precision']

        return total_precision / total_samples if total_samples > 0 else 0.0

    def run_benchmark(self, num_samples: int = 100) -> Dict[str, Any]:
        """
        Run the complete perception benchmark.

        Args:
            num_samples: Number of samples to benchmark

        Returns:
            Dictionary with benchmark results
        """
        print(f"Starting perception benchmark with {num_samples} samples...")

        # Load ground truth data
        ground_truth_data = self.load_ground_truth(self.ground_truth_path)
        samples = ground_truth_data['samples']

        # Limit to requested number of samples
        samples = samples[:num_samples]
        self.results['total_samples'] = len(samples)

        all_detections = []
        all_ground_truth = []
        detection_times = []

        for i, sample in enumerate(samples):
            if i % 10 == 0:
                print(f"Processing sample {i+1}/{len(samples)}...")

            # Generate synthetic image for testing
            image = self._generate_synthetic_image(sample)

            # Time the detection
            start_time = cv2.getTickCount()
            detections = self.detection_system.detect_objects(image)
            end_time = cv2.getTickCount()
            detection_time = (end_time - start_time) / cv2.getTickFrequency()
            detection_times.append(detection_time)

            # Store results
            all_detections.append(detections)
            all_ground_truth.append(sample['objects'])

            # Calculate metrics for this sample
            sample_metrics = self.calculate_precision_recall_f1(
                detections, sample['objects'], self.results['benchmark_config']['iou_threshold']
            )
            sample_metrics['detection_time'] = detection_time
            sample_metrics['num_detections'] = len(detections)
            sample_metrics['num_ground_truth'] = len(sample['objects'])

            self.results['detection_results'].append({
                'sample_id': sample['image_id'],
                'metrics': sample_metrics
            })

        # Calculate overall metrics
        total_precision = sum([r['metrics']['precision'] for r in self.results['detection_results']])
        total_recall = sum([r['metrics']['recall'] for r in self.results['detection_results']])
        total_f1 = sum([r['metrics']['f1_score'] for r in self.results['detection_results']])

        overall_precision = total_precision / len(self.results['detection_results']) if self.results['detection_results'] else 0
        overall_recall = total_recall / len(self.results['detection_results']) if self.results['detection_results'] else 0
        overall_f1 = total_f1 / len(self.results['detection_results']) if self.results['detection_results'] else 0

        # Calculate mAP
        mean_ap = self.calculate_mean_average_precision(all_detections, all_ground_truth)

        # Calculate performance metrics
        avg_detection_time = np.mean(detection_times) if detection_times else 0
        fps = 1.0 / avg_detection_time if avg_detection_time > 0 else 0

        # Store accuracy metrics
        self.results['accuracy_metrics'] = {
            'overall_precision': overall_precision,
            'overall_recall': overall_recall,
            'overall_f1_score': overall_f1,
            'mean_average_precision': mean_ap,
            'accuracy_percentage': overall_f1 * 100,  # Convert to percentage
            'per_class_metrics': self._calculate_per_class_metrics(all_detections, all_ground_truth)
        }

        # Store performance metrics
        self.results['performance_metrics'] = {
            'average_detection_time_ms': avg_detection_time * 1000,
            'frames_per_second': fps,
            'total_processing_time': sum(detection_times),
            'gpu_acceleration': False,  # Would be true in real implementation
            'model_inference_time': avg_detection_time  # In real implementation, this would be actual inference time
        }

        self.results['benchmark_end_time'] = datetime.now().isoformat()

        # Check if accuracy meets requirements
        accuracy_met = self.results['accuracy_metrics']['accuracy_percentage'] >= 90
        print(f"Accuracy: {self.results['accuracy_metrics']['accuracy_percentage']:.2f}%")
        print(f"Target: 90%+")
        print(f"Requirement met: {'YES' if accuracy_met else 'NO'}")

        return self.results

    def _generate_synthetic_image(self, sample: Dict[str, Any]) -> np.ndarray:
        """
        Generate a synthetic image based on the sample ground truth.

        Args:
            sample: Sample data containing object information

        Returns:
            Synthetic image
        """
        height, width = 480, 640
        image = np.random.randint(100, 150, (height, width, 3), dtype=np.uint8)

        # Add some random background elements
        for _ in range(5):
            x = np.random.randint(0, width)
            y = np.random.randint(0, height)
            radius = np.random.randint(10, 30)
            color = [np.random.randint(50, 100), np.random.randint(50, 100), np.random.randint(50, 100)]
            cv2.circle(image, (x, y), radius, color, -1)

        # Add objects from ground truth
        for obj in sample['objects']:
            x_min, y_min, x_max, y_max = obj['bbox']
            center_x = (x_min + x_max) // 2
            center_y = (y_min + y_max) // 2
            obj_width = x_max - x_min
            obj_height = y_max - y_min

            # Generate color based on class
            class_id = obj['class_id']
            color = self._get_color_for_class(class_id)

            # Draw object based on its type (simplified)
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, -1)

        return image

    def _get_color_for_class(self, class_id: int) -> List[int]:
        """
        Get a color for a given class ID.

        Args:
            class_id: Class ID

        Returns:
            RGB color as [R, G, B] list
        """
        colors = [
            [255, 0, 0],    # Red for human
            [0, 255, 0],    # Green for chair
            [0, 0, 255],    # Blue for table
            [255, 255, 0],  # Cyan for door
            [255, 0, 255],  # Magenta for stair
            [0, 255, 255],  # Yellow for obstacle
            [128, 0, 128],  # Purple for robot
            [255, 165, 0],  # Orange for box
            [0, 128, 0]     # Dark green for plant
        ]
        return colors[class_id % len(colors)]

    def _calculate_per_class_metrics(self, all_detections: List, all_ground_truth: List) -> Dict[str, Dict[str, float]]:
        """
        Calculate metrics for each object class.

        Args:
            all_detections: List of all detections
            all_ground_truth: List of all ground truth

        Returns:
            Dictionary with per-class metrics
        """
        per_class_metrics = {}

        # Get all unique class IDs
        all_class_ids = set()
        for gt_list in all_ground_truth:
            for obj in gt_list:
                all_class_ids.add(obj['class_id'])

        for class_id in all_class_ids:
            class_name = self.detection_system.object_classes.get(class_id, f"unknown_{class_id}")
            class_detections = []
            class_ground_truth = []

            # Filter detections and ground truth for this class
            for detections, ground_truth in zip(all_detections, all_ground_truth):
                class_det = [d for d in detections if d.class_id == class_id]
                class_gt = [obj for obj in ground_truth if obj['class_id'] == class_id]

                class_detections.append(class_det)
                class_ground_truth.append(class_gt)

            # Calculate metrics for this class
            if class_ground_truth:
                total_precision = 0
                total_recall = 0
                total_f1 = 0
                valid_samples = 0

                for det, gt in zip(class_detections, class_ground_truth):
                    if len(gt) > 0 or len(det) > 0:  # Only calculate if there's something to evaluate
                        metrics = self.calculate_precision_recall_f1(
                            det, gt, self.results['benchmark_config']['iou_threshold']
                        )
                        total_precision += metrics['precision']
                        total_recall += metrics['recall']
                        total_f1 += metrics['f1_score']
                        valid_samples += 1

                if valid_samples > 0:
                    per_class_metrics[class_name] = {
                        'precision': total_precision / valid_samples,
                        'recall': total_recall / valid_samples,
                        'f1_score': total_f1 / valid_samples,
                        'support': sum([len(gt) for gt in class_ground_truth])  # Total ground truth instances
                    }

        return per_class_metrics

    def generate_report(self, output_path: str = None) -> str:
        """
        Generate a benchmark report.

        Args:
            output_path: Path to save the report (optional)

        Returns:
            Report as string
        """
        report = []
        report.append("=" * 60)
        report.append("PERCEPTION BENCHMARK REPORT")
        report.append("=" * 60)
        report.append(f"Start Time: {self.results['benchmark_start_time']}")
        report.append(f"End Time: {self.results['benchmark_end_time']}")
        report.append(f"Total Samples: {self.results['total_samples']}")
        report.append("")

        # Accuracy metrics
        report.append("ACCURACY METRICS:")
        report.append("-" * 20)
        acc_metrics = self.results['accuracy_metrics']
        report.append(f"Overall Precision: {acc_metrics['overall_precision']:.4f}")
        report.append(f"Overall Recall: {acc_metrics['overall_recall']:.4f}")
        report.append(f"Overall F1 Score: {acc_metrics['overall_f1_score']:.4f}")
        report.append(f"Mean Average Precision: {acc_metrics['mean_average_precision']:.4f}")
        report.append(f"Accuracy Percentage: {acc_metrics['accuracy_percentage']:.2f}%")
        report.append(f"Target: 90%+ Accuracy: {'PASS' if acc_metrics['accuracy_percentage'] >= 90 else 'FAIL'}")
        report.append("")

        # Performance metrics
        report.append("PERFORMANCE METRICS:")
        report.append("-" * 20)
        perf_metrics = self.results['performance_metrics']
        report.append(f"Average Detection Time: {perf_metrics['average_detection_time_ms']:.2f} ms")
        report.append(f"Frames Per Second: {perf_metrics['frames_per_second']:.2f} FPS")
        report.append(f"Total Processing Time: {perf_metrics['total_processing_time']:.2f} s")
        report.append("")

        # Per-class metrics
        report.append("PER-CLASS METRICS:")
        report.append("-" * 20)
        for class_name, metrics in acc_metrics['per_class_metrics'].items():
            report.append(f"{class_name}:")
            report.append(f"  Precision: {metrics['precision']:.4f}")
            report.append(f"  Recall: {metrics['recall']:.4f}")
            report.append(f"  F1 Score: {metrics['f1_score']:.4f}")
            report.append(f"  Support: {metrics['support']}")
        report.append("")

        # Summary
        report.append("SUMMARY:")
        report.append("-" * 10)
        accuracy_pass = acc_metrics['accuracy_percentage'] >= 90
        fps_pass = perf_metrics['frames_per_second'] >= 30  # Assuming 30 FPS requirement
        report.append(f"Accuracy Requirement (90%+): {'PASS' if accuracy_pass else 'FAIL'}")
        report.append(f"Performance Requirement (30+ FPS): {'PASS' if fps_pass else 'FAIL'}")
        report.append(f"Overall Benchmark Result: {'PASS' if accuracy_pass and fps_pass else 'FAIL'}")

        report_str = "\n".join(report)

        if output_path:
            with open(output_path, 'w') as f:
                f.write(report_str)
            print(f"Benchmark report saved to: {output_path}")

        return report_str


def main():
    parser = argparse.ArgumentParser(description='Perception Benchmarking Tool')
    parser.add_argument('--ground-truth', type=str, help='Path to ground truth JSON file')
    parser.add_argument('--num-samples', type=int, default=100, help='Number of samples to benchmark')
    parser.add_argument('--output', type=str, default='perception_benchmark_report.txt', help='Output report file')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')

    args = parser.parse_args()

    # Initialize benchmark
    benchmark = PerceptionBenchmark(ground_truth_path=args.ground_truth)

    # Run benchmark
    results = benchmark.run_benchmark(num_samples=args.num_samples)

    # Generate and print report
    report = benchmark.generate_report(output_path=args.output)
    print(report)

    # Exit with appropriate code based on results
    accuracy_pass = results['accuracy_metrics']['accuracy_percentage'] >= 90
    fps_pass = results['performance_metrics']['frames_per_second'] >= 30
    overall_pass = accuracy_pass and fps_pass

    print(f"\nBenchmark completed. Overall result: {'PASS' if overall_pass else 'FAIL'}")
    sys.exit(0 if overall_pass else 1)


if __name__ == "__main__":
    main()