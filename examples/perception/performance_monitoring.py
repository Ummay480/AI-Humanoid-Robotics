#!/usr/bin/env python3
"""
Performance Monitoring Example

Demonstrates how to monitor and optimize perception pipeline performance.
Shows logging, metrics collection, health monitoring, and optimization.
"""

import time
import numpy as np
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.perception.monitoring.logger import (
    PerceptionLogger,
    PerformanceMonitor,
    HealthMonitor,
    TimingContext,
    MetricsCollector
)
from src.perception.monitoring.performance_optimizer import (
    PerformanceOptimizer,
    LatencyOptimizer,
    MemoryOptimizer,
    ResourceManager
)


def simulate_sensor_processing(duration_ms):
    """Simulate sensor processing with variable duration."""
    time.sleep(duration_ms / 1000.0)


def simulate_detection_processing(num_detections):
    """Simulate object detection processing."""
    # Simulate YOLO processing time (depends on number of objects)
    base_time = 10  # ms
    per_object_time = 0.5  # ms
    total_time = base_time + num_detections * per_object_time
    time.sleep(total_time / 1000.0)
    return total_time


def main():
    """Run performance monitoring demonstration."""
    print("=" * 70)
    print("Performance Monitoring Demonstration")
    print("=" * 70)

    # 1. Setup monitoring infrastructure
    print("\n1. Setting up monitoring infrastructure...")

    # Create logger
    logger = PerceptionLogger('perf_demo', log_dir='logs', log_level='INFO')
    logger.info("Performance monitoring demo started")

    # Create monitors
    perf_monitor = PerformanceMonitor(window_size=1000)
    health_monitor = HealthMonitor()
    metrics_collector = MetricsCollector(output_dir='metrics')

    # Create optimizer
    optimizer = PerformanceOptimizer()

    print("   ✓ Logger configured")
    print("   ✓ Performance monitor ready")
    print("   ✓ Health monitor ready")
    print("   ✓ Performance optimizer ready")

    # 2. Simulate perception pipeline
    print("\n2. Simulating perception pipeline (100 iterations)...")

    num_iterations = 100
    for i in range(num_iterations):
        # Sensor acquisition
        with TimingContext('sensor_acquisition', perf_monitor, logger):
            simulate_sensor_processing(5.0)  # 5ms

        # Object detection (variable number of objects)
        num_objects = np.random.randint(0, 10)
        with TimingContext('object_detection', perf_monitor):
            detection_time = simulate_detection_processing(num_objects)

        perf_monitor.record_metric('objects_detected', num_objects, 'count')

        # Sensor fusion
        with TimingContext('sensor_fusion', perf_monitor):
            simulate_sensor_processing(3.0)  # 3ms

        # Record system metrics
        perf_monitor.record_metric('fps', 30.0, 'Hz')
        perf_monitor.record_metric('memory_mb', 150 + i * 0.1, 'MB')

        # Update health status periodically
        if i % 20 == 0:
            # Check latency
            latency_stats = perf_monitor.get_statistics('object_detection')
            if latency_stats and latency_stats['mean'] < 20:
                health_monitor.update_health(
                    'detection_latency',
                    'OK',
                    f"Detection latency: {latency_stats['mean']:.2f}ms"
                )
            elif latency_stats:
                health_monitor.update_health(
                    'detection_latency',
                    'WARNING',
                    f"High detection latency: {latency_stats['mean']:.2f}ms"
                )

            # Perform health check
            health_results = health_monitor.check_health(perf_monitor)
            logger.info(
                f"Health check at iteration {i}",
                overall_status=health_results['overall']
            )

        # Progress indicator
        if (i + 1) % 25 == 0:
            print(f"   Processed {i + 1}/{num_iterations} iterations...")

    print("   ✓ Simulation complete")

    # 3. Analyze performance metrics
    print("\n3. Performance Metrics Analysis:")
    print("=" * 70)

    stats = perf_monitor.get_all_statistics()

    # Latency metrics
    print("\n   Latency Metrics:")
    for metric in ['sensor_acquisition', 'object_detection', 'sensor_fusion']:
        if metric in stats and stats[metric]:
            s = stats[metric]
            print(f"   {metric}:")
            print(f"      Mean:   {s['mean']:.2f} ms")
            print(f"      Std:    {s['std']:.2f} ms")
            print(f"      Min:    {s['min']:.2f} ms")
            print(f"      Max:    {s['max']:.2f} ms")
            print(f"      P95:    {s['p95']:.2f} ms")
            print(f"      P99:    {s['p99']:.2f} ms")

            # Check if meeting requirements
            if s['mean'] < 20:
                print(f"      Status: ✓ Meeting <20ms requirement")
            else:
                print(f"      Status: ✗ Exceeding 20ms target")

    # Detection metrics
    if 'objects_detected' in stats:
        s = stats['objects_detected']
        print(f"\n   Object Detection:")
        print(f"      Average objects per frame: {s['mean']:.1f}")
        print(f"      Max objects detected: {int(s['max'])}")

    # 4. Health status
    print("\n4. System Health Status:")
    print("=" * 70)

    overall_health = health_monitor.get_overall_health()
    print(f"\n   Overall Status: {overall_health}")

    component_health = health_monitor.get_health()
    if component_health:
        print("\n   Component Status:")
        for component, status in component_health.items():
            print(f"   - {component}: {status['status']} - {status['message']}")

    # 5. System uptime
    uptime = perf_monitor.get_uptime()
    print(f"\n   System Uptime: {uptime:.2f} seconds")

    # 6. Optimization recommendations
    print("\n5. Optimization Analysis:")
    print("=" * 70)

    report = optimizer.get_optimization_report()

    print("\n   Latency:")
    print(f"   - Average: {report['latency']['average_ms']:.2f} ms")
    print(f"   - Meeting target: {report['latency']['meeting_target']}")

    print("\n   Memory:")
    print(f"   - Current usage: {report['memory']['current_mb']:.2f} MB")
    print(f"   - Increase from baseline: {report['memory']['increase_mb']:.2f} MB")
    print(f"   - Potential leak: {report['memory']['potential_leak']}")

    print("\n   Resources:")
    print(f"   - CPU usage: {report['resources']['cpu_percent']:.1f}%")
    print(f"   - Memory available: {report['resources']['memory_available_mb']:.0f} MB")
    print(f"   - Active threads: {report['resources']['active_threads']}/{report['resources']['max_threads']}")

    print("\n   Cache:")
    print(f"   - Size: {report['cache']['size']}/{report['cache']['max_size']}")
    print(f"   - Utilization: {report['cache']['utilization']*100:.1f}%")

    # Get optimization suggestions
    suggestions = optimizer.suggest_optimizations()
    if suggestions:
        print("\n   Optimization Suggestions:")
        for suggestion in suggestions:
            print(f"   - {suggestion}")
    else:
        print("\n   ✓ No optimization suggestions - system performing well")

    # 7. Export metrics
    print("\n6. Exporting Metrics:")
    print("=" * 70)

    # Export to JSON
    json_file = 'demo_metrics.json'
    metrics_collector.export_json(json_file)
    print(f"   ✓ Exported metrics to metrics/{json_file}")

    # Export to CSV
    csv_file = 'demo_metrics.csv'
    metrics_collector.export_csv(csv_file)
    print(f"   ✓ Exported metrics to metrics/{csv_file}")

    # 8. Component-specific optimization demos
    print("\n7. Component-Specific Optimization Examples:")
    print("=" * 70)

    # Latency optimizer
    print("\n   Latency Optimizer:")
    latency_opt = LatencyOptimizer(target_latency_ms=15.0)

    @latency_opt.measure_latency
    def sample_operation():
        time.sleep(0.012)  # 12ms operation

    # Run operation multiple times
    for _ in range(10):
        sample_operation()

    print(f"   - Average latency: {latency_opt.get_average_latency():.2f}ms")
    print(f"   - Meeting target: {latency_opt.is_meeting_target()}")

    # Memory optimizer
    print("\n   Memory Optimizer:")
    mem_opt = MemoryOptimizer()
    print(f"   - Current memory: {mem_opt.get_memory_usage():.2f} MB")
    print(f"   - Memory increase: {mem_opt.get_memory_increase():.2f} MB")

    # Optimize array
    large_array = np.random.rand(1000, 1000).astype(np.float64)
    original_size = large_array.nbytes / 1024 / 1024
    optimized = mem_opt.optimize_numpy_array(large_array)
    optimized_size = optimized.nbytes / 1024 / 1024
    print(f"   - Array optimization: {original_size:.2f}MB → {optimized_size:.2f}MB "
          f"(saved {original_size - optimized_size:.2f}MB)")

    # Resource manager
    print("\n   Resource Manager:")
    resource_mgr = ResourceManager()
    sys_stats = resource_mgr.get_system_stats()
    print(f"   - CPU usage: {sys_stats['cpu_percent']:.1f}%")
    print(f"   - Memory usage: {sys_stats['memory_percent']:.1f}%")
    print(f"   - Available threads: {resource_mgr.get_available_threads()}")

    # 9. Summary
    print("\n8. Summary:")
    print("=" * 70)

    total_operations = num_iterations * 3  # sensor, detect, fusion
    avg_latency = sum(
        stats[m]['mean'] for m in ['sensor_acquisition', 'object_detection', 'sensor_fusion']
        if m in stats and stats[m]
    ) / 3

    print(f"\n   Total operations processed: {total_operations}")
    print(f"   Average pipeline latency: {avg_latency:.2f}ms")
    print(f"   System uptime: {uptime:.2f}s")
    print(f"   Overall health: {overall_health}")

    if overall_health == 'OK' and avg_latency < 20:
        print("\n   ✓ System performing within specifications")
    else:
        print("\n   ⚠ System requires attention")

    logger.info("Performance monitoring demo completed")

    print("\n" + "=" * 70)
    print("Demonstration Complete!")
    print("=" * 70)
    print(f"\nLogs saved to: logs/perf_demo.log")
    print(f"Metrics saved to: metrics/demo_metrics.json")

    return {
        'statistics': stats,
        'health': overall_health,
        'uptime': uptime,
        'suggestions': suggestions
    }


if __name__ == '__main__':
    try:
        results = main()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
