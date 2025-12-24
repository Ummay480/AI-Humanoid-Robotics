#!/usr/bin/env python3
"""
3D Environmental Mapping Demonstration

Shows how to build and query a 3D occupancy grid from sensor data.
Demonstrates point cloud processing and obstacle mapping.
"""

import numpy as np
import time
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from src.perception.sensor_fusion.mapping import (
    OccupancyGrid3D,
    PointCloudProcessor
)


def generate_room_point_cloud():
    """
    Generate a synthetic point cloud representing a room with obstacles.

    Returns:
        numpy.ndarray: Point cloud (N x 3)
    """
    points = []

    # Floor (z=0)
    for x in np.linspace(-4, 4, 50):
        for y in np.linspace(-4, 4, 50):
            points.append([x, y, 0.0])

    # Walls
    # Front wall (y=4)
    for x in np.linspace(-4, 4, 30):
        for z in np.linspace(0, 2.5, 20):
            points.append([x, 4.0, z])

    # Back wall (y=-4)
    for x in np.linspace(-4, 4, 30):
        for z in np.linspace(0, 2.5, 20):
            points.append([x, -4.0, z])

    # Left wall (x=-4)
    for y in np.linspace(-4, 4, 30):
        for z in np.linspace(0, 2.5, 20):
            points.append([-4.0, y, z])

    # Right wall (x=4)
    for y in np.linspace(-4, 4, 30):
        for z in np.linspace(0, 2.5, 20):
            points.append([4.0, y, z])

    # Table (obstacle in center)
    for x in np.linspace(-1, 1, 10):
        for y in np.linspace(-0.5, 0.5, 5):
            for z in np.linspace(0, 0.8, 5):
                points.append([x, y, z])

    # Chair (obstacle)
    for x in np.linspace(2, 2.5, 5):
        for y in np.linspace(1, 1.5, 5):
            for z in np.linspace(0, 1, 8):
                points.append([x, y, z])

    return np.array(points)


def main():
    """Run mapping demonstration."""
    print("=" * 70)
    print("3D Environmental Mapping Demonstration")
    print("=" * 70)

    # 1. Create occupancy grid
    print("\n1. Creating 3D Occupancy Grid...")
    resolution = 0.05  # 5cm voxels
    grid_size = (200, 200, 60)  # 10m x 10m x 3m
    origin = np.array([-5.0, -5.0, 0.0])

    grid = OccupancyGrid3D(
        resolution=resolution,
        size=grid_size,
        origin=origin
    )

    print(f"   Resolution: {resolution * 100:.0f} cm")
    print(f"   Grid size: {grid_size[0]} x {grid_size[1]} x {grid_size[2]} voxels")
    print(f"   Coverage: {grid_size[0] * resolution:.1f}m x "
          f"{grid_size[1] * resolution:.1f}m x {grid_size[2] * resolution:.1f}m")
    print(f"   Origin: {origin}")

    # 2. Generate point cloud
    print("\n2. Generating synthetic room point cloud...")
    raw_points = generate_room_point_cloud()
    print(f"   Generated {len(raw_points):,} points")

    # 3. Process point cloud
    print("\n3. Processing point cloud...")
    processor = PointCloudProcessor()

    # Downsample
    print("   - Downsampling...")
    start_time = time.perf_counter()
    downsampled = processor.downsample(raw_points, voxel_size=0.1)
    downsample_time = (time.perf_counter() - start_time) * 1000
    print(f"     Reduced from {len(raw_points):,} to {len(downsampled):,} points "
          f"({downsample_time:.1f}ms)")

    # Remove outliers
    print("   - Removing outliers...")
    start_time = time.perf_counter()
    cleaned = processor.remove_outliers(downsampled, nb_neighbors=20, std_ratio=2.0)
    outlier_time = (time.perf_counter() - start_time) * 1000
    outliers_removed = len(downsampled) - len(cleaned)
    print(f"     Removed {outliers_removed} outliers ({outlier_time:.1f}ms)")

    # Extract ground plane
    print("   - Extracting ground plane...")
    start_time = time.perf_counter()
    ground_plane, ground_points, non_ground = processor.extract_ground_plane(
        cleaned,
        distance_threshold=0.05,
        num_iterations=1000
    )
    ground_time = (time.perf_counter() - start_time) * 1000

    if ground_plane is not None:
        print(f"     Ground plane: {ground_plane[:3]} (normal), "
              f"{ground_plane[3]:.2f} (offset)")
        print(f"     Ground points: {len(ground_points):,}, "
              f"Non-ground: {len(non_ground):,} ({ground_time:.1f}ms)")

    # Cluster non-ground points
    print("   - Clustering obstacles...")
    start_time = time.perf_counter()
    clusters = processor.cluster_points(non_ground, eps=0.3, min_samples=10)
    cluster_time = (time.perf_counter() - start_time) * 1000
    num_clusters = len(set(clusters)) - (1 if -1 in clusters else 0)
    print(f"     Found {num_clusters} obstacle clusters ({cluster_time:.1f}ms)")

    # 4. Update occupancy grid
    print("\n4. Updating occupancy grid...")
    sensor_origin = np.array([0.0, 0.0, 1.5])  # Camera at 1.5m height

    start_time = time.perf_counter()
    grid.update_with_point_cloud(cleaned, sensor_origin)
    update_time = (time.perf_counter() - start_time) * 1000
    print(f"   Updated grid with {len(cleaned):,} points ({update_time:.1f}ms)")

    # 5. Mark detected objects
    print("\n5. Marking detected objects...")

    # Table
    table_position = np.array([0.0, 0.0, 0.4])
    table_size = np.array([2.0, 1.0, 0.8])
    grid.mark_object(table_position, table_size, occupancy_value=0.9)
    print(f"   - Table at {table_position} (size: {table_size})")

    # Chair
    chair_position = np.array([2.25, 1.25, 0.5])
    chair_size = np.array([0.5, 0.5, 1.0])
    grid.mark_object(chair_position, chair_size, occupancy_value=0.9)
    print(f"   - Chair at {chair_position} (size: {chair_size})")

    # 6. Query occupancy
    print("\n6. Querying occupancy at test points...")

    test_points = [
        (np.array([0.0, 0.0, 0.5]), "Center of table"),
        (np.array([2.25, 1.25, 0.5]), "Center of chair"),
        (np.array([3.0, 3.0, 1.0]), "Empty space"),
        (np.array([0.0, 0.0, 0.0]), "Floor"),
        (np.array([4.0, 0.0, 1.0]), "Wall"),
    ]

    for point, description in test_points:
        occupancy = grid.get_occupancy(point)
        status = "OCCUPIED" if occupancy > 0.5 else "FREE"
        print(f"   - {description:20s} {point}: "
              f"{occupancy:.2f} ({status})")

    # 7. Get grid statistics
    print("\n7. Grid Statistics:")
    stats = grid.get_statistics()
    print(f"   - Total voxels: {stats['total_voxels']:,}")
    print(f"   - Occupied voxels: {stats['occupied_voxels']:,} "
          f"({stats['occupied_percentage']:.1f}%)")
    print(f"   - Free voxels: {stats['free_voxels']:,} "
          f"({stats['free_percentage']:.1f}%)")
    print(f"   - Unknown voxels: {stats['unknown_voxels']:,} "
          f"({stats['unknown_percentage']:.1f}%)")

    # 8. Export map
    print("\n8. Exporting map...")
    output_file = 'environmental_map_demo.npy'
    grid.export_to_file(output_file)
    print(f"   Saved occupancy grid to {output_file}")

    # 9. Performance summary
    print("\n9. Performance Summary:")
    total_processing_time = (
        downsample_time + outlier_time + ground_time +
        cluster_time + update_time
    )
    print(f"   - Total processing time: {total_processing_time:.1f}ms")
    print(f"     * Downsampling:  {downsample_time:.1f}ms")
    print(f"     * Outlier removal: {outlier_time:.1f}ms")
    print(f"     * Ground extraction: {ground_time:.1f}ms")
    print(f"     * Clustering: {cluster_time:.1f}ms")
    print(f"     * Grid update: {update_time:.1f}ms")

    if total_processing_time < 20:
        print(f"\n   ✓ Meeting real-time requirement (<20ms)")
    else:
        print(f"\n   ✗ Exceeds real-time requirement (target: <20ms)")

    print("\n" + "=" * 70)
    print("Demonstration complete!")
    print("=" * 70)

    return {
        'grid': grid,
        'processed_points': cleaned,
        'clusters': clusters,
        'statistics': stats,
        'processing_time_ms': total_processing_time
    }


if __name__ == '__main__':
    try:
        results = main()

        # Optional: Keep script running to inspect results
        print("\nPress Ctrl+C to exit...")
        try:
            import time
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nExiting...")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
