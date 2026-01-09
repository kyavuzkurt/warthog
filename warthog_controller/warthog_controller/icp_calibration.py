#!/usr/bin/env python3
"""
ICP-Based LiDAR Calibration
Computes calibration transform by aligning misaligned point cloud to ground truth.

Usage:
  # Activate virtual environment first!
  source venv/bin/activate
  
  python3 icp_calibration.py \
    --ground-truth map_ground_truth.pcd \
    --misaligned map_misaligned.pcd \
    --output calibration_result.yaml
"""

import argparse
import sys
import yaml
import numpy as np
from pathlib import Path

try:
    import open3d as o3d
except ImportError:
    print("ERROR: Open3D not found. Activate virtual environment:")
    print("  source venv/bin/activate")
    sys.exit(1)


def rotation_matrix_to_euler(R):
    """
    Convert rotation matrix to Euler angles (roll, pitch, yaw) in radians.
    Uses ZYX convention (yaw-pitch-roll).
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    
    return roll, pitch, yaw


def compute_fpfh_features(pcd, radius):
    """Compute FPFH features for point cloud"""
    print(f"  Computing FPFH features with radius {radius}...")
    
    # Estimate normals if not present
    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=30)
        )
    
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius*5, max_nn=100)
    )
    return fpfh


def preprocess_point_cloud(pcd, voxel_size):
    """Downsample and compute normals"""
    print(f"  Downsampling with voxel size {voxel_size}...")
    pcd_down = pcd.voxel_down_sample(voxel_size)
    
    print(f"  Points after downsampling: {len(pcd_down.points)}")
    
    print("  Estimating normals...")
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 2, max_nn=30
        )
    )
    
    print("  Removing statistical outliers...")
    pcd_clean, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"  Points after outlier removal: {len(pcd_clean.points)}")
    
    return pcd_clean


def ransac_registration(source, target, voxel_size):
    """Coarse alignment using RANSAC"""
    print("\n[2/4] RANSAC-based global registration...")
    
    # Compute features
    source_fpfh = compute_fpfh_features(source, voxel_size)
    target_fpfh = compute_fpfh_features(target, voxel_size)
    
    distance_threshold = voxel_size * 1.5
    print(f"  RANSAC distance threshold: {distance_threshold}")
    
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target,
        source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
    )
    
    print(f"  RANSAC fitness: {result.fitness:.4f}")
    print(f"  RANSAC RMSE: {result.inlier_rmse:.4f} m")
    
    return result


def icp_refinement(source, target, initial_transform, voxel_size):
    """Fine alignment using point-to-plane ICP"""
    print("\n[3/4] ICP refinement (point-to-plane)...")
    
    threshold = voxel_size * 0.4
    print(f"  ICP distance threshold: {threshold}")
    
    # Multi-scale ICP
    current_transform = initial_transform
    scales = [1.0, 0.5, 0.25]
    
    for i, scale in enumerate(scales):
        print(f"\n  Scale {i+1}/{len(scales)}: {scale}")
        current_threshold = threshold * scale
        
        result = o3d.pipelines.registration.registration_icp(
            source, target,
            current_threshold,
            current_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=100
            )
        )
        
        current_transform = result.transformation
        print(f"    Fitness: {result.fitness:.4f}")
        print(f"    RMSE: {result.inlier_rmse:.6f} m")
    
    return result


def calibrate_lidar(ground_truth_path, misaligned_path, output_path, visualize=False):
    """
    Main calibration function.
    
    Args:
        ground_truth_path: Path to ground truth point cloud (.pcd, .ply, .xyz)
        misaligned_path: Path to misaligned point cloud
        output_path: Path to save calibration results (.yaml)
        visualize: Show visualization of alignment
    """
    print("="*70)
    print("LiDAR Calibration using ICP")
    print("="*70)
    
    # Load point clouds
    print("\n[1/4] Loading point clouds...")
    print(f"  Ground truth: {ground_truth_path}")
    pcd_gt = o3d.io.read_point_cloud(str(ground_truth_path))
    print(f"    Points: {len(pcd_gt.points)}")
    
    print(f"  Misaligned: {misaligned_path}")
    pcd_mis = o3d.io.read_point_cloud(str(misaligned_path))
    print(f"    Points: {len(pcd_mis.points)}")
    
    if len(pcd_gt.points) == 0 or len(pcd_mis.points) == 0:
        print("\nERROR: One or both point clouds are empty!")
        return None
    
    # Preprocess
    voxel_size = 0.05  # 5cm voxels for processing
    pcd_gt_down = preprocess_point_cloud(pcd_gt, voxel_size)
    pcd_mis_down = preprocess_point_cloud(pcd_mis, voxel_size)
    
    # RANSAC global registration
    ransac_result = ransac_registration(pcd_mis_down, pcd_gt_down, voxel_size)
    
    # ICP refinement
    icp_result = icp_refinement(pcd_mis_down, pcd_gt_down, ransac_result.transformation, voxel_size)
    
    # Extract calibration parameters
    print("\n[4/4] Extracting calibration parameters...")
    T = icp_result.transformation
    
    # Extract translation
    translation = T[0:3, 3]
    
    # Extract rotation
    rotation_matrix = T[0:3, 0:3]
    roll, pitch, yaw = rotation_matrix_to_euler(rotation_matrix)
    
    print(f"\n  Translation correction:")
    print(f"    dx = {translation[0]:.6f} m")
    print(f"    dy = {translation[1]:.6f} m")
    print(f"    dz = {translation[2]:.6f} m")
    
    print(f"\n  Rotation correction:")
    print(f"    roll  = {roll:.6f} rad ({np.degrees(roll):.4f}°)")
    print(f"    pitch = {pitch:.6f} rad ({np.degrees(pitch):.4f}°)")
    print(f"    yaw   = {yaw:.6f} rad ({np.degrees(yaw):.4f}°)")
    
    print(f"\n  Alignment quality:")
    print(f"    Fitness: {icp_result.fitness:.4f} ({icp_result.fitness*100:.2f}%)")
    print(f"    RMSE: {icp_result.inlier_rmse:.6f} m")
    
    # Save results
    calibration_data = {
        'calibration': {
            'translation': {
                'x': float(translation[0]),
                'y': float(translation[1]),
                'z': float(translation[2])
            },
            'rotation': {
                'roll': float(roll),
                'pitch': float(pitch),
                'yaw': float(yaw)
            },
            'rotation_degrees': {
                'roll': float(np.degrees(roll)),
                'pitch': float(np.degrees(pitch)),
                'yaw': float(np.degrees(yaw))
            }
        },
        'validation': {
            'fitness': float(icp_result.fitness),
            'inlier_rmse': float(icp_result.inlier_rmse),
            'correspondence_set_size': len(icp_result.correspondence_set)
        },
        'transformation_matrix': T.tolist()
    }
    
    output_path = Path(output_path)
    with open(output_path, 'w') as f:
        yaml.dump(calibration_data, f, default_flow_style=False, sort_keys=False)
    
    print(f"\n✓ Calibration saved to: {output_path}")
    
    # Save transformation matrix separately
    transform_path = output_path.with_suffix('.npy')
    np.save(transform_path, T)
    print(f"✓ Transformation matrix saved to: {transform_path}")
    
    # Visualize if requested
    if visualize:
        print("\nGenerating visualization...")
        
        # Apply transformation to misaligned cloud
        pcd_mis_aligned = pcd_mis.transform(T)
        
        # Color the clouds
        pcd_gt.paint_uniform_color([0.0, 0.651, 0.929])  # Blue (ground truth)
        pcd_mis.paint_uniform_color([1.0, 0.0, 0.0])     # Red (before calibration)
        pcd_mis_aligned.paint_uniform_color([0.0, 1.0, 0.0])  # Green (after calibration)
        
        # Show before/after
        print("\nVisualization:")
        print("  Blue: Ground truth")
        print("  Red: Misaligned (before calibration)")
        print("  Green: Aligned (after calibration)")
        print("\nClose the window to continue...")
        
        o3d.visualization.draw_geometries(
            [pcd_gt.voxel_down_sample(0.1), pcd_mis_aligned.voxel_down_sample(0.1)],
            window_name="Calibration Result: Ground Truth (Blue) vs Calibrated (Green)"
        )
    
    print("\n" + "="*70)
    print("Calibration complete!")
    print("="*70)
    
    return calibration_data


def main():
    """CLI entry point"""
    parser = argparse.ArgumentParser(
        description='LiDAR calibration using ICP registration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 icp_calibration.py \\
    --ground-truth map_ground_truth.pcd \\
    --misaligned map_misaligned.pcd \\
    --output calibration_result.yaml \\
    --visualize
        """
    )
    
    parser.add_argument(
        '--ground-truth',
        type=str,
        required=True,
        help='Path to ground truth point cloud (PCD, PLY, XYZ format)'
    )
    parser.add_argument(
        '--misaligned',
        type=str,
        required=True,
        help='Path to misaligned point cloud'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='calibration_result.yaml',
        help='Output calibration file path (YAML format)'
    )
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='Show visualization of alignment'
    )
    
    args = parser.parse_args()
    
    # Validate input files
    gt_path = Path(args.ground_truth)
    mis_path = Path(args.misaligned)
    
    if not gt_path.exists():
        print(f"ERROR: Ground truth file not found: {gt_path}")
        return 1
    
    if not mis_path.exists():
        print(f"ERROR: Misaligned file not found: {mis_path}")
        return 1
    
    # Run calibration
    try:
        result = calibrate_lidar(
            gt_path,
            mis_path,
            args.output,
            visualize=args.visualize
        )
        
        if result is None:
            return 1
        
        return 0
        
    except Exception as e:
        print(f"\nERROR during calibration: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())

