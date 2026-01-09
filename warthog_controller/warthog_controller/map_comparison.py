#!/usr/bin/env python3
"""
3D Map Quality Comparison Tool
Computes PSNR, SSIM, and 3D point cloud metrics for map quality assessment.

Usage:
  source venv/bin/activate
  
  python3 map_comparison.py \\
    --ground-truth map_ground_truth.pcd \\
    --test-maps map_misaligned.pcd map_calibrated.pcd map_rgbd.pcd \\
    --labels "Misaligned" "Calibrated" "RGBD" \\
    --output comparison_results.csv
"""

import argparse
import sys
import csv
from pathlib import Path
import numpy as np

try:
    import open3d as o3d
    import cv2
    from skimage.metrics import structural_similarity as ssim
    from skimage.metrics import peak_signal_noise_ratio as psnr
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    print(f"ERROR: Missing dependency: {e}")
    print("Activate virtual environment: source venv/bin/activate")
    sys.exit(1)


def point_cloud_to_2d_grid(pcd, resolution=0.05, height_slice=0.5, height_tolerance=0.3):
    """
    Convert 3D point cloud to 2D occupancy grid at specified height.
    
    Args:
        pcd: Open3D point cloud
        resolution: Grid resolution in meters
        height_slice: Height (z) to extract 2D slice
        height_tolerance: Tolerance around height_slice
    
    Returns:
        2D numpy array (occupancy grid)
    """
    points = np.asarray(pcd.points)
    
    if len(points) == 0:
        return None
    
    # Filter points near the height slice
    z_mask = np.abs(points[:, 2] - height_slice) < height_tolerance
    points_2d = points[z_mask][:, :2]  # Only x, y
    
    if len(points_2d) == 0:
        print(f"  Warning: No points found at height {height_slice}±{height_tolerance}")
        return None
    
    # Compute grid bounds
    min_bound = points_2d.min(axis=0)
    max_bound = points_2d.max(axis=0)
    
    # Create grid
    grid_size = ((max_bound - min_bound) / resolution).astype(int) + 1
    grid = np.full(grid_size[::-1], 128, dtype=np.uint8)  # Unknown = 128
    
    # Fill occupied cells
    indices = ((points_2d - min_bound) / resolution).astype(int)
    grid[indices[:, 1], indices[:, 0]] = 0  # Occupied = 0
    
    # Mark free space (simple approach: nearby cells are free)
    # In a real implementation, you'd use ray tracing
    kernel = np.ones((3, 3), np.uint8)
    occupied_mask = (grid == 0)
    free_mask = cv2.dilate(occupied_mask.astype(np.uint8), kernel, iterations=2)
    grid[free_mask.astype(bool) & (grid == 128)] = 255  # Free = 255
    
    return grid


def compute_chamfer_distance(pcd1, pcd2):
    """
    Compute Chamfer distance between two point clouds.
    Lower is better (measures average nearest neighbor distance).
    """
    # Downsample for efficiency
    pcd1_down = pcd1.voxel_down_sample(0.05)
    pcd2_down = pcd2.voxel_down_sample(0.05)
    
    # Compute distances
    dists1 = pcd1_down.compute_point_cloud_distance(pcd2_down)
    dists2 = pcd2_down.compute_point_cloud_distance(pcd1_down)
    
    chamfer = (np.mean(dists1) + np.mean(dists2)) / 2
    return chamfer


def compute_hausdorff_distance(pcd1, pcd2):
    """
    Compute Hausdorff distance (maximum nearest neighbor distance).
    Lower is better (measures worst-case error).
    """
    pcd1_down = pcd1.voxel_down_sample(0.05)
    pcd2_down = pcd2.voxel_down_sample(0.05)
    
    dists1 = pcd1_down.compute_point_cloud_distance(pcd2_down)
    dists2 = pcd2_down.compute_point_cloud_distance(pcd1_down)
    
    hausdorff = max(np.max(dists1), np.max(dists2))
    return hausdorff


def compute_overlap_ratio(pcd1, pcd2, threshold=0.05):
    """
    Compute overlap ratio: percentage of points within threshold distance.
    Higher is better (1.0 = perfect overlap).
    """
    pcd1_down = pcd1.voxel_down_sample(0.05)
    pcd2_down = pcd2.voxel_down_sample(0.05)
    
    dists1 = pcd1_down.compute_point_cloud_distance(pcd2_down)
    overlap = np.sum(np.array(dists1) < threshold) / len(dists1)
    return overlap


def align_grids(grid1, grid2):
    """
    Align two grids to same size by padding with unknown values (128).
    """
    h1, w1 = grid1.shape
    h2, w2 = grid2.shape
    
    h_max = max(h1, h2)
    w_max = max(w1, w2)
    
    aligned1 = np.full((h_max, w_max), 128, dtype=np.uint8)
    aligned2 = np.full((h_max, w_max), 128, dtype=np.uint8)
    
    aligned1[:h1, :w1] = grid1
    aligned2[:h2, :w2] = grid2
    
    return aligned1, aligned2


def compare_maps(gt_path, test_paths, labels):
    """
    Compare multiple test maps against ground truth.
    
    Args:
        gt_path: Path to ground truth point cloud
        test_paths: List of paths to test point clouds
        labels: List of labels for test maps
    
    Returns:
        DataFrame with comparison metrics
    """
    print("="*70)
    print("Map Quality Comparison")
    print("="*70)
    
    # Load ground truth
    print(f"\nLoading ground truth: {gt_path}")
    pcd_gt = o3d.io.read_point_cloud(str(gt_path))
    print(f"  Points: {len(pcd_gt.points)}")
    
    if len(pcd_gt.points) == 0:
        print("ERROR: Ground truth point cloud is empty!")
        return None
    
    # Convert ground truth to 2D grid
    print("\nConverting ground truth to 2D grid...")
    grid_gt = point_cloud_to_2d_grid(pcd_gt)
    
    if grid_gt is None:
        print("ERROR: Failed to create ground truth 2D grid!")
        return None
    
    print(f"  Grid size: {grid_gt.shape}")
    
    # Compare each test map
    results = []
    
    for test_path, label in zip(test_paths, labels):
        print(f"\n{'-'*70}")
        print(f"Comparing: {label}")
        print(f"  File: {test_path}")
        
        # Load test map
        pcd_test = o3d.io.read_point_cloud(str(test_path))
        print(f"  Points: {len(pcd_test.points)}")
        
        if len(pcd_test.points) == 0:
            print("  WARNING: Point cloud is empty, skipping...")
            continue
        
        # 3D metrics
        print("\n  Computing 3D metrics...")
        chamfer = compute_chamfer_distance(pcd_gt, pcd_test)
        hausdorff = compute_hausdorff_distance(pcd_gt, pcd_test)
        overlap = compute_overlap_ratio(pcd_gt, pcd_test)
        
        print(f"    Chamfer distance: {chamfer:.6f} m")
        print(f"    Hausdorff distance: {hausdorff:.6f} m")
        print(f"    Overlap ratio: {overlap:.4f} ({overlap*100:.2f}%)")
        
        # 2D metrics
        print("\n  Computing 2D metrics...")
        grid_test = point_cloud_to_2d_grid(pcd_test)
        
        if grid_test is not None:
            # Align grids
            grid_gt_aligned, grid_test_aligned = align_grids(grid_gt, grid_test)
            
            # Only compare known cells (not 128=unknown)
            mask = (grid_gt_aligned != 128) & (grid_test_aligned != 128)
            
            if np.sum(mask) > 100:  # Need enough known cells
                gt_masked = grid_gt_aligned[mask]
                test_masked = grid_test_aligned[mask]
                
                # PSNR
                psnr_value = psnr(gt_masked, test_masked, data_range=255)
                print(f"    PSNR: {psnr_value:.2f} dB")
                
                # SSIM (need 2D for structural similarity)
                # Convert masked regions back to 2D for SSIM
                ssim_value = ssim(
                    grid_gt_aligned.astype(float),
                    grid_test_aligned.astype(float),
                    data_range=255
                )
                print(f"    SSIM: {ssim_value:.4f}")
            else:
                print("    WARNING: Not enough overlap for 2D metrics")
                psnr_value = np.nan
                ssim_value = np.nan
        else:
            psnr_value = np.nan
            ssim_value = np.nan
        
        # Store results
        results.append({
            'Map': label,
            'Chamfer_Distance_m': chamfer,
            'Hausdorff_Distance_m': hausdorff,
            'Overlap_Ratio': overlap,
            'PSNR_dB': psnr_value,
            'SSIM': ssim_value,
            'Points': len(pcd_test.points)
        })
    
    # Create DataFrame
    df = pd.DataFrame(results)
    
    print("\n" + "="*70)
    print("Comparison Summary:")
    print("="*70)
    print(df.to_string(index=False))
    print()
    
    return df


def plot_comparison(df, output_dir):
    """Generate comparison plots"""
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True, parents=True)
    
    # Metrics to plot
    metrics = [
        ('Chamfer_Distance_m', 'Chamfer Distance (m)', 'lower is better'),
        ('Hausdorff_Distance_m', 'Hausdorff Distance (m)', 'lower is better'),
        ('Overlap_Ratio', 'Overlap Ratio', 'higher is better'),
        ('PSNR_dB', 'PSNR (dB)', 'higher is better'),
        ('SSIM', 'SSIM', 'higher is better')
    ]
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()
    
    for idx, (metric, title, direction) in enumerate(metrics):
        ax = axes[idx]
        
        data = df[['Map', metric]].dropna()
        if len(data) == 0:
            continue
        
        bars = ax.bar(data['Map'], data[metric])
        ax.set_title(title)
        ax.set_ylabel(metric)
        ax.tick_params(axis='x', rotation=45)
        ax.grid(axis='y', alpha=0.3)
        
        # Color bars: green=better, red=worse
        if 'lower' in direction:
            best_val = data[metric].min()
            colors = ['green' if v == best_val else 'orange' if v < best_val*1.5 else 'red' 
                     for v in data[metric]]
        else:
            best_val = data[metric].max()
            colors = ['green' if v == best_val else 'orange' if v > best_val*0.7 else 'red'
                     for v in data[metric]]
        
        for bar, color in zip(bars, colors):
            bar.set_color(color)
        
        # Add note
        ax.text(0.5, 0.95, f'({direction})', 
               transform=ax.transAxes, ha='center', va='top',
               fontsize=8, style='italic', alpha=0.6)
    
    # Remove extra subplot
    fig.delaxes(axes[5])
    
    plt.tight_layout()
    plot_path = output_dir / 'comparison_metrics.png'
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    print(f"✓ Plot saved to: {plot_path}")
    plt.close()


def main():
    """CLI entry point"""
    parser = argparse.ArgumentParser(
        description='Compare 3D maps using multiple quality metrics',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        '--ground-truth',
        type=str,
        required=True,
        help='Path to ground truth map (PCD/PLY format)'
    )
    parser.add_argument(
        '--test-maps',
        type=str,
        nargs='+',
        required=True,
        help='Paths to test maps'
    )
    parser.add_argument(
        '--labels',
        type=str,
        nargs='+',
        required=True,
        help='Labels for test maps'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='comparison_results.csv',
        help='Output CSV file path'
    )
    parser.add_argument(
        '--plot-dir',
        type=str,
        default='comparison_plots',
        help='Directory for output plots'
    )
    
    args = parser.parse_args()
    
    # Validate
    if len(args.test_maps) != len(args.labels):
        print("ERROR: Number of test maps must match number of labels!")
        return 1
    
    gt_path = Path(args.ground_truth)
    if not gt_path.exists():
        print(f"ERROR: Ground truth not found: {gt_path}")
        return 1
    
    for path in args.test_maps:
        if not Path(path).exists():
            print(f"ERROR: Test map not found: {path}")
            return 1
    
    # Run comparison
    try:
        df = compare_maps(gt_path, args.test_maps, args.labels)
        
        if df is None or len(df) == 0:
            print("ERROR: Comparison failed!")
            return 1
        
        # Save results
        output_path = Path(args.output)
        df.to_csv(output_path, index=False)
        print(f"\n✓ Results saved to: {output_path}")
        
        # Generate plots
        plot_comparison(df, args.plot_dir)
        
        return 0
        
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())

