#!/usr/bin/env python3
"""
Postprocess globalMap.pcd → final_map.pcd for use as a SuperLoc prior map.

Steps:
  1. Voxel downsample  (default 5 cm)
  2. Statistical outlier removal

Usage:
    python3 postprocess_map.py --input maps/globalMap.pcd --output maps/final_map.pcd

Dependencies:
    pip install open3d
"""

import argparse
import os
import sys

import open3d as o3d


def postprocess(
    input_path: str,
    output_path: str,
    voxel_size: float,
    nb_neighbors: int,
    std_ratio: float,
) -> None:
    print(f"Loading  {input_path} ...")
    pcd = o3d.io.read_point_cloud(input_path)
    if not pcd.has_points():
        print("ERROR: input file is empty or could not be read.", file=sys.stderr)
        sys.exit(1)
    print(f"  Input:                    {len(pcd.points):>10,} points")

    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"  After voxel downsample:   {len(pcd.points):>10,} points  (voxel_size={voxel_size} m)")

    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    print(f"  After outlier removal:    {len(pcd.points):>10,} points  (nb={nb_neighbors}, std={std_ratio})")

    o3d.io.write_point_cloud(output_path, pcd)
    print(f"Saved    {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Downsample and clean a PCD map for SuperLoc localization."
    )
    parser.add_argument("--input",  required=True, help="Input PCD (e.g. maps/globalMap.pcd)")
    parser.add_argument("--output", required=True, help="Output PCD (e.g. maps/final_map.pcd)")
    parser.add_argument(
        "--voxel-size", type=float, default=0.05,
        help="Voxel size in metres (default: 0.05)",
    )
    parser.add_argument(
        "--nb-neighbors", type=int, default=20,
        help="Statistical outlier removal: neighbour count (default: 20)",
    )
    parser.add_argument(
        "--std-ratio", type=float, default=2.0,
        help="Statistical outlier removal: std deviation multiplier (default: 2.0)",
    )
    args = parser.parse_args()

    output = args.output
    if not output.lower().endswith(".pcd"):
        output = os.path.splitext(output)[0] + ".pcd"
        print(f"Output path changed to {output} (SuperLoc requires PCD format)")

    postprocess(args.input, output, args.voxel_size, args.nb_neighbors, args.std_ratio)


if __name__ == "__main__":
    main()
