#!/usr/bin/env python3
"""
Convert a PLY point cloud to PCD format.
Output is written to the same directory as the input file.

Usage:
    python3 scripts/convert_ply_to_pcd.py --input maps/map_1_superodom_lidar/globalMap.ply
"""

import argparse
import os
import sys

import open3d as o3d


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", required=True, help="Path to the .ply file to convert")
    args = ap.parse_args()

    input_path = os.path.abspath(args.input)
    if not os.path.exists(input_path):
        print(f"ERROR: file not found: {input_path}")
        sys.exit(1)

    if not input_path.lower().endswith(".ply"):
        print(f"WARNING: input does not have a .ply extension: {input_path}")

    output_path = os.path.splitext(input_path)[0] + ".pcd"

    print(f"Reading : {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    print(f"Points  : {len(pcd.points):,}")
    print(f"Writing : {output_path}")
    o3d.io.write_point_cloud(output_path, pcd)
    print("Done.")


if __name__ == "__main__":
    main()
