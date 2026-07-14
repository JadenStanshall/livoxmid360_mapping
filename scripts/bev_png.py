#!/usr/bin/env python3
"""
Convert a dense point cloud to a bird's-eye-view PNG.

Intensity field is mapped to a greyscale colour scale.  No downsampling.
The output PNG is written next to the input file.

Usage:
    python3 bev_png.py <path/to/map.pcd> [--ppm 100] [--output path/to/out.png]

Dependencies:
    pip install open3d numpy pillow
"""

import argparse
import os
import sys

import numpy as np
import open3d as o3d
from PIL import Image


def load_intensity(pcd_path: str):
    """Return (xyz, intensity_1d) arrays. intensity is normalised to [0, 1]."""
    # Use the tensor API — it exposes named fields (intensity, colors, etc.)
    tpcd = o3d.t.io.read_point_cloud(pcd_path)
    if tpcd.is_empty():
        print("ERROR: file is empty or could not be read.", file=sys.stderr)
        sys.exit(1)

    xyz = tpcd.point.positions.numpy()  # (N, 3)

    if "intensity" in tpcd.point:
        raw = tpcd.point["intensity"].numpy().ravel().astype(np.float64)
        i_min, i_max = raw.min(), raw.max()
        intensity = (raw - i_min) / (i_max - i_min) if i_max > i_min else np.zeros(len(raw))
    elif "colors" in tpcd.point:
        intensity = tpcd.point["colors"].numpy()[:, 0].astype(np.float64)
        i_max = intensity.max()
        if i_max > 0:
            intensity /= i_max
    else:
        # Fallback: use normalised Z height
        z = xyz[:, 2]
        z_min, z_max = z.min(), z.max()
        intensity = (z - z_min) / (z_max - z_min) if z_max > z_min else np.zeros(len(z))
        print("WARNING: no intensity field found; colouring by Z height instead.")

    return xyz, intensity.astype(np.float32)


def project_bev(xyz: np.ndarray, intensity: np.ndarray, ppm: float):
    """
    Project points onto the XY plane and accumulate intensity per pixel.

    Returns a float32 image array shaped (H, W) with values in [0, 1].
    """
    x, y = xyz[:, 0], xyz[:, 1]

    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    width  = max(1, int(np.ceil((x_max - x_min) * ppm)) + 1)
    height = max(1, int(np.ceil((y_max - y_min) * ppm)) + 1)

    print(f"  XY extent:  {x_max - x_min:.2f} m × {y_max - y_min:.2f} m")
    print(f"  Image size: {width} × {height} px  ({ppm:.0f} px/m)")

    # Map world coords to pixel indices
    col = ((x - x_min) * ppm).astype(np.int32)
    row = ((y_max - y) * ppm).astype(np.int32)   # flip Y so +Y is up

    col = np.clip(col, 0, width  - 1)
    row = np.clip(row, 0, height - 1)

    acc   = np.zeros((height, width), dtype=np.float64)
    count = np.zeros((height, width), dtype=np.int32)

    # np.add.at handles multiple hits per pixel correctly
    np.add.at(acc,   (row, col), intensity)
    np.add.at(count, (row, col), 1)

    mask = count > 0
    img = np.zeros((height, width), dtype=np.float32)
    img[mask] = (acc[mask] / count[mask]).astype(np.float32)

    return img


def save_png(img: np.ndarray, output_path: str, invert: bool = False) -> None:
    # Scale to [0, 255]; unoccupied pixels stay black (or white when inverted)
    i_max = img.max()
    if i_max > 0:
        img = img / i_max
    pixel = (img * 255).astype(np.uint8)
    if invert:
        pixel = 255 - pixel
    Image.fromarray(pixel, mode="L").save(output_path)
    print(f"Saved  {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Dense point cloud → bird's-eye-view PNG (intensity greyscale)."
    )
    parser.add_argument("input", help="Input point cloud (.pcd or .ply)")
    parser.add_argument(
        "--ppm", type=float, default=100.0,
        help="Pixels per metre (default: 100 → 1 cm/px)",
    )
    parser.add_argument(
        "--output", default=None,
        help="Output PNG path (default: <input_stem>_bev.png next to the input file)",
    )
    parser.add_argument(
        "--invert", action="store_true",
        help="Invert the greyscale (high intensity → black, background → white)",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"ERROR: {args.input} not found.", file=sys.stderr)
        sys.exit(1)

    if args.output:
        output_path = args.output
    else:
        stem = os.path.splitext(args.input)[0]
        suffix = "_bev_inverted.png" if args.invert else "_bev.png"
        output_path = stem + suffix

    print(f"Loading  {args.input} ...")
    xyz, intensity = load_intensity(args.input)
    print(f"  Points: {len(xyz):,}")

    img = project_bev(xyz, intensity, args.ppm)
    save_png(img, output_path, invert=args.invert)


if __name__ == "__main__":
    main()
