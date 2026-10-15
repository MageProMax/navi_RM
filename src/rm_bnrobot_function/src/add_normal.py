#!/usr/bin/env python3
"""Estimate PCD normals without workstation-specific paths."""
import argparse
import math
from pathlib import Path

import open3d as o3d


def main():
    parser = argparse.ArgumentParser(description="为 PCD 点云估计法向量")
    parser.add_argument("input", type=Path, help="输入 PCD 文件")
    parser.add_argument("output", type=Path, help="输出 PCD 文件")
    parser.add_argument("--radius", type=float, default=0.5)
    parser.add_argument("--max-nn", type=int, default=30)
    args = parser.parse_args()
    if not math.isfinite(args.radius) or args.radius <= 0 or args.max_nn < 3:
        parser.error("radius 必须为有限正数，max-nn 至少为 3")
    if args.input.resolve() == args.output.resolve():
        parser.error("输入和输出须使用不同文件，保留原始地图")
    cloud = o3d.io.read_point_cloud(str(args.input))
    if cloud.is_empty():
        parser.error(f"地图为空或无法读取: {args.input}")
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=args.radius, max_nn=args.max_nn))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(args.output), cloud):
        raise OSError(f"无法写入地图: {args.output}")
    print(f"已保存 {len(cloud.points)} 个点及法向量: {args.output}")


if __name__ == "__main__":
    main()
