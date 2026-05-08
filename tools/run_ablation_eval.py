#!/usr/bin/env python3
"""Run Zero-DCE / Depth-to-map ablation on Normal.bag and darkroom1.bag."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
import textwrap
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]

VARIANTS = {
    "baseline": [
        "use_zero_dce:=false",
        "use_depth_to_map:=0",
        "use_depth_to_map_pose_graph:=0",
        "depth_map_uncertainty_enable:=0",
        "use_structural_planes:=0",
        "loop_geom_verify:=0",
    ],
    "zero_dce_only": [
        "use_zero_dce:=true",
        "zero_dce_use_onnx:=true",
        "use_depth_to_map:=0",
        "use_depth_to_map_pose_graph:=0",
        "depth_map_uncertainty_enable:=0",
        "use_structural_planes:=0",
        "loop_geom_verify:=0",
    ],
    "depth_to_map_only": [
        "use_zero_dce:=false",
        "use_depth_to_map:=1",
        "use_depth_to_map_pose_graph:=1",
        "depth_map_uncertainty_enable:=0",
        "use_structural_planes:=0",
        "loop_geom_verify:=0",
    ],
    "full": [
        "use_zero_dce:=true",
        "zero_dce_use_onnx:=true",
        "use_depth_to_map:=1",
        "use_depth_to_map_pose_graph:=1",
        "depth_map_uncertainty_enable:=0",
        "use_structural_planes:=0",
        "loop_geom_verify:=0",
    ],
    "uncertainty_only": [
        "use_zero_dce:=true",
        "zero_dce_use_onnx:=true",
        "use_depth_to_map:=1",
        "use_depth_to_map_pose_graph:=1",
        "depth_map_uncertainty_enable:=1",
        "use_structural_planes:=0",
        "loop_geom_verify:=0",
    ],
    "planes_only": [
        "use_zero_dce:=true",
        "zero_dce_use_onnx:=true",
        "use_depth_to_map:=1",
        "use_depth_to_map_pose_graph:=1",
        "depth_map_uncertainty_enable:=0",
        "use_structural_planes:=1",
        "loop_geom_verify:=0",
    ],
    "loop_geom_only": [
        "use_zero_dce:=true",
        "zero_dce_use_onnx:=true",
        "use_depth_to_map:=1",
        "use_depth_to_map_pose_graph:=1",
        "depth_map_uncertainty_enable:=0",
        "use_structural_planes:=0",
        "loop_geom_verify:=1",
    ],
    "planes_loop": [
        "use_zero_dce:=true",
        "zero_dce_use_onnx:=true",
        "use_depth_to_map:=1",
        "use_depth_to_map_pose_graph:=1",
        "depth_map_uncertainty_enable:=0",
        "use_structural_planes:=1",
        "loop_geom_verify:=1",
    ],
    "full_optimized": [
        "use_zero_dce:=true",
        "zero_dce_use_onnx:=true",
        "use_depth_to_map:=1",
        "use_depth_to_map_pose_graph:=1",
        "depth_map_uncertainty_enable:=1",
        "use_structural_planes:=1",
        "loop_geom_verify:=1",
    ],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--normal-bag", default="/home/mhenwa/slam/bags/Normal.bag")
    parser.add_argument("--darkroom-bag", default="/home/mhenwa/slam/bags/darkroom1.bag")
    parser.add_argument("--gt-root", default="/home/mhenwa/slam/Ground-Challenge/psudo_gt")
    parser.add_argument("--docker-image", default="vins-rgbd:melodic")
    parser.add_argument("--output-root", default=str(REPO_ROOT / "output" / "ablation"))
    parser.add_argument("--workspace", default=str(REPO_ROOT / ".docker_catkin_ws"))
    parser.add_argument("--timestamp", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--startup-wait", type=float, default=8.0)
    parser.add_argument("--post-bag-wait", type=float, default=3.0)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument(
        "--variant",
        action="append",
        choices=sorted(VARIANTS.keys()),
        help="Run only selected variant(s). Defaults to all variants.",
    )
    parser.add_argument(
        "--sequence",
        action="append",
        choices=["normal", "darkroom1"],
        help="Run only selected sequence(s). Defaults to both.",
    )
    return parser.parse_args()


def run(cmd: List[str], cwd: Path = REPO_ROOT, check: bool = True) -> subprocess.CompletedProcess:
    print("+ " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, cwd=str(cwd), check=check)


def docker_base_args(args: argparse.Namespace, bag_path: Optional[Path] = None) -> List[str]:
    docker_args = [
        "docker",
        "run",
        "--rm",
        "--network",
        "host",
        "-v",
        f"{REPO_ROOT}:/workspace/VINS-RGBD",
        "-v",
        f"{Path(args.workspace).expanduser().resolve()}:/workspace/VINS-RGBD/.docker_catkin_ws",
        "-w",
        "/workspace/VINS-RGBD",
    ]
    if bag_path is not None:
        docker_args += ["-v", f"{bag_path.parent}:/data:ro"]
    return docker_args + [args.docker_image]


def build_in_docker(args: argparse.Namespace) -> None:
    command = (
        "source /opt/ros/melodic/setup.bash; "
        "cd .docker_catkin_ws; "
        "catkin build feature_tracker vins_estimator pose_graph -DCMAKE_BUILD_TYPE=Release"
    )
    run(docker_base_args(args) + ["bash", "-lc", command])


def extract_normal_gt(args: argparse.Namespace, normal_bag: Path, out_dir: Path) -> Path:
    gt_path = out_dir / "gt" / "Normal_vrpn_gt.txt"
    gt_path.parent.mkdir(parents=True, exist_ok=True)
    command = (
        "source /opt/ros/melodic/setup.bash; "
        "python tools/extract_pose_stamped_gt.py "
        f"--bag /data/{normal_bag.name} "
        "--topic /vrpn_client_node/jackal/pose "
        f"--out /workspace/VINS-RGBD/{gt_path.relative_to(REPO_ROOT)}"
    )
    run(docker_base_args(args, normal_bag) + ["bash", "-lc", command])
    return gt_path


def launch_args_for_sequence(seq_name: str) -> List[str]:
    if seq_name == "darkroom1":
        return [
            "config_path:=/workspace/VINS-RGBD/config/ground_challenge/groundchallenge_config.yaml",
            "depth_config_path:=/workspace/VINS-RGBD/config/ground_challenge/groundchallenge_depth_config.yaml",
        ]
    return []


def run_variant(
    args: argparse.Namespace,
    seq_name: str,
    bag_path: Path,
    variant: str,
    run_dir: Path,
) -> bool:
    run_dir.mkdir(parents=True, exist_ok=True)
    variant_args = " ".join(launch_args_for_sequence(seq_name) + VARIANTS[variant])
    rel_run_dir = run_dir.relative_to(REPO_ROOT)
    command = textwrap.dedent(
        f"""
        set -o pipefail
        source /opt/ros/melodic/setup.bash
        source .docker_catkin_ws/devel/setup.bash
        rm -rf output/vins output/pose_graph output/pcd output/voxblox
        rm -f output/perf_monitor.json output/launch.log output/rosbag.log
        mkdir -p output/vins output/pose_graph output/pcd output/voxblox

        roslaunch vins_estimator realsense_color.launch {variant_args} > output/launch.log 2>&1 &
        launch_pid=$!
        sleep {args.startup_wait}

        python tools/perf_monitor_node.py _output_path:=/workspace/VINS-RGBD/output/perf_monitor.json > output/perf_monitor.log 2>&1 &
        perf_pid=$!
        sleep 1

        rosbag play --quiet /data/{bag_path.name} > output/rosbag.log 2>&1
        bag_status=$?
        sleep {args.post_bag_wait}

        kill -INT $launch_pid >/dev/null 2>&1 || true
        wait $launch_pid >/dev/null 2>&1 || true
        kill -INT $perf_pid >/dev/null 2>&1 || true
        wait $perf_pid >/dev/null 2>&1 || true

        mkdir -p {rel_run_dir}
        cp -a output/vins {rel_run_dir}/ || true
        cp -a output/pose_graph {rel_run_dir}/ || true
        cp -a output/pcd {rel_run_dir}/ || true
        cp -a output/voxblox {rel_run_dir}/ || true
        cp output/perf_monitor.json {rel_run_dir}/perf_monitor.json 2>/dev/null || true
        cp output/perf_monitor.log {rel_run_dir}/perf_monitor.log 2>/dev/null || true
        cp output/launch.log {rel_run_dir}/launch.log 2>/dev/null || true
        cp output/rosbag.log {rel_run_dir}/rosbag.log 2>/dev/null || true
        exit $bag_status
        """
    ).strip()
    result = run(docker_base_args(args, bag_path) + ["bash", "-lc", command], check=False)
    return result.returncode == 0


def eval_run(run_dir: Path, gt_path: Path, variant: str) -> Optional[Dict[str, float]]:
    est_path = run_dir / "vins" / "vins_result_loop.csv"
    if not est_path.is_file():
        return None
    eval_dir = run_dir / "eval"
    result = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "tools" / "eval_trajectory.py"),
            "--est",
            str(est_path),
            "--gt",
            str(gt_path),
            "--out-dir",
            str(eval_dir),
            "--name",
            variant,
        ],
        cwd=str(REPO_ROOT),
        check=False,
    )
    if result.returncode != 0:
        return None
    metrics_path = eval_dir / f"{variant}_metrics.json"
    with metrics_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_perf(path: Path) -> Dict:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def get_nested(data: Dict, keys: Iterable[str]) -> Optional[float]:
    current = data
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current if isinstance(current, (int, float)) else None


def row_from_result(seq_name: str, variant: str, run_dir: Path, success: bool, metrics: Optional[Dict]) -> Dict:
    perf = load_perf(run_dir / "perf_monitor.json")
    return {
        "sequence": seq_name,
        "variant": variant,
        "success": int(success and metrics is not None),
        "ate_rmse_m": None if metrics is None else metrics.get("ate_rmse_m"),
        "ate_mean_m": None if metrics is None else metrics.get("ate_mean_m"),
        "ate_max_m": None if metrics is None else metrics.get("ate_max_m"),
        "enhanced_hz": get_nested(perf, ["topics", "enhanced", "hz"]),
        "feature_hz": get_nested(perf, ["topics", "feature", "hz"]),
        "odom_hz": get_nested(perf, ["topics", "odom", "hz"]),
        "mesh_hz": get_nested(perf, ["topics", "mesh", "hz"]),
        "raw_to_enhanced_ms_mean": get_nested(perf, ["latencies_ms", "raw_to_enhanced_ms", "mean"]),
        "image_to_feature_ms_mean": get_nested(perf, ["latencies_ms", "image_to_feature_ms", "mean"]),
        "image_to_odom_ms_mean": get_nested(perf, ["latencies_ms", "image_to_odom_ms", "mean"]),
        "run_dir": str(run_dir.relative_to(REPO_ROOT)),
    }


def fmt(value) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.4f}"
    return str(value)


def write_summary(rows: List[Dict], out_dir: Path) -> None:
    fields = [
        "sequence",
        "variant",
        "success",
        "ate_rmse_m",
        "ate_mean_m",
        "ate_max_m",
        "enhanced_hz",
        "feature_hz",
        "odom_hz",
        "mesh_hz",
        "raw_to_enhanced_ms_mean",
        "image_to_feature_ms_mean",
        "image_to_odom_ms_mean",
        "run_dir",
    ]
    with (out_dir / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    headers = [
        "Seq",
        "Variant",
        "ATE RMSE m",
        "ATE Mean m",
        "ATE Max m",
        "Enhanced Hz",
        "Feature Hz",
        "Odom Hz",
        "Raw->Enh ms",
        "Img->Feat ms",
        "Img->Odom ms",
    ]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        values = [
            row["sequence"],
            row["variant"],
            fmt(row["ate_rmse_m"]),
            fmt(row["ate_mean_m"]),
            fmt(row["ate_max_m"]),
            fmt(row["enhanced_hz"]),
            fmt(row["feature_hz"]),
            fmt(row["odom_hz"]),
            fmt(row["raw_to_enhanced_ms_mean"]),
            fmt(row["image_to_feature_ms_mean"]),
            fmt(row["image_to_odom_ms_mean"]),
        ]
        lines.append("| " + " | ".join(values) + " |")
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    normal_bag = Path(args.normal_bag).expanduser().resolve()
    darkroom_bag = Path(args.darkroom_bag).expanduser().resolve()
    for bag in (normal_bag, darkroom_bag):
        if not bag.is_file():
            raise FileNotFoundError(bag)

    out_dir = Path(args.output_root).expanduser().resolve() / args.timestamp
    out_dir.mkdir(parents=True, exist_ok=True)

    if not args.skip_build:
        build_in_docker(args)

    normal_gt = extract_normal_gt(args, normal_bag, out_dir)
    darkroom_gt = Path(args.gt_root).expanduser().resolve() / "darkroom1.txt"
    if not darkroom_gt.is_file():
        raise FileNotFoundError(darkroom_gt)

    sequences = args.sequence or ["normal", "darkroom1"]
    variants = args.variant or ["baseline", "zero_dce_only", "depth_to_map_only", "full"]
    sequence_info = {
        "normal": {"bag": normal_bag, "gt": normal_gt},
        "darkroom1": {"bag": darkroom_bag, "gt": darkroom_gt},
    }

    rows = []
    for seq_name in sequences:
        for variant in variants:
            run_dir = out_dir / seq_name / variant
            print(f"\n=== {seq_name} / {variant} ===", flush=True)
            success = run_variant(args, seq_name, sequence_info[seq_name]["bag"], variant, run_dir)
            metrics = eval_run(run_dir, sequence_info[seq_name]["gt"], variant) if success else None
            rows.append(row_from_result(seq_name, variant, run_dir, success, metrics))
            write_summary(rows, out_dir)

    write_summary(rows, out_dir)
    print(f"\nsummary: {out_dir / 'summary.md'}")
    print(f"csv: {out_dir / 'summary.csv'}")
    return 0 if all(row["success"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
