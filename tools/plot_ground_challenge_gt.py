#!/usr/bin/env python3
"""Plot a Ground-Challenge pseudo GT trajectory from a specified txt file."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "output"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read a specified Ground-Challenge pseudo GT txt file and export "
            "trajectory plots."
        )
    )
    parser.add_argument(
        "--gt",
        required=True,
        help="Path to a Ground-Challenge pseudo GT txt file, e.g. darkroom1.txt.",
    )
    parser.add_argument(
        "--out-dir",
        help=(
            "Directory where plots and the copied trajectory txt will be written. "
            "Defaults to output/gt/<gt-stem>."
        ),
    )
    parser.add_argument(
        "--name",
        help="Display/output prefix. Defaults to the GT file stem.",
    )
    return parser.parse_args()


def load_pseudo_gt(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 4:
                raise ValueError(f"GT file {path} has fewer than 4 columns on line {line_number}.")
            try:
                timestamp_sec = float(parts[0])
                tx = float(parts[1])
                ty = float(parts[2])
                tz = float(parts[3])
            except ValueError as exc:
                raise ValueError(f"GT file {path} has a non-numeric value on line {line_number}.") from exc
            rows.append((timestamp_sec, tx, ty, tz))

    if not rows:
        raise ValueError(f"GT file {path} is empty.")

    data = np.asarray(rows, dtype=np.float64)
    ensure_sorted_ascending(data[:, 0], path)
    return dedupe_positions(data[:, 0], data[:, 1:4])


def ensure_sorted_ascending(times: np.ndarray, path: Path) -> None:
    if times.ndim != 1:
        raise ValueError(f"Internal error: timestamps for {path} are not 1D.")
    diffs = np.diff(times)
    if np.any(diffs < 0):
        raise ValueError(f"Timestamps in {path} are not sorted ascending.")


def dedupe_positions(times: np.ndarray, positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    diffs = np.diff(times)
    if not np.any(diffs == 0):
        return times, positions
    unique_mask = np.concatenate(([True], diffs > 0))
    deduped_times = times[unique_mask]
    deduped_positions = positions[unique_mask]
    if deduped_times.size < 2:
        raise ValueError(f"After dropping duplicate timestamps, too few samples remain: {deduped_times.size}.")
    return deduped_times, deduped_positions


def save_trajectory_txt(path: Path, times: np.ndarray, positions: np.ndarray) -> None:
    stacked = np.column_stack([times, positions])
    np.savetxt(path, stacked, fmt="%.9f %.6f %.6f %.6f")


def plot_xy(path: Path, positions: np.ndarray, name: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(positions[:, 0], positions[:, 1], label=name, linewidth=2.0)
    ax.set_title("Ground-Challenge GT Trajectory XY")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_xyz_time(path: Path, times: np.ndarray, positions: np.ndarray, name: str) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ("x", "y", "z")
    rel_time = times - times[0]
    for axis, label in enumerate(labels):
        axes[axis].plot(rel_time, positions[:, axis], label=name, linewidth=1.8)
        axes[axis].set_ylabel(f"{label} [m]")
        axes[axis].grid(True, linestyle="--", alpha=0.35)
        if axis == 0:
            axes[axis].legend()
    axes[-1].set_xlabel("time since start [s]")
    fig.suptitle("Ground-Challenge GT Trajectory vs Time")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()

    gt_path = Path(args.gt).expanduser().resolve()
    if not gt_path.is_file():
        raise FileNotFoundError(f"GT file does not exist: {gt_path}")

    display_name = args.name.strip() if args.name else gt_path.stem
    if not display_name:
        display_name = "gt"

    if args.out_dir:
        out_dir = Path(args.out_dir).expanduser().resolve()
    else:
        out_dir = DEFAULT_OUTPUT_ROOT / "gt" / gt_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)

    gt_times, gt_positions = load_pseudo_gt(gt_path)

    save_trajectory_txt(out_dir / f"{display_name}_trajectory.txt", gt_times, gt_positions)
    plot_xy(out_dir / f"{display_name}_traj_xy.png", gt_positions, display_name)
    plot_xyz_time(out_dir / f"{display_name}_traj_xyz_time.png", gt_times, gt_positions, display_name)

    print(f"gt file: {gt_path}")
    print(f"samples: {gt_times.size}")
    print(f"time range [sec]: {gt_times[0]:.6f} -> {gt_times[-1]:.6f}")
    print(f"outputs: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
