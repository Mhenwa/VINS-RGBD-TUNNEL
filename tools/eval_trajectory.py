#!/usr/bin/env python3
"""Evaluate VINS-RGBD CSV trajectory against a generic time tx ty tz GT file."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


MIN_ALIGNMENT_SAMPLES = 3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--est", required=True, help="VINS result CSV path.")
    parser.add_argument("--gt", required=True, help="GT txt path: time tx ty tz.")
    parser.add_argument("--out-dir", required=True, help="Output directory.")
    parser.add_argument("--name", default="run", help="Output filename prefix.")
    return parser.parse_args()


def ensure_sorted(times: np.ndarray, path: Path) -> None:
    if np.any(np.diff(times) < 0):
        raise ValueError(f"Timestamps in {path} are not sorted ascending.")


def dedupe(times: np.ndarray, positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if not np.any(np.diff(times) == 0):
        return times, positions
    mask = np.concatenate(([True], np.diff(times) > 0))
    return times[mask], positions[mask]


def load_vins_csv(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line:
                continue
            parts = [part.strip() for part in line.split(",")]
            while parts and parts[-1] == "":
                parts.pop()
            if len(parts) < 4:
                raise ValueError(f"{path}:{line_number} has fewer than 4 columns.")
            rows.append((float(parts[0]) / 1e9, float(parts[1]), float(parts[2]), float(parts[3])))
    if not rows:
        raise ValueError(f"Estimate file is empty: {path}")
    data = np.asarray(rows, dtype=np.float64)
    ensure_sorted(data[:, 0], path)
    return dedupe(data[:, 0], data[:, 1:4])


def load_gt_txt(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 4:
                raise ValueError(f"{path}:{line_number} has fewer than 4 columns.")
            rows.append((float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])))
    if not rows:
        raise ValueError(f"GT file is empty: {path}")
    data = np.asarray(rows, dtype=np.float64)
    ensure_sorted(data[:, 0], path)
    return dedupe(data[:, 0], data[:, 1:4])


def trim_to_overlap(
    gt_times: np.ndarray,
    gt_positions: np.ndarray,
    est_times: np.ndarray,
    est_positions: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    start = max(gt_times[0], est_times[0])
    end = min(gt_times[-1], est_times[-1])
    if end <= start:
        raise ValueError(
            f"No time overlap. GT [{gt_times[0]:.6f}, {gt_times[-1]:.6f}], "
            f"EST [{est_times[0]:.6f}, {est_times[-1]:.6f}]"
        )
    mask = (gt_times >= start) & (gt_times <= end)
    overlap_times = gt_times[mask]
    gt_overlap = gt_positions[mask]
    if overlap_times.size < MIN_ALIGNMENT_SAMPLES:
        raise ValueError(f"Only {overlap_times.size} overlap samples.")
    est_interp = np.column_stack(
        [np.interp(overlap_times, est_times, est_positions[:, axis]) for axis in range(3)]
    )
    return overlap_times, gt_overlap, est_interp


def align_se3(source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    src_mean = source.mean(axis=0)
    tgt_mean = target.mean(axis=0)
    src = source - src_mean
    tgt = target - tgt_mean
    covariance = src.T @ tgt / source.shape[0]
    u_mat, _, v_t = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(v_t.T @ u_mat.T) < 0:
        correction[-1, -1] = -1.0
    rotation = v_t.T @ correction @ u_mat.T
    translation = tgt_mean - rotation @ src_mean
    return rotation, translation


def apply_transform(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (rotation @ points.T).T + translation


def save_txt(path: Path, times: np.ndarray, positions: np.ndarray) -> None:
    np.savetxt(path, np.column_stack([times, positions]), fmt="%.9f %.9f %.9f %.9f")


def plot_xy(path: Path, gt: np.ndarray, est: np.ndarray, name: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(gt[:, 0], gt[:, 1], label="GT", linewidth=2.0)
    ax.plot(est[:, 0], est[:, 1], label=name, linewidth=2.0)
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory XY")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_xyz(path: Path, times: np.ndarray, gt: np.ndarray, est: np.ndarray, name: str) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    rel_time = times - times[0]
    for axis, label in enumerate(("x", "y", "z")):
        axes[axis].plot(rel_time, gt[:, axis], label="GT", linewidth=1.8)
        axes[axis].plot(rel_time, est[:, axis], label=name, linewidth=1.8)
        axes[axis].set_ylabel(f"{label} [m]")
        axes[axis].grid(True, linestyle="--", alpha=0.35)
        if axis == 0:
            axes[axis].legend()
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_error(path: Path, times: np.ndarray, errors: np.ndarray, rmse: float) -> None:
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(times - times[0], errors, linewidth=1.8)
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("translation error [m]")
    ax.set_title(f"Translation Error vs Time (ATE RMSE = {rmse:.4f} m)")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    est_path = Path(args.est).expanduser().resolve()
    gt_path = Path(args.gt).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    name = args.name.strip() or "run"

    est_times, est_positions = load_vins_csv(est_path)
    gt_times, gt_positions = load_gt_txt(gt_path)
    overlap_times, gt_overlap, est_interp = trim_to_overlap(
        gt_times, gt_positions, est_times, est_positions
    )
    rotation, translation = align_se3(est_interp, gt_overlap)
    est_aligned = apply_transform(est_interp, rotation, translation)
    errors = np.linalg.norm(est_aligned - gt_overlap, axis=1)
    metrics = {
        "estimate_file": str(est_path),
        "gt_file": str(gt_path),
        "sample_count": int(overlap_times.size),
        "overlap_start_sec": float(overlap_times[0]),
        "overlap_end_sec": float(overlap_times[-1]),
        "ate_rmse_m": float(math.sqrt(np.mean(np.square(errors)))),
        "ate_mean_m": float(np.mean(errors)),
        "ate_median_m": float(np.median(errors)),
        "ate_max_m": float(np.max(errors)),
    }

    save_txt(out_dir / f"{name}_aligned_est.txt", overlap_times, est_aligned)
    save_txt(out_dir / f"{name}_gt_used.txt", overlap_times, gt_overlap)
    with (out_dir / f"{name}_metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2, sort_keys=True)
        handle.write("\n")
    plot_xy(out_dir / f"{name}_traj_xy.png", gt_overlap, est_aligned, name)
    plot_xyz(out_dir / f"{name}_traj_xyz_time.png", overlap_times, gt_overlap, est_aligned, name)
    plot_error(out_dir / f"{name}_trans_error_time.png", overlap_times, errors, metrics["ate_rmse_m"])

    print(f"ATE RMSE [m]: {metrics['ate_rmse_m']:.6f}")
    print(f"ATE mean [m]: {metrics['ate_mean_m']:.6f}")
    print(f"ATE max [m]: {metrics['ate_max_m']:.6f}")
    print(f"outputs: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
