#!/usr/bin/env python3
"""Evaluate a VINS-RGBD trajectory against Ground-Challenge pseudo GT."""

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
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "output"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a VINS-RGBD result file against Ground-Challenge pseudo GT "
            "and export aligned trajectories, plots, and translation error metrics."
        )
    )
    parser.add_argument(
        "--est",
        required=True,
        help="Path to a VINS-RGBD result csv, e.g. vins_result_loop.csv.",
    )
    parser.add_argument(
        "--seq",
        required=True,
        help="Sequence name or bag name, e.g. darkroom1 or darkroom1.bag.",
    )
    parser.add_argument(
        "--gt-root",
        required=True,
        help="Directory containing Ground-Challenge pseudo GT txt files.",
    )
    parser.add_argument(
        "--out-dir",
        help=(
            "Directory where metrics, aligned trajectories, and plots will be written. "
            "Defaults to output/eval/<seq>."
        ),
    )
    parser.add_argument(
        "--name",
        help="Display/output prefix. Defaults to the estimate file stem.",
    )
    return parser.parse_args()


def normalize_seq_name(seq: str) -> str:
    seq_name = Path(seq).stem.strip().lower()
    if not seq_name:
        raise ValueError("Sequence name is empty after normalization.")
    return seq_name


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
                raise ValueError(
                    f"Estimate file {path} has fewer than 4 columns on line {line_number}."
                )
            try:
                timestamp_sec = float(parts[0]) / 1e9
                tx = float(parts[1])
                ty = float(parts[2])
                tz = float(parts[3])
            except ValueError as exc:
                raise ValueError(
                    f"Estimate file {path} has a non-numeric value on line {line_number}."
                ) from exc
            rows.append((timestamp_sec, tx, ty, tz))

    if not rows:
        raise ValueError(f"Estimate file {path} is empty.")

    data = np.asarray(rows, dtype=np.float64)
    ensure_sorted_ascending(data[:, 0], path)
    return data[:, 0], data[:, 1:4]


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
    return data[:, 0], data[:, 1:4]


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
    if deduped_times.size < MIN_ALIGNMENT_SAMPLES:
        raise ValueError(f"After dropping duplicate timestamps, too few samples remain: {deduped_times.size}.")
    return deduped_times, deduped_positions


def trim_to_overlap(
    gt_times: np.ndarray,
    gt_positions: np.ndarray,
    est_times: np.ndarray,
    est_positions: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    overlap_start = max(gt_times[0], est_times[0])
    overlap_end = min(gt_times[-1], est_times[-1])
    if overlap_end <= overlap_start:
        raise ValueError(
            "Ground truth and estimate have no overlapping timestamps. "
            f"GT range: [{gt_times[0]:.6f}, {gt_times[-1]:.6f}], "
            f"estimate range: [{est_times[0]:.6f}, {est_times[-1]:.6f}]."
        )

    mask = (gt_times >= overlap_start) & (gt_times <= overlap_end)
    gt_times_overlap = gt_times[mask]
    gt_positions_overlap = gt_positions[mask]
    if gt_times_overlap.size < MIN_ALIGNMENT_SAMPLES:
        raise ValueError(
            f"Only {gt_times_overlap.size} GT samples remain in the overlapping time range; "
            f"need at least {MIN_ALIGNMENT_SAMPLES}."
        )

    est_interp = np.column_stack(
        [np.interp(gt_times_overlap, est_times, est_positions[:, axis]) for axis in range(3)]
    )
    return gt_times_overlap, gt_positions_overlap, est_interp


def align_se3_umeyama(source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if source.shape != target.shape:
        raise ValueError("Source and target trajectories must have the same shape for alignment.")
    if source.shape[0] < MIN_ALIGNMENT_SAMPLES:
        raise ValueError(
            f"Need at least {MIN_ALIGNMENT_SAMPLES} matched samples for SE3 alignment; "
            f"got {source.shape[0]}."
        )

    src_mean = source.mean(axis=0)
    tgt_mean = target.mean(axis=0)
    src_centered = source - src_mean
    tgt_centered = target - tgt_mean

    covariance = src_centered.T @ tgt_centered / source.shape[0]
    u_mat, _, v_t = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(v_t.T @ u_mat.T) < 0:
        correction[-1, -1] = -1.0
    rotation = v_t.T @ correction @ u_mat.T
    translation = tgt_mean - rotation @ src_mean
    return rotation, translation


def apply_transform(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (rotation @ points.T).T + translation


def compute_translation_errors(gt_positions: np.ndarray, est_positions: np.ndarray) -> np.ndarray:
    return np.linalg.norm(est_positions - gt_positions, axis=1)


def save_trajectory_txt(path: Path, times: np.ndarray, positions: np.ndarray) -> None:
    stacked = np.column_stack([times, positions])
    np.savetxt(path, stacked, fmt="%.9f %.6f %.6f %.6f")


def save_metrics_json(path: Path, metrics: dict) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2, sort_keys=True)
        handle.write("\n")


def plot_xy(path: Path, gt_positions: np.ndarray, est_positions: np.ndarray, est_name: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], label="GT", linewidth=2.0)
    ax.plot(est_positions[:, 0], est_positions[:, 1], label=est_name, linewidth=2.0)
    ax.set_title("Trajectory XY")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_xyz_time(
    path: Path,
    times: np.ndarray,
    gt_positions: np.ndarray,
    est_positions: np.ndarray,
    est_name: str,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ("x", "y", "z")
    rel_time = times - times[0]
    for axis, label in enumerate(labels):
        axes[axis].plot(rel_time, gt_positions[:, axis], label="GT", linewidth=1.8)
        axes[axis].plot(rel_time, est_positions[:, axis], label=est_name, linewidth=1.8)
        axes[axis].set_ylabel(f"{label} [m]")
        axes[axis].grid(True, linestyle="--", alpha=0.35)
        if axis == 0:
            axes[axis].legend()
    axes[-1].set_xlabel("time since overlap start [s]")
    fig.suptitle("Trajectory vs Time")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_error_time(path: Path, times: np.ndarray, errors: np.ndarray, ate_rmse: float) -> None:
    fig, ax = plt.subplots(figsize=(10, 4.5))
    rel_time = times - times[0]
    ax.plot(rel_time, errors, linewidth=1.8)
    ax.set_title(f"Translation Error vs Time (ATE RMSE = {ate_rmse:.4f} m)")
    ax.set_xlabel("time since overlap start [s]")
    ax.set_ylabel("translation error [m]")
    ax.grid(True, linestyle="--", alpha=0.35)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def summarize_metrics(
    seq_name: str,
    est_path: Path,
    sample_count: int,
    overlap_start: float,
    overlap_end: float,
    errors: np.ndarray,
) -> dict:
    return {
        "seq": seq_name,
        "estimate_file": str(est_path.resolve()),
        "sample_count": int(sample_count),
        "overlap_start_sec": float(overlap_start),
        "overlap_end_sec": float(overlap_end),
        "ate_rmse_m": float(math.sqrt(np.mean(np.square(errors)))),
        "ate_mean_m": float(np.mean(errors)),
        "ate_median_m": float(np.median(errors)),
        "ate_max_m": float(np.max(errors)),
    }


def main() -> int:
    args = parse_args()

    est_path = Path(args.est).expanduser().resolve()
    if not est_path.is_file():
        raise FileNotFoundError(f"Estimate file does not exist: {est_path}")

    gt_root = Path(args.gt_root).expanduser().resolve()
    if not gt_root.is_dir():
        raise FileNotFoundError(f"GT root directory does not exist: {gt_root}")

    seq_name = normalize_seq_name(args.seq)
    gt_path = gt_root / f"{seq_name}.txt"
    if not gt_path.is_file():
        raise FileNotFoundError(f"GT file does not exist for sequence '{seq_name}': {gt_path}")

    display_name = args.name.strip() if args.name else est_path.stem
    if not display_name:
        display_name = "estimate"

    if args.out_dir:
        out_dir = Path(args.out_dir).expanduser().resolve()
    else:
        out_dir = DEFAULT_OUTPUT_ROOT / "eval" / seq_name
    out_dir.mkdir(parents=True, exist_ok=True)

    est_times, est_positions = load_vins_csv(est_path)
    est_times, est_positions = dedupe_positions(est_times, est_positions)

    gt_times, gt_positions = load_pseudo_gt(gt_path)
    gt_times, gt_positions = dedupe_positions(gt_times, gt_positions)

    overlap_times, gt_overlap, est_interp = trim_to_overlap(
        gt_times, gt_positions, est_times, est_positions
    )
    rotation, translation = align_se3_umeyama(est_interp, gt_overlap)
    est_aligned = apply_transform(est_interp, rotation, translation)
    errors = compute_translation_errors(gt_overlap, est_aligned)
    metrics = summarize_metrics(
        seq_name=seq_name,
        est_path=est_path,
        sample_count=overlap_times.size,
        overlap_start=overlap_times[0],
        overlap_end=overlap_times[-1],
        errors=errors,
    )

    save_trajectory_txt(out_dir / f"{display_name}_aligned_est.txt", overlap_times, est_aligned)
    save_trajectory_txt(out_dir / f"{display_name}_gt_used.txt", overlap_times, gt_overlap)
    save_metrics_json(out_dir / f"{display_name}_metrics.json", metrics)
    plot_xy(out_dir / f"{display_name}_traj_xy.png", gt_overlap, est_aligned, display_name)
    plot_xyz_time(
        out_dir / f"{display_name}_traj_xyz_time.png",
        overlap_times,
        gt_overlap,
        est_aligned,
        display_name,
    )
    plot_error_time(
        out_dir / f"{display_name}_trans_error_time.png",
        overlap_times,
        errors,
        metrics["ate_rmse_m"],
    )

    print(f"seq: {seq_name}")
    print(f"estimate file: {est_path}")
    print(f"time overlap [sec]: {metrics['overlap_start_sec']:.6f} -> {metrics['overlap_end_sec']:.6f}")
    print(f"samples: {metrics['sample_count']}")
    print(f"ATE RMSE [m]: {metrics['ate_rmse_m']:.6f}")
    print(f"ATE mean [m]: {metrics['ate_mean_m']:.6f}")
    print(f"ATE median [m]: {metrics['ate_median_m']:.6f}")
    print(f"ATE max [m]: {metrics['ate_max_m']:.6f}")
    print(f"outputs: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
