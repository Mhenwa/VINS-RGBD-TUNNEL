#!/usr/bin/env python3
"""Plot one or more VINS-RGBD result CSV trajectories."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "output"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read VINS-RGBD result CSV files and export overlaid trajectory plots. "
            "Only the first 4 columns are used: timestamp_ns, tx, ty, tz."
        )
    )
    parser.add_argument(
        "--est",
        action="append",
        required=True,
        help="Path to a VINS-RGBD result CSV. Pass multiple --est values to compare runs.",
    )
    parser.add_argument(
        "--label",
        action="append",
        help="Display label for each --est. Defaults to each file stem.",
    )
    parser.add_argument(
        "--out-dir",
        help="Directory where plots will be written. Defaults to output/plots/vins.",
    )
    parser.add_argument(
        "--prefix",
        default="vins_results",
        help="Output filename prefix. Defaults to vins_results.",
    )
    return parser.parse_args()


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
    return dedupe_positions(data[:, 0], data[:, 1:4])


def ensure_sorted_ascending(times: np.ndarray, path: Path) -> None:
    diffs = np.diff(times)
    if np.any(diffs < 0):
        raise ValueError(f"Timestamps in {path} are not sorted ascending.")


def dedupe_positions(times: np.ndarray, positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    diffs = np.diff(times)
    if not np.any(diffs == 0):
        return times, positions
    unique_mask = np.concatenate(([True], diffs > 0))
    return times[unique_mask], positions[unique_mask]


def plot_xy(path: Path, trajectories: Sequence[Tuple[str, np.ndarray]]) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    for label, positions in trajectories:
        ax.plot(positions[:, 0], positions[:, 1], label=label, linewidth=2.0)
        ax.scatter(positions[0, 0], positions[0, 1], marker="o", s=30)
        ax.scatter(positions[-1, 0], positions[-1, 1], marker="x", s=40)
    ax.set_title("VINS-RGBD Trajectory XY")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_xyz_time(path: Path, trajectories: Sequence[Tuple[str, np.ndarray, np.ndarray]]) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ("x", "y", "z")
    time_origin = min(times[0] for _, times, _ in trajectories)
    for axis, coord_label in enumerate(labels):
        for label, times, positions in trajectories:
            axes[axis].plot(
                times - time_origin,
                positions[:, axis],
                label=label,
                linewidth=1.8,
            )
        axes[axis].set_ylabel(f"{coord_label} [m]")
        axes[axis].grid(True, linestyle="--", alpha=0.35)
        if axis == 0:
            axes[axis].legend()
    axes[-1].set_xlabel("time since earliest result start [s]")
    fig.suptitle("VINS-RGBD Trajectory vs Time")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()

    est_paths = [Path(est).expanduser().resolve() for est in args.est]
    labels = args.label if args.label else [path.stem for path in est_paths]
    if len(labels) != len(est_paths):
        raise ValueError(f"Got {len(labels)} labels for {len(est_paths)} estimate files.")

    if args.out_dir:
        out_dir = Path(args.out_dir).expanduser().resolve()
    else:
        out_dir = DEFAULT_OUTPUT_ROOT / "plots" / "vins"
    out_dir.mkdir(parents=True, exist_ok=True)

    trajectories = []
    for label, est_path in zip(labels, est_paths):
        if not est_path.is_file():
            raise FileNotFoundError(f"Estimate file does not exist: {est_path}")
        times, positions = load_vins_csv(est_path)
        trajectories.append((label, times, positions))

    plot_xy(out_dir / f"{args.prefix}_traj_xy.png", [(label, positions) for label, _, positions in trajectories])
    plot_xyz_time(out_dir / f"{args.prefix}_traj_xyz_time.png", trajectories)

    for label, times, positions in trajectories:
        print(f"{label}: {times.size} samples")
        print(f"  time range [sec]: {times[0]:.6f} -> {times[-1]:.6f}")
        print(
            "  position start/end [m]: "
            f"({positions[0, 0]:.5f}, {positions[0, 1]:.5f}, {positions[0, 2]:.5f}) -> "
            f"({positions[-1, 0]:.5f}, {positions[-1, 1]:.5f}, {positions[-1, 2]:.5f})"
        )
    print(f"outputs: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
