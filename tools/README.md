# Tools

## Zero-DCE / Depth-to-map ablation

Use `run_ablation_eval.py` from the repository root to run the full ablation matrix on `Normal.bag` and `darkroom1.bag`.

```bash
python3 tools/run_ablation_eval.py \
  --normal-bag /home/mhenwa/slam/bags/Normal.bag \
  --darkroom-bag /home/mhenwa/slam/bags/darkroom1.bag \
  --gt-root /home/mhenwa/slam/Ground-Challenge/psudo_gt
```

Use `--skip-build` when the Docker catkin workspace is already built. Useful narrower runs:

```bash
python3 tools/run_ablation_eval.py --skip-build --sequence normal --variant full
python3 tools/run_ablation_eval.py --skip-build --sequence darkroom1 --variant baseline
```

The script runs `baseline`, `zero_dce_only`, `depth_to_map_only`, and `full`, records realtime performance with `perf_monitor_node.py`, evaluates trajectories with `eval_trajectory.py`, and writes `summary.csv` plus `summary.md` under `output/ablation/<timestamp>/`.

## Generic trajectory evaluation

Use `eval_trajectory.py` to compare `output/vins/vins_result_loop.csv` with any text GT file whose first four numeric columns are `time tx ty tz`.

```bash
python3 tools/eval_trajectory.py \
  --est output/vins/vins_result_loop.csv \
  --gt output/gt/Normal_vrpn_gt.txt \
  --out-dir output/eval/normal \
  --name normal_full
```

For `Normal.bag`, first extract `/vrpn_client_node/jackal/pose` inside a ROS Melodic environment:

```bash
python tools/extract_pose_stamped_gt.py \
  --bag /data/Normal.bag \
  --topic /vrpn_client_node/jackal/pose \
  --out output/gt/Normal_vrpn_gt.txt
```

## Ground-Challenge trajectory evaluation

Use `eval_ground_challenge.py` after `VINS-RGBD` finishes a sequence run.

Example:

```bash
python3 tools/eval_ground_challenge.py \
  --est output/vins/vins_result_loop.csv \
  --seq darkroom1.bag \
  --gt-root /home/mhenwa/slam/Ground-Challenge/psudo_gt \
  --out-dir output/eval/darkroom1 \
  --name loop
```

The script:

- reads a specified `VINS-RGBD` result csv
- matches `psudo_gt/<seq>.txt`
- uses GT timestamps and interpolates the estimate onto that timeline
- aligns the estimate to GT with a fixed-scale SE3 transform
- writes aligned trajectories, translation metrics, and plots

## Ground-Challenge GT plotting

Use `plot_ground_challenge_gt.py` when you just want to visualize one specified pseudo GT file.

Example:

```bash
python3 tools/plot_ground_challenge_gt.py \
  --gt /home/mhenwa/slam/Ground-Challenge/psudo_gt/darkroom1.txt \
  --out-dir output/gt/darkroom1 \
  --name darkroom1_gt
```

The script:

- reads one specified `psudo_gt/*.txt` file directly
- uses only the first 4 columns: timestamp, tx, ty, tz
- writes a copied trajectory txt plus XY and XYZ-vs-time plots
