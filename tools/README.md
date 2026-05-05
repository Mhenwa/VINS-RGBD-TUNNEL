# Tools

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
