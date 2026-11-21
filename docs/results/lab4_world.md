# Gurdy 3.0 - world `lab4`: build, self-mapping, doorway check

World: 4 rooms A-D in a ring (A-B-D-C-A), 8 x 8 m, 1 m doorways, 10 objects of classes the detector already knows
(`scripts/gen_world_lab4.py`; layout and objects in its docstring; `maps/lab4.places.yaml`; ground truth `docs/results/data/lab4/lab4_gt.json`).
Same class in different rooms: box (red A / blue B / yellow C), barrel (blue B / red D), traffic cone (A / D). No chair anywhere.

## The robot maps it by itself (`scripts/lab4_explore_run.sh`, data `docs/results/data/lab4/lab4_map/`)
slam_toolbox + Nav2 + frontier explorer, policy `terrain_dr2`, ground-truth odometry, headless, no map at start.
Explorer: "no frontiers left" after **492 s sim**, 19 frontier goals (10 resolved on arrival, 6 close-enough, 3 failed, 1 blacklisted), back at the start (0.105 m).
RTF ~0.54 during this run (other light jobs were running; not a clean RTF measurement).

**Problem found:** the map saved at the end was corrupted. Map snapshots (every 10 s) are clean up to t = 470 s (wall coverage 99.9 %), and between t = 470 and 480 s,
while the robot drove back through room B towards the start, the SLAM map jumped (extent grew from 8 x 8 m to 11.6 x 8.3 m, wall coverage 50 %,
2251 occupied cells > 0.25 m from any wall; `docs/results/img/lab4_map_corrupted_final.png`). Cause not investigated (a late wrong loop closure / scan match
is the likely mechanism; the earlier flat_lab runs did not show it). Handling: the saved occupancy grid of the **last clean snapshot (t ~ 470 s)** is used as
`maps/lab4_map.{yaml,pgm}`; the corrupted final save is kept as `maps/lab4_map_corrupted_final.*`; the slam_toolbox posegraph of the run is corrupted too and was not kept.
This map is what localization (AMCL) uses in all later runs. It is a real map made by the robot, but picked from a snapshot, not the file the run produced.

Map quality (`scripts/plot_map_lab4.py`, vs true walls): **99.9 %** of wall samples have an occupied cell within 2 cells (0.1 m); 98 of 2427 occupied cells lie > 0.25 m from a wall or object
(object shadows / outlines). All 10 objects appear (`docs/results/img/lab4_map.png`: black = occupied, red = true walls, blue = true objects).

## Nav2 passes every doorway (`scripts/lab4_door_check.sh`, localization on `lab4_map`, policy, ground-truth odom)
16 goals around the ring, forward then backwards, each through a doorway centre or room centre: **16 / 16 SUCCEEDED**, final error 0.03-0.20 m (goal tolerance 0.25 m checked: all reached),
0 recoveries, max stall 1 s. Single run (no variance). Data: `docs/results/data/lab4/door_check/goals.txt`.
