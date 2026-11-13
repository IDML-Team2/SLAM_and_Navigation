# Notebooks – look inside the data

Three Jupyter notebooks to understand single parts of the tutorial by
computing and plotting things yourself. They do **not** replace the terminal
steps of the tutorial (starting SLAM, Nav2, teleop) – those stay in terminals.

| Notebook | What you do | Tutorial | Needs the robot? |
| --- | --- | --- | --- |
| [01_laser_scan_and_lidar_position.ipynb](01_laser_scan_and_lidar_position.ipynb) | open a `LaserScan`, turn distances into points, move them from `laser` to `base_link`, see what wrong values in `robot.yaml` do (incl. a simulated "turning on the spot" test) | 2, 3, 6.4 | no (sample scan) / live: `robot.launch.py` |
| [02_map_files.ipynb](02_map_files.ipynb) | open `.yaml` + `.pgm`, free / occupied / unknown, pixels ↔ meters, can the robot reach a goal? which gaps can it pass? | 6.5, 8 | no – works with the sample map or your own saved map |
| [03_poses_and_goals.ipynb](03_poses_and_goals.ipynb) | quaternions, where is the robot (TF, AMCL), check a goal against the map, send goals from Python and plot the driven path | 3, 10 | sections 1–2 no; 3–5 live: `robot.launch.py` + `navigation.launch.py` |

Every notebook starts with `LIVE = False`: it then uses the files in
`sample_data/` and runs anywhere – e.g. at home. Set `LIVE = True` to use the
real robot (or, in notebook 2, your own map `~/maps/my_map.yaml`).

```
notebooks/
├── 01_laser_scan_and_lidar_position.ipynb
├── 02_map_files.ipynb
├── 03_poses_and_goals.ipynb
├── nb_helpers.py          ← helper functions used by all notebooks (worth reading!)
└── sample_data/
    ├── sample_map.pgm     ← a 6 m x 4 m sample room (same format as map_saver_cli)
    ├── sample_map.yaml
    └── sample_scan.yaml   ← a scan taken in that room at x = 1.0, y = 0.2
```

## Starting Jupyter

Jupyter runs on the **PC** (not on the Jetson). It must be started from a
terminal in which ROS 2 and the workspace are sourced – otherwise the live
parts cannot find ROS.

**ROS 2 Humble installed directly** (Jupyter is part of the packages in
["Before you start"](../../TUTORIAL_SLAM_NAVIGATION.md#before-you-start-install-the-required-software);
only the notebook part: `sudo apt install jupyter-notebook python3-matplotlib`):

```bash
cd ~/imdl_nav_ws/notebooks
jupyter notebook
```

A browser opens with the notebook list.

**Docker** (Jupyter is part of the image; after pulling this change, rebuild
once with `./run.sh --build`):

```bash
nav_notebooks          # inside the container
```

Open the printed link `http://127.0.0.1:8888/?token=...` in the browser on the PC.

**VS Code** works as well: start it from a sourced terminal (`code .`), open a
notebook and choose the kernel "Python 3" (`/usr/bin/python3`).

## Tips

- Run the cells from top to bottom with **Shift + Enter**.
- The stop button (■) interrupts a cell. In notebook 3 it also cancels a
  running Nav2 goal – the robot stops.
- Messages from ROS nodes (e.g. `[nav_client]: Result: reached`) may appear in
  the terminal where you started Jupyter instead of in the notebook.
- Changed `robot.yaml`? Re-run the cell that loads it (`load_robot_config()`).
- Something odd? *Kernel → Restart & Run All* starts from a clean state.
