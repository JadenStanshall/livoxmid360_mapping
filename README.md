# livox mapping and localization

3d lidar mapping and localization pipeline for an indoor mobile robot using the livox mid 360.

---

## prerequisites

**install ros2 humble and build tools**:

```bash
sudo apt install ros-humble-desktop ros-humble-ament-cmake-auto
sudo apt install python3-colcon-common-extensions python3-rosdep cmake git
```

## setup

```bash
git clone https://github.com/JadenStanshall/livoxmid360_mapping.git ~/livoxmid360_mapping
cd ~/livoxmid360_mapping
./setup.sh
```

`setup.sh` does:
1. init and clone submodules
2. apply patches
3. install rosdeps
4. builds workspace


---

## running mapping pipeline

### superodom mapping

```bash
./run_superodom_mapping.sh <bag_path> <output_dir>
```

**ex:**
```bash
./run_superodom_mapping.sh /home/jaden/fluff/bags/custom_msg_map_1_2026-05-11-10-28-28 maps/ipr_maps/map_1_so_v2
```

- output file: `<output_dir>/globalMap.ply` (its ply, not pcd)

---

## map viz

```bash
./pcl_viz.sh <map_path.pcd or .ply>

# ex:
./pcl_viz.sh maps/ipr_maps/map_1_spark_v9/globalMap.pcd

# Downsample before displaying (faster for large maps)
./pcl_viz.sh <map_path.pcd or .ply> --voxel 0.05
```

---

## map eval

```bash
python3 scripts/evaluate_map.py <map_dir>
```

**ex:**
```bash
python3 scripts/evaluate_map.py maps/ipr_maps/map_1_spark_v9
```

outputs z span of keyframe trajectory, floor flatness, rotation purity. each graded with pass, warn, fail.

**target:** z span < 0.05 m on a flat indoor floor.

---

## bev png

convert any point cloud into top down png.

```bash
python3 scripts/bev_png.py <map.pcd> [--ppm 100] [--invert] [--output path/to/out.png]
```

**ex:**
```bash
# deafult, dark points on black bg
python3 scripts/bev_png.py maps/final_maps/map_1_so_v2.pcd

# inverted, easier to see
python3 scripts/bev_png.py maps/final_maps/map_1_so_v2.pcd --invert

# specify resolution
python3 scripts/bev_png.py maps/final_maps/map_1_so_v2.pcd --ppm 50
```

- output is saved alongside input file: `<stem>_bev.png` or `<stem>_bev_inverted.png`
- colour scale: intensity field mapped to greyscale; falls back to z height if no intensity field is present
- default resolution: 100 px/m (1 cm/px)
- multiple points in the same pixel are averaged

---

## postprocessing for localization


```bash
python3 scripts/postprocess_map.py \
    --input  maps/ipr_maps/<map_dir>/globalMap.pcd \
    --output maps/final_maps/<name>.pcd
```

**ex:**
```bash
python3 scripts/postprocess_map.py \
    --input  maps/ipr_maps/map_1_spark_v9/globalMap.pcd \
    --output maps/final_maps/map_1.pcd
```

by default, applies 5 cm voxel downsampling and statistical outlier removal. output PCD is what superloc loads as the prior map.


---


## initializing pose

mainly for the localization validation runs, but need to supply `x/y/yaw` to `run_localization.sh`. use the interactive estimator to find the values:

```bash
source ~/ws_livox/install/setup.bash && source ~/livoxmid360_mapping/install/setup.bash
python3 scripts/initial_pose_estimator.py <map.pcd> <bag_path>
```

**ex:**
```bash
python3 scripts/initial_pose_estimator.py \
    maps/final_maps/map_1_so_v2.pcd \
    /home/jaden/fluff/bags/custom_msg_map_2_2026-05-11-10-34-34
```

opens a 2d top down view: the prior map in grey, the first lidar scan from the bag in cyan. use keyboard to roughly align the scan with the map, then press space to let icp snap it to the exact pose.

**pose init tool controls:**

`w / s`: translate y 0.5m
`a / d`: translate x 0.5m
`q / e`: rotate yaw 5deg
`shift + above controls`: fine control 0.05m/0.5deg
`space`: icp refinement
`r`: reset
`enter`: confirm

on `enter`, the tool prints the ready-to-paste `run_localization.sh` command with the init pose.

---

## running localization validation

```bash
./run_localization.sh <bag_path> <map_path> [x] [y] [z] [yaw]
```

**ex:**
```bash
./run_localization.sh \
    /home/jaden/fluff/bags/custom_msg_map_2_2026-05-11-10-34-34 \
    /home/jaden/livoxmid360_mapping/maps/final_maps/map_1_so_v2.pcd \
    -2.4706 -17.8324 0.0 1.7982
```

- launches superloc against the prior map
- replays the specified bag at 1× speed
- records `/laser_odometry`, `/tf`, and `/laser_odom_path` to an odom bag
- runs live localization monitor in the terminal
- launches rviz display
- kills everything cleanly

to kill mid-run:
```bash
./kill_localization.sh
```


**rviz display:**
- **grey** — prior map (`/overall_map`)
- **green** — local accumulated map (`/laser_cloud_map`)
- **cyan** — live scan in map frame (`/registered_scan`)
- **grey box** — robot body at current pose estimate
- **path trail** — robot trajectory (`/robot_path`)

cyan live scan should align with grey prior map. if not, it means the initial pose is wrong or localization is drifting.

> superodom requires `sensor: "livox"` which expects `livox_ros_driver2/msg/CustomMsg` on `/livox/lidar`. PointCloud2 bags will not work.

---

## localization monitor

live dashboard displaying live localization data:

```
 Localization Monitor
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 elapsed                        42.0    s
 rate                            9.8    Hz
 node warnings                    41
 node errors                       0
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 POSE
   x                         -2.4706    m
   y                        -17.8324    m
   yaw                       +103.00    °
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 UNCERTAINTY  (σ from pose covariance)
   not reported by this pipeline
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 MOTION
   speed                       0.241    m/s
   ω                           -1.23    °/s
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 SMOOTHNESS  (last 20/20 frames)
   position jitter              0.63    cm
   yaw jitter                   0.02    °
   max position jump            1.99    cm
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 SCAN MATCHING  (best/worst over 2 s)
   best  (mean NN dist)         1.8     cm
   worst (mean NN dist)         4.3     cm
```

full localization summary is printed covering trajectory stats, frequency consistency, smoothness over the full run, instability events, global drift, and covariance stats.

raw node output is suppressed from the terminal and written to `launch.log`. to watch live run the following in another terminal:
```bash
tail -f <output_dir>/launch.log
```
