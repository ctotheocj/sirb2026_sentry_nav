# ign_sim_pointcloud_tool

`ign_sim_pointcloud_tool_node` converts the Ignition Gazebo simulated LiDAR point cloud
into the organized Velodyne-style cloud expected by the Point-LIO configuration used in
simulation.

In the default simulation launch it subscribes to `livox/lidar` and republishes a cloud
with ring/time style fields derived from the configured scan layout.

## Parameters

| Parameter | Default in simulation YAML | Notes |
| --- | --- | --- |
| `pcd_topic` | `livox/lidar` | Input point cloud topic |
| `n_scan` | `32` | Vertical scan lines |
| `horizon_scan` | `1875` | Horizontal bins |
| `ang_bottom` | `7.0` | Vertical angle bottom used by conversion |
| `ang_res_y` | `1.84375` | Vertical angular resolution |

## Build

```bash
colcon build --packages-select ign_sim_pointcloud_tool
```
