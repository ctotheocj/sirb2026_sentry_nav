std_msgs/Header header
nav_msgs/Path path
float64[] timestamps    # 与 path.poses 等长，单位 s，从轨迹起点计 (来自 MINCO 时间参数化)
