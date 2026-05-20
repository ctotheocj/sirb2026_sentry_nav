int32 id
float64 x
float64 y
float64 vx
float64 vy
float64 radius
float32 confidence          # 0~1, track age归一化
geometry_msgs/Point[] predicted_positions  # 未来N步预测位置 (x,y,z=0)
float32 prediction_dt       # 预测步长 (s)
