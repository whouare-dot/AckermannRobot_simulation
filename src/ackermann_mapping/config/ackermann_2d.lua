include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link", -- 或者是 imu_link (gyro_link)，但通常用基座标系
  published_frame = "odom",          -- Cartographer 发布 map -> odom
  odom_frame = "odom",               -- 里程计坐标系名字
  provide_odom_frame = false,        -- 必须为 false，因为你的 EKF 节点已经发布了 odom -> base_footprint
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,              -- 你的 EKF 提供了里程计，开启它能辅助建图
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,              -- 对应你的16线激光数据
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D 轨迹构建器参数微调
TRAJECTORY_BUILDER_2D.use_imu_data = true -- 阿克曼底盘强烈建议开启 IMU
TRAJECTORY_BUILDER_2D.min_range = 0.5     -- 对应 sensors.xacro [cite: 15]
TRAJECTORY_BUILDER_2D.max_range = 30.0    -- 虽然 xacro 写 100，但建图通常限制在 30-50m

-- 防止 IMU 数据抖动导致地图错乱
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.

return options