-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true, -- 是否使用 IMU 数据，true 时，前端会用 IMU 提供的姿态变化（roll/pitch/yaw）帮助扫描匹配。2D 模式只会用到 roll/pitch 修正，不用 yaw 来优化。
  min_range = 0., -- 激光有效距离范围	过滤掉小于 min_range 或大于 max_range 的点，避免近处干扰或远处噪声影响匹配。
  max_range = 30.,
  min_z = -0.8,
  max_z = 2., -- 点云高度范围	虽然是 2D SLAM，Cartographer 会把输入点云的 Z 值过滤掉不在此范围内的点（主要针对多线激光或 3D 雷达投影到 2D）。
  missing_data_ray_length = 5., -- 缺失数据射线的长度	当激光某个方向无回波时，假设该方向的点距离是多少（用于占据栅格推断空闲区域）。
  num_accumulated_range_data = 1, -- 累积多少帧激光后再做一次匹配	提高每次匹配的点数密度，但会增加延迟。
  voxel_filter_size = 0.025, -- 初步体素滤波的体素大小	用于对输入激光点云进行降采样，减少计算量。

  adaptive_voxel_filter = { -- 对激光数据在匹配前做稀疏化（点数多时滤波，点数少时保留）
    max_length = 0.5, -- 滤波体素最大边长（点密度很高时会使用接近此值的降采样）
    min_num_points = 200, -- 保证降采样后至少有这么多点
    max_range = 50., -- 过滤超出这个范围的点
  },

  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  -- 是否使用 real_time_correlative_scan_matcher 为ceres提供先验信息
  -- 计算复杂度高 , 但是很鲁棒 , 在odom或者imu不准时依然能达到很好的效果
  -- 如只使用单线激光雷达，频率又很低，建图效果不好总是叠图，可以打开，在扫描匹配前就提供先验信息
  use_online_correlative_scan_matching = false, -- 实时相关性扫描匹配，用暴力搜索匹配（slow 但抗大位移失效），一般在传感器噪声大或里程计不准时用。
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1, -- 平移搜索窗口大小（米）
    angular_search_window = math.rad(20.), -- 旋转搜索窗口大小（弧度）
    translation_delta_cost_weight = 1e-1, -- 平移偏移的代价权重
    rotation_delta_cost_weight = 1e-1, -- 旋转偏移的代价权重
  },

  ceres_scan_matcher = {
    occupied_space_weight = 1., -- 点和地图占据概率匹配的权重
    translation_weight = 10., -- 平移差的惩罚权重
    rotation_weight = 40., -- 旋转差的惩罚权重
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  -- 运动过滤器，过滤掉移动很小的帧（防止重复处理相似的扫描）。
  motion_filter = {-- 任何下面一项超出阈值才会保留
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10., -- 用于低通滤波 IMU 的加速度数据，提取重力方向（越大滤波越平滑）。
  pose_extrapolator = {
    use_imu_based = false, -- 用 IMU 驱动外推
    constant_velocity = {-- 用常速模型（constant velocity）外推
      imu_gravity_time_constant = 10., 
      pose_queue_duration = 0.001,
    },
    imu_based = {
      pose_queue_duration = 5., -- 缓存的位姿历史时间长度
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  submaps = {
    num_range_data = 90, -- 一个 submap 累积多少帧数据后固定（完成构建）
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05, -- 地图的分辨率
    },
    range_data_inserter = { -- 控制将激光数据插入地图的规则
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      tsdf_range_data_inserter = {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}
