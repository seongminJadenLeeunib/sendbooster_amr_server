include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  -- 고정된 맵 프레임
  tracking_frame = "odom",  -- odom 프레임을 추적하는 프레임
  published_frame = "base_footprint",  -- 로봇의 위치를 나타내는 프레임
  odom_frame = "odom",  -- 오도메트리 프레임
  provide_odom_frame = true,  -- odom 프레임 제공
  publish_frame_projected_to_2d = false,  -- 2D로 변환된 프레임을 발행하지 않음
  use_pose_extrapolator = true,  -- 포즈 보간기 사용
  use_odometry = true,  -- 오도메트리 사용
  use_nav_sat = false,  -- 내비게이션 위성 시스템 사용하지 않음
  use_landmarks = false,  -- 랜드마크 사용하지 않음
  num_laser_scans = 1,  -- 레이저 스캔 수
  num_multi_echo_laser_scans = 0,  -- 멀티 에코 레이저 스캔 수
  num_subdivisions_per_laser_scan = 10,  -- 레이저 스캔의 서브디비전 수
  num_point_clouds = 0,  -- 포인트 클라우드 수
  lookup_transform_timeout_sec = 0.5,  -- 변환 조회 타임아웃
  submap_publish_period_sec = 0.3,  -- 서브맵 발행 주기
  pose_publish_period_sec = 5e-3,  -- 포즈 발행 주기
  trajectory_publish_period_sec = 30e-3,  -- 트레젝토리 발행 주기
  rangefinder_sampling_ratio = 1.,  -- 레인지파인더 샘플링 비율
  odometry_sampling_ratio = 1.,  -- 오도메트리 샘플링 비율
  fixed_frame_pose_sampling_ratio = 1.,  -- 고정된 프레임 포즈 샘플링 비율
  imu_sampling_ratio = 1.,  -- IMU 샘플링 비율
  landmarks_sampling_ratio = 1.,  -- 랜드마크 샘플링 비율
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

return options
