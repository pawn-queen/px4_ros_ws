// Shared defaults plus native P0 safety and P1 mapping/observation parameters.
// YAML overrides remain authoritative.
#pragma once
#include <rclcpp/rclcpp.hpp>
namespace powerplant {
inline void declare_voxel_mapper_parameters(rclcpp::Node &n) {
  n.declare_parameter<std::string>("localized_odom_topic",
                                   "/powerplant/localization/odom");
  n.declare_parameter<std::string>("depth_image_topic", "/depth_camera");
  n.declare_parameter<std::string>("depth_camera_info_topic",
                                   "/depth_camera/camera_info");
  n.declare_parameter<std::string>("laser_scan_topic", "/scan");
  n.declare_parameter<std::string>("pointcloud_topic", "/points");
  n.declare_parameter<std::string>("occupied_voxels_topic",
                                   "/powerplant/map/occupied_voxels");
  n.declare_parameter<std::string>("observed_voxels_topic",
                                   "/powerplant/map/observed_voxels");
  n.declare_parameter<std::string>("occupancy_grid_topic",
                                   "/powerplant/map/occupancy_grid");
  n.declare_parameter<std::string>("voxel_marker_topic",
                                   "/powerplant/map/voxel_markers");
  n.declare_parameter<std::string>("fusion_status_topic",
                                   "/powerplant/map/fusion_status");
  n.declare_parameter<std::string>("map_frame", "map");
  n.declare_parameter<std::string>("base_frame", "base_link");
  n.declare_parameter<std::string>("localization_status_topic",
                                   "/powerplant/localization/status");
  n.declare_parameter<double>("max_position_variance_m2", 4.0);
  n.declare_parameter<double>("max_localization_age_s", 0.5);
  n.declare_parameter<double>("max_sensor_age_s", 2.0);
  n.declare_parameter<double>("max_future_stamp_skew_s", 0.1);
  n.declare_parameter<double>("voxel_resolution_m", 0.25);
  n.declare_parameter<double>("hit_log_odds", 0.85);
  n.declare_parameter<double>("miss_log_odds", -0.4);
  n.declare_parameter<double>("occupied_log_odds_threshold", 0.0);
  n.declare_parameter<double>("free_log_odds_threshold", -0.2);
  n.declare_parameter<bool>("ray_clear_enabled", true);
  n.declare_parameter<double>("publish_rate_hz", 2.0);
  n.declare_parameter<int64_t>("depth_stride_px", 8);
  n.declare_parameter<bool>("depth_adaptive_sampling_enabled", true);
  n.declare_parameter<int64_t>("depth_min_stride_px", 1);
  n.declare_parameter<double>("depth_max_ray_spacing_m", 0.20);
  n.declare_parameter<double>("depth_edge_threshold_m", 0.25);
  n.declare_parameter<int64_t>("depth_max_rays_per_frame", 12000);
  n.declare_parameter<double>("depth_min_valid_fraction", 0.05);
  n.declare_parameter<double>("depth_fusion_rate_hz", 3.0);
  n.declare_parameter<double>("lidar_fusion_rate_hz", 3.0);
  n.declare_parameter<double>("max_depth_range_m", 18.0);
  n.declare_parameter<double>("min_depth_range_m", 0.25);
  n.declare_parameter<double>("camera_fx", 430.0);
  n.declare_parameter<double>("camera_fy", 430.0);
  n.declare_parameter<double>("camera_cx", 320.0);
  n.declare_parameter<double>("camera_cy", 240.0);
  n.declare_parameter<std::vector<double>>("camera_rotation_rpy_rad",
                                           {0., 0., 0.});
  n.declare_parameter<std::vector<double>>("camera_translation_flu",
                                           {0.12, -0.03, 0.24});
  n.declare_parameter<std::vector<double>>("lidar_rotation_rpy_rad",
                                           {0., 0., 0.});
  n.declare_parameter<std::vector<double>>("lidar_translation_flu",
                                           {0.0, 0.0, 0.0});
  n.declare_parameter<std::string>("pointcloud_frame", "base_link");
  n.declare_parameter<int64_t>("max_publish_points", 60000);
  n.declare_parameter<double>("grid_z_min_m", 0.3);
  n.declare_parameter<double>("grid_z_max_m", 6.0);
  n.declare_parameter<bool>("use_flight_height_grid_band", true);
  n.declare_parameter<double>("grid_z_band_below_m", 0.8);
  n.declare_parameter<double>("grid_z_band_above_m", 0.8);
  n.declare_parameter<bool>("startup_gate_enabled", true);
  n.declare_parameter<double>("startup_gate_min_height_m", 2.0);
  n.declare_parameter<double>("startup_gate_max_tilt_rad", 0.35);
  n.declare_parameter<double>("max_orientation_variance_rad2", 0.5);
  n.declare_parameter<bool>("startup_gate_clear_on_enable", true);
  n.declare_parameter<bool>("use_fixed_grid_bounds", false);
  n.declare_parameter<double>("grid_min_x_m", -25.0);
  n.declare_parameter<double>("grid_max_x_m", 35.0);
  n.declare_parameter<double>("grid_min_y_m", -10.0);
  n.declare_parameter<double>("grid_max_y_m", 50.0);
}
inline void declare_offboard_mission_parameters(rclcpp::Node &n) {
  n.declare_parameter<bool>("motion_model_configured", false);
  n.declare_parameter<double>("motion_max_horizontal_velocity_mps", .6);
  n.declare_parameter<double>("motion_max_vertical_velocity_mps", .3);
  n.declare_parameter<double>("motion_max_horizontal_acceleration_mps2", .5);
  n.declare_parameter<double>("motion_max_vertical_acceleration_mps2", .3);
  n.declare_parameter<double>("motion_brake_horizontal_mps2", .4);
  n.declare_parameter<double>("motion_brake_vertical_mps2", .25);
  n.declare_parameter<double>("motion_reaction_time_s", .3);
  n.declare_parameter<double>("motion_tracking_position_tolerance_m", .2);
  n.declare_parameter<double>("motion_tracking_velocity_tolerance_mps", .25);
  n.declare_parameter<double>("motion_sigma_multiplier", 2.);
  n.declare_parameter<double>("motion_map_margin_m", .05);
  n.declare_parameter<double>("motion_body_radius_m", .35);
  n.declare_parameter<double>("motion_body_z_min_m", 0.);
  n.declare_parameter<double>("motion_body_z_max_m", .5);
  n.declare_parameter<int64_t>("motion_max_check_voxels", 20000);
  n.declare_parameter<double>("motion_max_velocity_variance_m2ps2", 1.);
  n.declare_parameter<double>("motion_yaw_rate_radps", .2);
  n.declare_parameter<double>("motion_yaw_acceleration_radps2", .3);
  n.declare_parameter<double>("motion_settle_speed_mps", .08);
  n.declare_parameter<double>("motion_settle_dwell_s", .3);
  n.declare_parameter<double>("motion_takeoff_bootstrap_height_m", 0.0);
  n.declare_parameter<double>("motion_takeoff_bootstrap_duration_s", 0.0);
  n.declare_parameter<bool>("takeoff_ground_support_enabled", false);
  n.declare_parameter<std::string>("px4_timestamp_domain", "system");
  n.declare_parameter<double>("takeoff_ground_plane_z_m", 0.0);
  n.declare_parameter<double>("takeoff_ground_plane_error_m", .01);
  n.declare_parameter<std::vector<double>>("takeoff_ground_contact_points_flu", std::vector<double>{});
  n.declare_parameter<int64_t>("takeoff_ground_model_revision", 0);
  n.declare_parameter<double>("takeoff_ground_land_max_age_s", 2.0);
  n.declare_parameter<double>("takeoff_ground_transition_timeout_s", 12.0);
  n.declare_parameter<double>("takeoff_ground_max_rise_m", .75);
  n.declare_parameter<double>("motion_input_timeout_s", .3);
  n.declare_parameter<double>("motion_landing_reserve_s", 10.);
  n.declare_parameter<double>("offboard_warmup_min_s", 2.);
  n.declare_parameter<int64_t>("initial_scan_max_moves", 4);
  n.declare_parameter<double>("initial_scan_move_length_m", .3);
  n.declare_parameter<double>("initial_scan_move_wait_s", 4.);
  n.declare_parameter<bool>("explore_topology_enabled", true);
  n.declare_parameter<double>("topology_core_extra_clearance_m", .5);
  n.declare_parameter<double>("topology_switch_hysteresis", .1);
  n.declare_parameter<double>("inspection_sync_tolerance_s", .05);
  n.declare_parameter<double>("inspection_max_observation_age_s", 2.);
  n.declare_parameter<int64_t>("inspection_cache_max_bytes", 100663296);
  n.declare_parameter<double>("inspection_coverage_threshold", .9);
  n.declare_parameter<double>("inspection_surfel_resolution_m", .1);
  n.declare_parameter<int64_t>("inspection_max_view_attempts", 3);
  n.declare_parameter<int64_t>("inspection_min_roi_pixels", 1024);
  n.declare_parameter<double>("inspection_min_laplacian_variance", 40.);
  n.declare_parameter<double>("inspection_max_exposure_fraction", .25);
  n.declare_parameter<double>("inspection_min_depth_support_fraction", .2);
  n.declare_parameter<double>("inspection_capture_timeout_s", 2.);
  n.declare_parameter<double>("inspection_min_modeled_area_m2", .02);
  n.declare_parameter<int64_t>("inspection_max_viewpoints", 32);
  n.declare_parameter<double>("topology_rebuild_interval_s", 2.);
  n.declare_parameter<std::string>("yolo_source_image_topic",
                                   "/powerplant/perception/yolo_source_image");
  n.declare_parameter<std::string>("rgb_camera_info_topic",
                                   "/camera/camera_info");
  n.declare_parameter<std::vector<double>>("rgb_camera_translation_flu",
                                           {0.12, -0.03, 0.24});
  n.declare_parameter<std::vector<double>>("rgb_camera_rotation_rpy_rad",
                                           {0., 0., 0.});
  n.declare_parameter<std::string>("localized_odom_topic",
                                   "/powerplant/localization/odom");
  n.declare_parameter<std::string>("occupancy_grid_topic",
                                   "/powerplant/map/occupancy_grid");
  n.declare_parameter<std::string>("map_fusion_status_topic",
                                   "/powerplant/map/fusion_status");
  n.declare_parameter<std::string>("observed_voxels_topic",
                                   "/powerplant/map/observed_voxels");
  n.declare_parameter<std::string>("planned_path_topic",
                                   "/powerplant/control/planned_path");
  n.declare_parameter<std::string>("yolo_detections_topic",
                                   "/powerplant/perception/yolo_detections");
  n.declare_parameter<std::string>("target_position_topic",
                                   "/powerplant/perception/target_position");
  n.declare_parameter<std::string>("inspection_event_topic",
                                   "/powerplant/control/inspection_events");
  n.declare_parameter<std::string>("depth_image_topic", "/depth_camera");
  n.declare_parameter<std::string>("laser_scan_topic", "/mapping_lidar");
  n.declare_parameter<std::string>("depth_camera_info_topic",
                                   "/depth_camera/camera_info");
  n.declare_parameter<std::string>("rgb_image_topic", "/camera");
  n.declare_parameter<std::string>("vehicle_status_topic",
                                   "/fmu/out/vehicle_status");
  n.declare_parameter<std::string>("vehicle_land_detected_topic",
                                   "/fmu/out/vehicle_land_detected");
  n.declare_parameter<std::string>("localization_status_topic",
                                   "/powerplant/localization/status");
  n.declare_parameter<std::string>("battery_status_topic",
                                   "/fmu/out/battery_status");
  n.declare_parameter<std::string>("failsafe_flags_topic",
                                   "/fmu/out/failsafe_flags");
  n.declare_parameter<std::string>("vehicle_command_ack_topic",
                                   "/fmu/out/vehicle_command_ack");
  n.declare_parameter<std::string>("offboard_control_mode_topic",
                                   "/fmu/in/offboard_control_mode");
  n.declare_parameter<std::string>("trajectory_setpoint_topic",
                                   "/fmu/in/trajectory_setpoint");
  n.declare_parameter<std::string>("vehicle_command_topic",
                                   "/fmu/in/vehicle_command");
  n.declare_parameter<std::string>("mission_status_topic",
                                   "/powerplant/control/mission_status");
  n.declare_parameter<std::string>("map_frame", "map");
  n.declare_parameter<std::string>("base_frame", "base_link");
  n.declare_parameter<bool>("auto_start", false);
  n.declare_parameter<bool>("arm_on_start", true);
  n.declare_parameter<double>("command_retry_interval_s", 1.0);
  n.declare_parameter<int64_t>("offboard_warmup_setpoints", 20);
  n.declare_parameter<bool>("require_localization_for_start", true);
  n.declare_parameter<double>("max_localization_age_s", 0.5);
  n.declare_parameter<double>("max_future_stamp_skew_s", 0.1);
  n.declare_parameter<double>("max_vehicle_status_age_s", 1.0);
  n.declare_parameter<double>("max_land_detection_age_s", 1.0);
  n.declare_parameter<double>("max_start_yaw_variance_rad2", 0.5);
  n.declare_parameter<double>("max_start_position_variance_m2", 4.0);
  n.declare_parameter<double>("max_start_height_variance_m2", 4.0);
  n.declare_parameter<bool>("require_preflight_checks", true);
  n.declare_parameter<bool>("require_accepts_offboard_setpoints", false);
  n.declare_parameter<double>("takeoff_height_m", 5.0);
  n.declare_parameter<bool>("takeoff_region_confirmed", false);
  n.declare_parameter<double>("takeoff_region_radius_m", 1.0);
  n.declare_parameter<double>("takeoff_region_height_m", 0.0);
  n.declare_parameter<double>("initial_scan_hover_s", 2.0);
  n.declare_parameter<int64_t>("initial_scan_yaw_steps", 12);
  n.declare_parameter<int64_t>("initial_scan_coverage_bins", 72);
  n.declare_parameter<double>("initial_scan_required_coverage_fraction", 0.9);
  n.declare_parameter<double>("initial_scan_min_valid_depth_fraction", 0.2);
  n.declare_parameter<double>("initial_scan_min_free_path_m", 0.75);
  n.declare_parameter<double>("initial_scan_heading_timeout_s", 3.0);
  n.declare_parameter<double>("initial_scan_yaw_rate_radps", 0.2);
  n.declare_parameter<double>("initial_scan_yaw_dwell_s", 0.3);
  n.declare_parameter<double>("initial_scan_yaw_tolerance_rad", 0.12);
  n.declare_parameter<double>("initial_scan_max_tilt_rad", 0.35);
  n.declare_parameter<double>("initial_scan_max_tilt_variance_rad2", 0.5);
  n.declare_parameter<double>("initial_scan_max_speed_mps", 0.35);
  n.declare_parameter<double>("initial_scan_max_hover_yaw_rate_radps", 0.12);
  n.declare_parameter<double>("initial_scan_max_position_error_m", 0.5);
  n.declare_parameter<double>("initial_scan_max_horizontal_setpoint_speed_mps",
                              0.3);
  n.declare_parameter<double>("initial_scan_max_sensor_age_s", 2.0);
  n.declare_parameter<int64_t>("initial_scan_min_valid_lidar_ranges", 10);
  n.declare_parameter<int64_t>("initial_scan_min_known_grid_cells", 10);
  n.declare_parameter<double>("initial_scan_timeout_s", 90.0);
  n.declare_parameter<double>("return_home_height_m", 5.0);
  n.declare_parameter<double>("target_acceptance_m", 0.45);
  n.declare_parameter<double>("yaw_acceptance_rad", 0.25);
  n.declare_parameter<double>("cruise_yaw_enu_rad", 0.0);
  n.declare_parameter<double>("max_horizontal_setpoint_speed_mps", 2.0);
  n.declare_parameter<double>("max_vertical_setpoint_speed_mps", 0.8);
  n.declare_parameter<double>("max_yaw_rate_radps", 0.6);
  n.declare_parameter<bool>("avoidance_startup_gate_enabled", true);
  n.declare_parameter<double>("avoidance_startup_min_height_m", 2.0);
  n.declare_parameter<double>("avoidance_startup_max_tilt_rad", 0.35);
  n.declare_parameter<double>("replan_interval_s", 2.0);
  n.declare_parameter<bool>("safety_stop_enabled", true);
  n.declare_parameter<double>("safety_stop_distance_m", 1.2);
  n.declare_parameter<double>("safety_stop_width_m", 0.5);
  n.declare_parameter<bool>("safety_stop_treat_unknown_as_blocked", false);
  n.declare_parameter<std::string>("target_class_name", "gas_cylinders");
  n.declare_parameter<double>("min_target_confidence", 0.35);
  n.declare_parameter<double>("target_memory_s", 8.0);
  n.declare_parameter<bool>("lock_target_after_acquisition", true);
  n.declare_parameter<double>("explore_max_duration_s", 300.0);
  n.declare_parameter<double>("explore_task_duration_s", 0.0);
  n.declare_parameter<double>("explore_min_replan_interval_s", 1.0);
  n.declare_parameter<double>("explore_max_map_age_s", 3.0);
  n.declare_parameter<double>("explore_min_x_m", -24.0);
  n.declare_parameter<double>("explore_max_x_m", 34.0);
  n.declare_parameter<double>("explore_min_y_m", -9.0);
  n.declare_parameter<double>("explore_max_y_m", 49.0);
  n.declare_parameter<double>("explore_min_z_m", 3.5);
  n.declare_parameter<double>("explore_max_z_m", 7.0);
  n.declare_parameter<double>("roi_access_max_duration_s", 60.0);
  n.declare_parameter<double>("roi_access_margin_m", 10.0);
  n.declare_parameter<double>("roi_access_revisit_penalty_s", 5.0);
  n.declare_parameter<double>("explore_clearance_radius_m", 0.35);
  n.declare_parameter<double>("explore_max_search_radius_m", 10.0);
  n.declare_parameter<double>("explore_sensor_range_m", 8.0);
  n.declare_parameter<double>("explore_local_path_length_m", 2.0);
  n.declare_parameter<double>("explore_viewpoint_dwell_s", 0.4);
  n.declare_parameter<double>("explore_view_yaw_rate_radps", 0.6);
  n.declare_parameter<bool>("explore_require_camera_info", true);
  n.declare_parameter<int64_t>("explore_max_candidates", 96);
  n.declare_parameter<int64_t>("explore_max_frontier_targets", 400);
  n.declare_parameter<int64_t>("explore_yaw_bins", 12);
  n.declare_parameter<double>("explore_candidate_spacing_m", 0.75);
  n.declare_parameter<double>("explore_min_goal_distance_m", 0.75);
  n.declare_parameter<double>("explore_information_weight", 3.0);
  n.declare_parameter<double>("explore_surface_weight", 2.0);
  n.declare_parameter<double>("explore_travel_time_weight", 0.12);
  n.declare_parameter<double>("explore_clearance_weight", 0.5);
  n.declare_parameter<double>("explore_localization_weight", 0.5);
  n.declare_parameter<double>("explore_reserve_weight", 0.5);
  n.declare_parameter<double>("explore_cruise_speed_mps", 1.0);
  n.declare_parameter<double>("explore_return_speed_mps", 1.0);
  n.declare_parameter<double>("explore_return_reserve_s", 30.0);
  n.declare_parameter<double>("explore_max_position_variance_m2", 1.0);
  n.declare_parameter<double>("explore_localization_variance_growth_m2_per_m",
                              0.01);
  n.declare_parameter<double>("explore_min_battery_fraction", 0.2);
  n.declare_parameter<double>("explore_battery_max_age_s", 3.0);
  n.declare_parameter<double>("target_standoff_m", 2.2);
  n.declare_parameter<double>("inspection_altitude_m", 5.0);
  n.declare_parameter<int64_t>("inspection_orbit_points", 4);
  n.declare_parameter<int64_t>("inspection_required_photos", 4);
  n.declare_parameter<std::string>("inspection_photo_dir",
                                   "/home/pawn/px4_ros_ws/inspection_photos");
  n.declare_parameter<bool>("require_image_for_photo", true);
  n.declare_parameter<double>("max_photo_age_s", 1.0);
  n.declare_parameter<double>("camera_fx", 430.0);
  n.declare_parameter<double>("camera_fy", 430.0);
  n.declare_parameter<double>("camera_cx", 320.0);
  n.declare_parameter<double>("camera_cy", 240.0);
  n.declare_parameter<std::vector<double>>("camera_rotation_rpy_rad",
                                           {0., 0., 0.});
  n.declare_parameter<std::vector<double>>("camera_translation_flu",
                                           {0.12, -0.03, 0.24});
  n.declare_parameter<double>("bbox_depth_sample_fraction", 0.45);
  n.declare_parameter<double>("min_detection_depth_m", 0.25);
  n.declare_parameter<double>("max_detection_depth_m", 18.0);
  n.declare_parameter<double>("max_depth_age_s", 1.0);
  n.declare_parameter<double>("land_complete_height_m", 0.25);
  n.declare_parameter<double>("land_complete_timeout_s", 20.0);
  n.declare_parameter<double>("landing_confirm_dwell_s", 1.0);
}
} // namespace powerplant
