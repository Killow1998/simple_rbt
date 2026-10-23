package main

import (
	"fmt"
	"math"
	"regexp"
)

// Validation is shared by startup, profile loading and session restart.
// The core remains authoritative for physics; reject editable errors before
// replacing a working session or launching a worker.
func validateConfig(cfg Config, catalog Catalog) error {
	if err := validateScene(cfg.Scene); err != nil {
		return err
	}
	spec, ok := catalog.Robots[cfg.Robot]
	if !ok {
		return fmt.Errorf("Choose a registered robot / 请选择已注册的机器人: %s", cfg.Robot)
	}
	controller := cfg.Controller
	if controller == "auto" {
		controller = spec.DefaultController
	}
	if _, ok := catalog.Models[cfg.Robot+"-"+controller]; !ok {
		return fmt.Errorf("Choose a controller available for this robot / 请选择此机器人的可用控制器: %s", controller)
	}
	if _, ok := catalog.Terrains[cfg.Terrain+"-"+cfg.TerrainLevel]; !ok {
		return fmt.Errorf("Choose an available terrain and level / 请选择可用的地形与强度")
	}
	if cfg.Scene == nil && spec.Continuous && cfg.Terrain != "flat" && cfg.Terrain != "ramp" && cfg.Terrain != "hills" {
		return fmt.Errorf("This chassis requires flat, ramp or hills / 此底盘请选择平地、坡道或连续起伏")
	}
	if cfg.ControlMode != "native" && cfg.ControlMode != "tracked" {
		return fmt.Errorf("Motion must be native or tracked / 运动模式请选择原生或速度跟踪")
	}
	if cfg.Backend != "auto" && cfg.Backend != "cpu" && cfg.Backend != "cuda" && cfg.Backend != "warp" {
		return fmt.Errorf("LiDAR computation must be auto, cpu or cuda / 雷达计算请选择自动、CPU 或 CUDA")
	}
	if math.IsNaN(cfg.CameraHz) || math.IsInf(cfg.CameraHz, 0) || cfg.CameraHz <= 0 || cfg.CameraHz > 30 {
		return fmt.Errorf("Camera rate must be greater than 0 and at most 30 Hz / 相机频率必须大于 0 且不超过 30 Hz")
	}
	if cfg.CameraModel != "d435" && cfg.CameraModel != "d435i" {
		return fmt.Errorf("Camera model must be d435 or d435i / 相机型号请选择 D435 或 D435i")
	}
	for name, pose := range cfg.CameraExtrinsics {
		if name != "color" && name != "depth" && name != "imu" {
			return fmt.Errorf("Unknown camera calibration frame: %s", name)
		}
		for _, v := range pose.Position {
			if math.IsNaN(v) || math.IsInf(v, 0) || math.Abs(v) > 1 {
				return fmt.Errorf("Invalid camera calibration translation: %s", name)
			}
		}
		norm := 0.0
		for _, v := range pose.Quaternion {
			norm += v * v
		}
		if math.IsNaN(norm) || math.IsInf(norm, 0) || math.Abs(norm-1) > 1e-6 {
			return fmt.Errorf("Camera calibration requires a unit quaternion: %s", name)
		}
	}
	for name, pose := range cfg.Mounts {
		if name != "mid360" && name != "camera" && name != "external_camera" {
			return fmt.Errorf("Unknown sensor mount / 未知传感器外参: %s", name)
		}
		for _, v := range pose.Position {
			if math.IsNaN(v) || math.IsInf(v, 0) || math.Abs(v) > 10 {
				return fmt.Errorf("Mount translation must be finite and within ±10 m / 外参平移须为有限值且在 ±10 米内: %s", name)
			}
		}
		norm := 0.0
		for _, v := range pose.Quaternion {
			norm += v * v
		}
		if math.IsNaN(norm) || math.IsInf(norm, 0) || norm < 1e-12 {
			return fmt.Errorf("Mount quaternion must be finite and nonzero / 外参四元数须为有限值且非零: %s", name)
		}
	}
	for version, enabled := range cfg.Adapters {
		if enabled && version != "ros1" && version != "ros2" {
			return fmt.Errorf("Select ROS 1 Noetic or ROS 2 Humble / 请选择 ROS 1 Noetic 或 ROS 2 Humble")
		}
	}
	if !regexp.MustCompile(`^/([A-Za-z_][A-Za-z0-9_]*)(/[A-Za-z_][A-Za-z0-9_]*)*$`).MatchString(cfg.ROS.OdomTopic) {
		return fmt.Errorf("Odometry topic must be absolute, e.g. /ground_truth/odom / 里程计话题须为绝对名称，例如 /ground_truth/odom")
	}
	return nil
}
