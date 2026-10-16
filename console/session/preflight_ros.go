package session

import (
	"context"
	"encoding/json"
	"fmt"
	"strings"
	"time"
)

func (c checker) inspectROS(ctx context.Context, version string) DependencyCheck {
	image, err := adapterImage(version)
	if err != nil {
		return DependencyCheck{version, false, "fail", err.Error(), nil}
	}
	distro := "noetic"
	if version == "ros2" {
		distro = "humble"
	}
	details := map[string]any{"image": image, "expected_distro": distro, "scope": "isolated container; host ROS is not required"}
	deadline, cancel := context.WithTimeout(ctx, 15*time.Second)
	defer cancel()
	data, err := c.run(deadline, "docker", "image", "inspect", "--format", "{{.Id}}", image)
	if err != nil {
		return DependencyCheck{version, false, "fail", "Optional ROS image unavailable; prepare adapter / 可选 ROS 镜像尚未就绪: " + strings.TrimSpace(string(data)) + " " + err.Error(), details}
	}
	details["image_id"] = strings.TrimSpace(string(data))
	// Inspect the installation inside the image, rather than trusting host or
	// image environment variables. No network, master, DDS or GPU is started.
	script := "set -e; test -r /opt/ros/" + distro + "/setup.sh; unset ROS_DISTRO; . /opt/ros/" + distro + "/setup.sh; "
	if version == "ros1" {
		script += "actual=$(rosversion -d); test \"$actual\" = noetic; test -d /opt/ros/noetic/share/roscpp; "
	} else {
		script += "test \"$ROS_DISTRO\" = humble; actual=$ROS_DISTRO; prefix=$(ros2 pkg prefix rclcpp); test \"$prefix\" = /opt/ros/humble; "
	}
	script += "printf 'SPROBO_ROS_DISTRO=%s\\n' \"$actual\""
	script += "; /opt/simple-rbt/bin/simple-rbt-ros-session --runtime-info"
	data, err = c.run(deadline, "docker", "run", "--rm", "--pull", "never", "--network", "none", "--entrypoint", "/bin/sh", image, "-c", script)
	observed := ""
	var adapter struct {
		ConfigurableOdometry bool `json:"configurable_odometry"`
		CameraDevices        bool `json:"camera_devices"`
	}
	for _, line := range strings.Split(string(data), "\n") {
		if strings.HasPrefix(line, "{") {
			_ = json.Unmarshal([]byte(line), &adapter)
		}
		if value, ok := strings.CutPrefix(line, "SPROBO_ROS_DISTRO="); ok {
			observed = value
		}
	}
	details["observed_distro"] = observed
	if err != nil || observed != distro {
		return DependencyCheck{version, false, "fail", fmt.Sprintf("ROS installation must be %s / 容器 ROS 版本必须为 %s: %s", distro, distro, strings.TrimSpace(string(data))), details}
	}
	if !adapter.ConfigurableOdometry || !adapter.CameraDevices {
		return DependencyCheck{version, false, "fail", "ROS adapter is outdated; select Prepare adapters / ROS 适配器需要更新，请选择准备适配器", details}
	}
	details["configurable_odometry"] = true
	details["camera_devices"] = true
	return DependencyCheck{version, false, "pass", "Container ROS installation verified / 容器内 ROS 安装已验证: " + distro, details}
}

func CheckROSAdapter(ctx context.Context, version string) DependencyCheck {
	return (checker{run: runCommand}).inspectROS(ctx, version)
}
