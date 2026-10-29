// Shared wire conversion; included after the selected wrapper's Impl.
void RosBridge::publish_status(const std::string& json) {
  messages::String message; message.data = json; send(impl_->status, message);
}
void RosBridge::publish_imu(const ImuSample& sample) {
  Vec3 gyro{}, acceleration_g{};
  for (int axis = 0; axis < 3; ++axis) {
    gyro[axis] = wire_float32(sample.angular_velocity[axis]);
    acceleration_g[axis] = wire_float32(sample.specific_force[axis] / kG0);
  }
  finite(gyro); finite(acceleration_g);

  messages::Imu raw;
  raw.header = header(sample.stamp_ns, impl_->options.lidar_frame);
  xyz(raw.angular_velocity, gyro); xyz(raw.linear_acceleration, acceleration_g);
  raw.orientation.w = 1.;
  send(impl_->imu, raw);
  messages::Imu si;
  si.header = header(sample.stamp_ns, impl_->options.imu_frame);
  xyz(si.angular_velocity, sample.angular_velocity); xyz(si.linear_acceleration, sample.specific_force);
  si.orientation.w = 0.; si.orientation_covariance[0] = -1.;
  send(impl_->imu_si, si);
}
void RosBridge::publish_camera_imu(const ImuSample& sample) {
  messages::Imu message;
  message.header=header(sample.stamp_ns,impl_->options.camera_imu_frame);
  xyz(message.angular_velocity,sample.angular_velocity);
  xyz(message.linear_acceleration,sample.specific_force);
  message.orientation_covariance[0]=-1.;
  send(impl_->camera_imu,message);
}
void RosBridge::publish_scan(const LidarScan& scan) {
  const auto count = scan.xyz.size();
  messages::Custom custom;
  custom.header = header(scan.stamp_ns, impl_->options.lidar_frame);
  custom.timebase = scan.stamp_ns; custom.point_num = static_cast<std::uint32_t>(count);
  custom.lidar_id = 0; custom.points.resize(count);
  messages::PointCloud cloud;
  if (impl_->points) {
    cloud.header = custom.header; cloud.height = 1; cloud.width = static_cast<std::uint32_t>(count);
    using Field = messages::Field;
    const std::array<const char*, 7> names{"x", "y", "z", "intensity", "tag", "line", "timestamp"};
    const std::array<std::uint32_t, 7> offsets{0, 4, 8, 12, 16, 17, 18};
    const std::array<std::uint8_t, 7> datatypes{Field::FLOAT32, Field::FLOAT32, Field::FLOAT32,
                                             Field::FLOAT32, Field::UINT8, Field::UINT8, Field::FLOAT64};
    cloud.fields.resize(7);
    for (int field = 0; field < 7; ++field) {
      cloud.fields[field].name = names[field]; cloud.fields[field].offset = offsets[field];
      cloud.fields[field].datatype = datatypes[field]; cloud.fields[field].count = 1;
    }
    cloud.is_bigendian = false; cloud.is_dense = true; cloud.point_step = 26;
    cloud.row_step = static_cast<std::uint32_t>(count * 26); cloud.data.resize(count * 26);
  }
  for (std::size_t index = 0; index < count; ++index) {
    auto& point = custom.points[index];
    point.x = scan.xyz[index][0]; point.y = scan.xyz[index][1]; point.z = scan.xyz[index][2];
    point.offset_time = scan.offsets_ns[index]; point.line = scan.lines[index];
    point.tag = 0; point.reflectivity = 100;
    if (impl_->points) {
      auto* target = cloud.data.data() + index * 26;
      little_endian(target, point.x); little_endian(target + 4, point.y); little_endian(target + 8, point.z);
      little_endian(target + 12, 100.f); target[16] = 0; target[17] = point.line;
      little_endian(target + 18, static_cast<double>(scan.stamp_ns + point.offset_time));
    }
  }
  send(impl_->lidar, custom);
  if (impl_->points) send(impl_->points, cloud);
}
void RosBridge::publish_odometry(std::uint64_t ns, const Pose& pose,
                                 const Vec3& linear_velocity, const Vec3& angular_velocity) {
  messages::Odometry message;
  message.header = header(ns, impl_->options.odom_frame); message.child_frame_id = impl_->options.base_frame;
  xyz(message.pose.pose.position, pose.position); quaternion(message.pose.pose.orientation, pose.quaternion);
  xyz(message.twist.twist.linear, linear_velocity); xyz(message.twist.twist.angular, angular_velocity);
  send(impl_->odom, message);
  if (impl_->options.publish_odom_tf)
    impl_->publish_transform(transform(ns, impl_->options.odom_frame, impl_->options.base_frame, pose));
}
void RosBridge::publish_camera(std::uint64_t ns, const CameraFrame& frame, bool panorama) {
  publish_image(ns, frame, panorama ? 1 : 0);
}
void RosBridge::publish_external_camera(std::uint64_t ns,
                                        const CameraFrame &frame) {
  publish_image(ns, frame, 2);
}
void RosBridge::publish_image(std::uint64_t ns, const CameraFrame &frame,
                              int kind) {
  const bool panorama = kind == 1, external = kind == 2;
  const std::string color_frame =
      external ? impl_->external_frame : impl_->options.camera_frame;
  const std::string depth_frame = external ? impl_->external_depth_frame
                                           : impl_->options.camera_depth_frame;
  messages::Image rgb;
  rgb.header = header(ns, color_frame);
  rgb.height = frame.height;
  rgb.width = frame.width;
  rgb.encoding = "rgb8"; rgb.is_bigendian = false; rgb.step = frame.width * 3; rgb.data = frame.rgb;
  messages::Image depth;
  depth.header =
      header(ns, external || (!panorama && impl_->separate_camera_frames)
                     ? depth_frame
                     : color_frame);
  depth.height = frame.height;
  depth.width = frame.width;
  depth.encoding = "32FC1"; depth.is_bigendian = false; depth.step = frame.width * 4;
  depth.data.resize(frame.depth.size() * sizeof(float));
  for (std::size_t index = 0; index < frame.depth.size(); ++index)
    little_endian(depth.data.data() + index * sizeof(float), frame.depth[index]);
  if (!frame.rgb.empty()) send(impl_->rgb[kind], rgb);
  if (!frame.depth.empty()) send(impl_->depth[kind], depth);
  if (!panorama) {
    messages::CameraInfo info;
    info.header = rgb.header; info.height = frame.height; info.width = frame.width;
    const double focal = frame.height / (2. * std::tan(impl_->options.perspective_fov_y_deg * std::acos(-1.) / 360.));
    const double cx = (frame.width - 1.) / 2., cy = (frame.height - 1.) / 2.;
    info.distortion_model = "plumb_bob";
#ifdef SIMPLE_RBT_ROS1
    info.D.assign(5, 0.);
    info.K = {focal, 0., cx, 0., focal, cy, 0., 0., 1.};
    info.R = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    info.P = {focal, 0., cx, 0., 0., focal, cy, 0., 0., 0., 1., 0.};
#else
    info.d.assign(5, 0.);
    info.k = {focal, 0., cx, 0., focal, cy, 0., 0., 1.};
    info.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    info.p = {focal, 0., cx, 0., 0., focal, cy, 0., 0., 0., 1., 0.};
#endif
    send(external ? impl_->external_info : impl_->camera_info, info);
    if (!frame.depth.empty()) {
      info.header = depth.header;
      send(external ? impl_->external_depth_info : impl_->depth_info, info);
    }
  }
}
std::optional<JointCommand> RosBridge::joint_command() const {
  if (!impl_->joints_received ||
      SteadyClock::now() - *impl_->joints_received >=
          std::chrono::duration<double>(impl_->options.command_timeout_s))
    return {};
  return impl_->latest_joints;
}
void RosBridge::publish_joints(std::uint64_t ns, const JointState &state) {

  messages::Joints message;
  message.header = header(ns, impl_->options.base_frame);
  message.name = state.name;
  message.position = state.position;
  message.velocity = state.velocity;
  message.effort = state.effort;
  send(impl_->joints, message);
  if (!impl_->camera_parent.empty()) {
    impl_->publish_transform(transform(ns, impl_->options.base_frame,
                                       impl_->camera_parent, state.tool_pose));
  }
}
