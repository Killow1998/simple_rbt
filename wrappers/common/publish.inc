// Shared wire conversion; included after the selected wrapper's Impl.
void RosBridge::publish_status(const std::string& json) {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  messages::String message; message.data = json; send(impl_->status, message);
}
void RosBridge::publish_imu(const ImuSample& sample) {
  finite(sample.angular_velocity); finite(sample.specific_force);
  Vec3 gyro{}, acceleration_g{};
  for (int axis = 0; axis < 3; ++axis) {
    gyro[axis] = wire_float32(sample.angular_velocity[axis]);
    acceleration_g[axis] = wire_float32(sample.specific_force[axis] / kG0);
  }
  finite(gyro); finite(acceleration_g);
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
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
void RosBridge::publish_scan(const LidarScan& scan) {
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  const auto count = scan.xyz.size();
  if (scan.offsets_ns.size() != count || scan.lines.size() != count)
    throw std::invalid_argument("scan offsets and lines must match xyz count");
  if (count > std::numeric_limits<std::uint32_t>::max() / 26)
    throw std::invalid_argument("scan exceeds ROS pointcloud row_step range");
  for (const auto& xyz_value : scan.xyz) for (float value : xyz_value)
    if (!std::isfinite(value)) throw std::invalid_argument("scan xyz must be finite");
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
  valid_pose(pose); finite(linear_velocity); finite(angular_velocity);
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  messages::Odometry message;
  message.header = header(ns, impl_->options.odom_frame); message.child_frame_id = impl_->options.base_frame;
  xyz(message.pose.pose.position, pose.position); quaternion(message.pose.pose.orientation, pose.quaternion);
  xyz(message.twist.twist.linear, linear_velocity); xyz(message.twist.twist.angular, angular_velocity);
  send(impl_->odom, message);
  if (impl_->options.publish_odom_tf)
    impl_->publish_transform(transform(ns, impl_->options.odom_frame, impl_->options.base_frame, pose));
}
void RosBridge::publish_camera(std::uint64_t ns, const CameraFrame& frame, bool panorama) {
  if (frame.width <= 0 || frame.height <= 0 || frame.width > std::numeric_limits<int>::max() / 4)
    throw std::invalid_argument("camera width and height must be positive and fit ROS row stride");
  const std::size_t count = static_cast<std::size_t>(frame.width) * frame.height;
  if ((frame.rgb.size() != count * 3 && !frame.rgb.empty()) ||
      (frame.depth.size() != count && !frame.depth.empty()) ||
      (frame.rgb.empty() && frame.depth.empty()))
    throw std::invalid_argument("camera RGB/depth sizes must match width and height");
  if (frame.radial_depth != panorama)
    throw std::invalid_argument("camera radial_depth must match panorama kind");
  std::lock_guard<std::mutex> lock(impl_->publication_mutex); impl_->require_open();
  if (!impl_->has_camera) throw std::invalid_argument("camera extrinsics are required to publish camera images");
  const int kind = panorama ? 1 : 0;
  messages::Image rgb;
  rgb.header = header(ns, impl_->options.camera_frame); rgb.height = frame.height; rgb.width = frame.width;
  rgb.encoding = "rgb8"; rgb.is_bigendian = false; rgb.step = frame.width * 3; rgb.data = frame.rgb;
  messages::Image depth;
  depth.header = rgb.header; depth.height = frame.height; depth.width = frame.width;
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
    send(impl_->camera_info, info);
  }
}
