#include "../src/imu_sampler.hpp"
#include "simple_rbt/core.hpp"
#include <cmath>
#include <iostream>
using namespace simple_rbt;
void require(bool yes, const char *message) {
  if (!yes)
    throw std::runtime_error(message);
}
Vec3 velocity(const Simulation &s, const char *site) {
  mjtNum v[6];
  mj_objectVelocity(s.model(), s.data(), mjOBJ_SITE,
                    mj_name2id(s.model(), mjOBJ_SITE, site), v, 0);
  return {v[3], v[4], v[5]};
}
void native_test() {
  SimulationOptions options;
  options.project_root = SIMPLE_RBT_PROJECT_ROOT;
  options.robot = "scout_mini";
  options.camera_model = "d435i";
  options.mid360_mount =
      Pose{{.3, -.2, .5}, {.7071067811865476, 0., .7071067811865476, 0.}};
  Simulation sim(options);
  auto last_lidar = velocity(sim, "mid360_imu"),
       last_camera = velocity(sim, "camera_imu");
  double maximum = 0.;
  for (int step = 0; step < 4000; ++step) {
    const double t = sim.stamp_ns() * 1e-9;
    sim.set_command(t < 2 || t > 17 ? Vec3{}
                    : t < 7         ? Vec3{.7, 0., .5}
                    : t < 12        ? Vec3{-.5, 0., -.4}
                                    : Vec3{.4, 0., 0.});
    const auto old_lidar = sim.pose("mid360_imu").quaternion;
    const auto old_camera = sim.pose("camera_imu").quaternion;
    sim.step();
    int sensor_index = 0;
    for (const auto site : {"mid360_imu", "camera_imu"}) {
      const auto sample = sensor_index == 0 ? sim.imu() : sim.camera_imu();
      Quat predicted = sensor_index == 0 ? old_lidar : old_camera;
      mju_quatIntegrate(predicted.data(), sample.angular_velocity.data(),
                        sim.model()->opt.timestep);
      Vec3 rotation_error{};
      mju_subQuat(rotation_error.data(), sim.pose(site).quaternion.data(),
                  predicted.data());
      require(mju_norm3(rotation_error.data()) < 1e-9,
              "Gyro interval does not reproduce native rotation");
      auto &last = sensor_index++ == 0 ? last_lidar : last_camera;
      const auto now = velocity(sim, site);
      Vec3 world{};
      mju_rotVecQuat(world.data(), sample.specific_force.data(),
                     sim.pose(site).quaternion.data());
      for (int i = 0; i < 3; ++i)
        maximum = std::max(maximum,
                           std::abs((world[i] + sim.model()->opt.gravity[i]) *
                                        sim.model()->opt.timestep -
                                    (now[i] - last[i])));
      last = now;
    }
  }
  require(maximum < 1e-9,
          "IMU impulse disagrees with native site velocity change");
  std::cout << "native maximum impulse error: " << maximum << " m/s\n";
}
void sampling_test(std::uint64_t dt) {
  // Changing axes, unequal native/output periods and changing specific force.
  ImuSampler sampler(ImuSample{}, Quat{1, 0, 0, 0});
  Vec3 expected{}, measured{};
  std::uint64_t last = 0;
  unsigned emitted = 0;
  for (std::uint64_t t = dt; t <= 700000000; t += dt) {
    const double angle = t * 1e-9;
    Quat q{std::cos(angle / 2), 0, 0, std::sin(angle / 2)}, inverse{};
    mju_negQuat(inverse.data(), q.data());
    Vec3 world{std::sin(angle * 12), 2., 9.81};
    for (int i = 0; i < 3; ++i)
      expected[i] += world[i] * (dt * 1e-9);
    ImuSample input;
    input.stamp_ns = t;
    input.angular_velocity = {0, 0, 1};
    mju_rotVecQuat(input.specific_force.data(), world.data(), inverse.data());
    sampler.advance(input, q, [&](const ImuSample &output, auto bracket) {
      require(output.stamp_ns == (emitted ? last + 5000000 : 2500000),
              "IMU output clock is not 200 Hz");
      require(bracket[0] + 2500000 == output.stamp_ns &&
                  bracket[1] == output.stamp_ns + 2500000,
              "Invalid sample bracket");
      require(std::abs(output.angular_velocity[2] - 1.) < 1e-10,
              "Angular rate has a different support from acceleration");
      last = output.stamp_ns;
      ++emitted;
      const double theta = last * 1e-9;
      Quat rotation{std::cos(theta / 2), 0, 0, std::sin(theta / 2)};
      Vec3 specific{};
      mju_rotVecQuat(specific.data(), output.specific_force.data(),
                     rotation.data());
      for (int i = 0; i < 3; ++i)
        measured[i] += specific[i] * .005;
    });
    if (t % 5000000 == 0)
      for (int i = 0; i < 3; ++i)
        require(std::abs(measured[i] - expected[i]) < 1e-10,
                "200 Hz resampling loses specific impulse or mixes axes");
  }
  require(emitted == 140, "Missing output samples");
}
int main() {
  try {
    native_test();
    for (auto dt : {1000000ULL, 2000000ULL, 5000000ULL, 7000000ULL})
      sampling_test(dt);
    std::cout << "PASS: native IMU and 200 Hz impulse conservation\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
