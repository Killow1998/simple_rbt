#include "calibration.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
using namespace simple_rbt;
using ipc::Json;
namespace fs = std::filesystem;
int main(int argc, char **argv) {
  try {
    fs::path session, root, output;
    std::string algorithm = "fast-lio", ros = "ros1";
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (i + 1 >= argc)
        throw std::invalid_argument("Missing argument value");
      const std::string value = argv[++i];
      if (key == "--session-dir")
        session = value;
      else if (key == "--data-root")
        root = value;
      else if (key == "--output")
        output = value;
      else if (key == "--algorithm")
        algorithm = value;
      else if (key == "--ros")
        ros = value;
      else
        throw std::invalid_argument("Unknown argument: " + key);
    }
    if (session.empty() || root.empty() || output.empty() ||
        (ros != "ros1" && ros != "ros2"))
      throw std::invalid_argument(
          "Require --session-dir, --data-root, --output and --ros ros1|ros2");
    if (algorithm != "fast-lio" && algorithm != "fast-livo2")
      throw std::invalid_argument("Unknown tutorial algorithm");
    if (fs::exists(output))
      throw std::invalid_argument("Tutorial output already exists");
    ipc::Client client;
    if (!client.connect(session, "program", {"metadata", "camera"}))
      throw std::runtime_error("Core session is unavailable");
    Json hello, metadata;
    CameraFrame frame;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < deadline && client.connected()) {
      for (const auto &p : client.poll()) {
        if (p.kind == ipc::Kind::Hello)
          hello = ipc::decode_json(p);
        if (p.kind == ipc::Kind::Metadata)
          metadata = ipc::decode_json(p);
        if (p.kind == ipc::Kind::Camera)
          frame = ipc::decode_camera(p);
      }
      if (!hello.empty() && !metadata.empty() &&
          metadata.value("connections", Json::object()).value(ros, 0) > 0 &&
          (algorithm == "fast-lio" || frame.width > 0))
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (hello.empty() || metadata.empty())
      throw std::runtime_error("Core calibration handshake timed out");
    if (metadata.value("connections", Json::object()).value(ros, 0) == 0)
      throw std::runtime_error("Selected ROS adapter did not connect");
    if (hello["options"].value("publish_odom_tf", true) ||
        hello["options"].value("odom_topic", std::string()) !=
            "/ground_truth/odom")
      throw std::runtime_error("Tutorial requires truth TF disabled and "
                               "/ground_truth/odom for independent scoring");
    const auto sensors = metadata.at("sensors");
    if (!sensors.value("mid360", false))
      throw std::runtime_error("Tutorial requires LiDAR and its rigid IMU");
    if (algorithm == "fast-livo2" && (!sensors.value("camera", false) ||
                                      frame.width == 0 || frame.rgb.empty()))
      throw std::runtime_error(
          "Start the simulator with RGB enabled before FAST-LIVO2 calibration");
    Json parameters;
    std::ifstream file(root / "assets/tutorials" / (algorithm + ".json"));
    if (!file)
      throw std::runtime_error("Tutorial parameter template is missing");
    file >> parameters;
    const auto extrinsics = ipc::decode_extrinsics(hello.at("extrinsics"));
    parameters = tutorial::frontend_parameters(
        parameters, extrinsics, algorithm,
        metadata.value("lidar_model", std::string("mid360")));
    Json camera = algorithm == "fast-livo2"
                      ? tutorial::camera_parameters(
                            frame.width, frame.height,
                            hello["options"].at("perspective_fov_y_deg"))
                      : Json::object();
    parameters.update(camera);
    auto envelope = [&](const Json &p) {
      return ros == "ros2" ? Json{{"laserMapping", {{"ros__parameters", p}}}}
                           : p;
    };
    fs::create_directories(output);
    auto save = [&](const char *name, const Json &j) {
      std::ofstream out(output / name);
      out << j.dump(2) << '\n';
      if (!out)
        throw std::runtime_error("Cannot write tutorial configuration");
    };
    save("frontend.json", envelope(parameters));
    save("camera.json", envelope(camera));
    save("contract.json",
         {{"algorithm", algorithm},
          {"ros", ros},
          {"robot", hello.at("robot")},
          {"lidar_model", metadata.value("lidar_model", std::string("mid360"))},
          {"extrinsics", hello.at("extrinsics")},
          {"imu_topic", "/livox/imu_si"},
          {"imu_units", "m/s^2, rad/s"},
          {"timestamp", "simulation capture time"},
          {"ground_truth_input", false},
          {"camera", camera}});
    std::cout << Json{{"state", "calibrated"},
                      {"output", fs::absolute(output).string()},
                      {"algorithm", algorithm}}
              << '\n';
  } catch (const std::exception &e) {
    std::cerr << "Tutorial: " << e.what() << '\n';
    return 1;
  }
}
