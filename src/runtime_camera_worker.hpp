#pragma once
#include "simple_rbt/cameras.hpp"
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>

namespace simple_rbt {
// One frozen sample in flight. GL lifetime stays entirely on its owning thread;
// model/data are private, and publication remains on the simulation thread.
class CameraWorker {
public:
  CameraWorker(const mjModel *, const mjData *, CameraConfig, OpticalPose,
               OpticalPose);
  ~CameraWorker();
  void submit(const mjData *, OpticalPose, OpticalPose);
  bool ready();
  void wait();
  const CameraPair &frames() const { return frames_; }
  double seconds() const { return seconds_; }

private:
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr,
                                                             mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr,
                                                          mj_deleteData};
  CameraPair frames_;
  OpticalPose color_, depth_;
  double seconds_ = 0;
  std::mutex mutex_;
  std::condition_variable changed_;
  bool busy_ = false, stop_ = false;
  std::exception_ptr error_;
  std::thread thread_;
};
} // namespace simple_rbt
