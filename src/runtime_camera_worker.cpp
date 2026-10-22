#include "runtime_camera_worker.hpp"
#include "resources.hpp"
#include <future>
#include <stdexcept>

namespace simple_rbt {
CameraWorker::CameraWorker(const mjModel *model, const mjData *data,
                           CameraConfig config, OpticalPose color,
                           OpticalPose depth)
    : model_(mj_copyModel(nullptr, model), mj_deleteModel), color_(color),
      depth_(depth) {
  if (!model_)
    throw std::runtime_error("Cannot copy camera model");
  data_.reset(mj_makeData(model_.get()));
  if (!data_)
    throw std::runtime_error("Cannot allocate camera snapshot");
  mj_copyData(data_.get(), model_.get(), data);
  std::promise<void> initialized;
  auto initialization = initialized.get_future();
  thread_ = std::thread([this, config, &initialized] {
    bool started = false;
    try {
      CameraRig camera(model_.get(), config);
      camera.render_into(data_.get(), color_, depth_, frames_);
      initialized.set_value();
      started = true;
      std::unique_lock<std::mutex> lock(mutex_);
      for (;;) {
        changed_.wait(lock, [&] { return busy_ || stop_; });
        if (stop_)
          break;
        lock.unlock();
        double began = monotonic_seconds();
        camera.render_into(data_.get(), color_, depth_, frames_);
        seconds_ = monotonic_seconds() - began;
        lock.lock();
        busy_ = false;
        changed_.notify_all();
      }
    } catch (...) {
      if (!started)
        initialized.set_exception(std::current_exception());
      else {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = std::current_exception();
        busy_ = false;
        changed_.notify_all();
      }
    }
  });
  try {
    initialization.get();
  } catch (...) {
    thread_.join();
    throw;
  }
}
CameraWorker::~CameraWorker() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    changed_.notify_all();
  }
  thread_.join();
}
void CameraWorker::submit(const mjData *data, OpticalPose color,
                          OpticalPose depth) {
  wait();
  std::lock_guard<std::mutex> lock(mutex_);
  mj_copyData(data_.get(), model_.get(), data);
  color_ = color;
  depth_ = depth;
  busy_ = true;
  changed_.notify_all();
}
bool CameraWorker::ready() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (error_)
    std::rethrow_exception(error_);
  return !busy_;
}
void CameraWorker::wait() {
  std::unique_lock<std::mutex> lock(mutex_);
  changed_.wait(lock, [&] { return !busy_; });
  if (error_)
    std::rethrow_exception(error_);
}
} // namespace simple_rbt
