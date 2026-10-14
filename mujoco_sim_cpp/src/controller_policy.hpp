#pragma once
#include <filesystem>
#include <memory>
#include <onnxruntime_cxx_api.h>
#include <vector>
namespace simple_rbt {
class Policy {
public:
  std::vector<float> last;
  Policy(const std::filesystem::path &, std::size_t actions,
         const char *output_name = "actions");
  void run_into(std::vector<float> &input);

private:
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "simple_rbt_cpp"};
  Ort::SessionOptions options_;
  std::unique_ptr<Ort::Session> session_;
  Ort::MemoryInfo memory_ =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input_tensor_{nullptr}, output_tensor_{nullptr};
  const char *output_name_;
  const float *input_buffer_ = nullptr;
  std::size_t input_size_ = 0;
};
} // namespace simple_rbt
