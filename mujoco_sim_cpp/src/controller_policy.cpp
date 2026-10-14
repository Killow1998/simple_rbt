#include "controller_policy.hpp"
#include <array>
#include <cmath>
#include <stdexcept>
namespace simple_rbt {
Policy::Policy(const std::filesystem::path &path, std::size_t actions,
               const char *output_name)
    : last(actions, 0.f), output_name_(output_name) {
  options_.SetIntraOpNumThreads(1);
  options_.SetInterOpNumThreads(1);
  session_ = std::make_unique<Ort::Session>(env_, path.c_str(), options_);
  auto shape =
      session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  if (shape.size() != 2 || shape[1] <= 0)
    throw std::runtime_error("Unsupported ONNX input shape");
  input_size_ = static_cast<std::size_t>(shape[1]);
  auto output_shape =
      session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  // The retained legacy export declares a symbolic action dimension. ORT
  // validates its actual output against our fixed action buffer at Run time.
  if (output_shape.size() != 2 ||
      (output_shape[1] != -1 &&
       output_shape[1] != static_cast<std::int64_t>(actions)))
    throw std::runtime_error("ONNX action shape mismatch");
  const std::array<std::int64_t, 2> dimensions{
      1, static_cast<std::int64_t>(actions)};
  output_tensor_ = Ort::Value::CreateTensor<float>(
      memory_, last.data(), last.size(), dimensions.data(), 2);
}
void Policy::run_into(std::vector<float> &input) {
  if (input.size() != input_size_)
    throw std::runtime_error("ONNX observation shape mismatch");
  if (!input_buffer_) {
    const std::array<std::int64_t, 2> shape{
        1, static_cast<std::int64_t>(input.size())};
    input_tensor_ = Ort::Value::CreateTensor<float>(
        memory_, input.data(), input.size(), shape.data(), 2);
    input_buffer_ = input.data();
  } else if (input.data() != input_buffer_) {
    throw std::logic_error("Policy observation buffer moved after binding");
  }
  const char *input_name = "obs";
  session_->Run(Ort::RunOptions{nullptr}, &input_name, &input_tensor_, 1,
                &output_name_, &output_tensor_, 1);
  for (float value : last)
    if (!std::isfinite(value))
      throw std::runtime_error("Nonfinite ONNX action");
}
} // namespace simple_rbt
