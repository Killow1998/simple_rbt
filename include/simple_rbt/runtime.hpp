#pragma once
#include "simple_rbt/run_options.hpp"
#include "simple_rbt/transport.hpp"
#include <functional>

namespace simple_rbt {
class RunObserver {
public:
  virtual ~RunObserver() = default;
  virtual void configure(Simulation &, Transport *) = 0;
  virtual bool before_step(Simulation &, Transport *) = 0;
  virtual bool should_stop(Simulation &) = 0;
  virtual void finish(Json &) = 0;
};
using TransportFactory = std::function<std::unique_ptr<Transport>(const Extrinsics&)>;
Json run_simulation(const RunOptions &, RunObserver *observer = nullptr,
                    const TransportFactory& transport_factory = {});
} // namespace simple_rbt
