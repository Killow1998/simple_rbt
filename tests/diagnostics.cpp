#include "simple_rbt/event_log.hpp"
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>
using namespace simple_rbt::diagnostics;
namespace fs = std::filesystem;
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
int main() {
  auto root = fs::temp_directory_path() /
              ("srbt-logs-test-" + std::to_string(getpid()));
  fs::create_directories(root);
  try {
    auto store = root / "logs";
    fs::create_directories(store);
    auto old = store / "srbt-1000000000000000000-1.jsonl";
    std::ofstream(old) << "old\n";
    fs::resize_file(old, storage_limit - 10);
    std::ofstream(store / "user-notes.txt") << "keep";
    append_batch(store, "{\"schema\":1,\"event\":\"new\"}\n");
    require(!fs::exists(old), "100 MB rotation failed");
    require(fs::exists(store / "user-notes.txt"), "unrelated file removed");
    std::string file;
    for (auto &e : fs::directory_iterator(store))
      if (e.path().extension() == ".jsonl")
        file = e.path();
    std::ofstream(file, std::ios::app) << "{broken";
    append_batch(store, "{\"schema\":1,\"event\":\"recovered\"}\n");
    std::ifstream in(file);
    std::string line;
    int count = 0;
    while (std::getline(in, line)) {
      (void)Json::parse(line);
      ++count;
    }
    require(count == 2, "partial-write recovery failed");
    setenv("SIMPLE_RBT_LOG_DIR", store.c_str(), 1);
    {
      Log events("test", "diagnostic-test");
      events.set_trace(true);
      events.set_modules({"sensor"});
      events.trace("ipc.suppressed", {}, 1);
      events.trace("sensor.selected", {}, 1);
      events.set_modules({"sensor", "ipc", "ros", "control"});
      for (int n = 0; n < 1000; ++n)
        events.trace("scan", {{"sample", sample_key("lidar", n)}}, n);
      require(events.health()["write_batches"] == 0,
              "per-event disk writes detected");
      for (int n = 0; n < 500; ++n)
        events.event("flood", {{"text", std::string(12000, 'x')}});
      require(events.health()["dropped"].get<int>() > 0, "queue unbounded");
    }
    bool marker = false, selected = false, suppressed = false;
    for (auto &e : fs::directory_iterator(store))
      if (e.path().extension() == ".jsonl") {
        std::ifstream input(e.path());
        while (std::getline(input, line)) {
          auto j = Json::parse(line);
          marker |= j.value("event", "") == "logging.dropped";
          selected |= j.value("event", "") == "sensor.selected";
          suppressed |= j.value("event", "") == "ipc.suppressed";
        }
      }
    require(marker, "missing overflow marker");
    require(selected && !suppressed, "module trace filtering failed");
    std::ofstream(root / "blocked") << "file";
    setenv("SIMPLE_RBT_LOG_DIR", (root / "blocked").c_str(), 1);
    {
      Log events("failure");
      events.event("test");
      std::this_thread::sleep_for(std::chrono::milliseconds(1100));
      require(events.health()["write_failures"].get<int>() > 0,
              "write failure hidden");
    }
    fs::remove_all(root);
    std::cout
        << "PASS: quota, recovery, batching, queue bounds, failure reporting\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
