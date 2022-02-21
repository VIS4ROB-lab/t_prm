#include <benchmarking/benchmark.h>

namespace benchmarking {

Benchmark::Benchmark(const std::string& name, int num_runs) : name(name), m_num_runs(num_runs) {}
Benchmark::~Benchmark() {}

}  // namespace benchmarking