#pragma once

#include <benchmarking/base.h>

#include <fstream>

namespace benchmarking {

class ResultHandler {
public:
    ResultHandler(std::string name) : m_name(name) {}
    virtual ~ResultHandler() = default;

    virtual void nextBenchmarkIs(std::shared_ptr<Benchmark> benchmark) {}
    virtual void handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) = 0;
    std::string getName() const {
        return m_name;
    }

    virtual void printSummary(std::string planner, int benchmark_idx){};  // called after all runs for a single benchmark are done
    virtual void printSummary() {}                                        // called after everything is done

private:
    std::string m_name;
};

class ResultHandlerPathLength : public ResultHandler {
public:
    ResultHandlerPathLength() : ResultHandler("Path Length"), m_file("path_length.csv") {}
    void handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) override;

    void printSummary(std::string planner, int benchmark_idx) override;

    void nextBenchmarkIs(std::shared_ptr<Benchmark> benchmark) override;

private:
    std::vector<double> m_path_lengths_for_idx;
    std::string last_description;
    std::ofstream m_file;

    std::shared_ptr<Benchmark> m_current_benchmark;
};

class ResultHandlerSuccessRate : public ResultHandler {
public:
    ResultHandlerSuccessRate() : ResultHandler("Success Rate"), m_file("success_rate.csv") {}
    void handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) override;
    void printSummary(std::string planner, int benchmark_idx) override;

private:
    std::vector<bool> m_results_for_idx;
    std::ofstream m_file;
    std::string last_description;
};

class ResultHandlerComputingTime : public ResultHandler {
public:
    ResultHandlerComputingTime() : ResultHandler("Computing Time"), m_file("computing_time.csv") {}
    void handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) override;
    void printSummary(std::string planner, int benchmark_idx) override;

private:
    std::vector<double> m_results_for_idx;
    std::ofstream m_file;
    std::string last_description;
};

class ResultHandlerThetaChanges : public ResultHandler {
public:
    ResultHandlerThetaChanges() : ResultHandler("Theta Changes"), m_file("theta_changes.csv") {}
    void handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) override;
    void printSummary(std::string planner, int benchmark_idx) override;

private:
    std::vector<double> m_results_for_idx;
    std::ofstream m_file;
    std::string last_description;
};

class ResultHandlerTPRMWaiting : public ResultHandler {
public:
    ResultHandlerTPRMWaiting() : ResultHandler("TPRM Waiting") {}
    void handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) override;
    void printSummary(std::string planner, int benchmark_idx) override;

private:
    int m_waiting = 0;
};

class ResultHandlerPathWriter : public ResultHandler {
public:
    ResultHandlerPathWriter() : ResultHandler("Path Writer") {}
    void handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) override;
};

class ResultHandlerAllWriter : public ResultHandler {
public:
    ResultHandlerAllWriter();
    void handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) override;

private:
    std::ofstream m_file;
};

}  // namespace benchmarking