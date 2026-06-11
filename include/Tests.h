#pragma once
#include <string>
#include <atomic>
#include <fstream>
#include <mutex>
#include <chrono>

class Tarantula;

class Tests {
public:
    explicit Tests(Tarantula& robot);
    ~Tests();

    void startCircleIKValidation(int leg_id);
    void startBodyCircleValidation();
    void startISO9283Repeatability(int leg_id);

    void startTelemetry(const std::string& filename);
    void stopTelemetry();
    bool isTelemetryActive() const;
    uint32_t getTelemetrySampleCount() const;

    void tickTelemetry();

private:
    Tarantula& robot_;

    std::atomic<bool> telemetry_active_{ false };
    std::ofstream     telemetry_file_;
    std::chrono::steady_clock::time_point telemetry_start_time_;
    std::atomic<uint32_t> telemetry_sample_count_{ 0 };
    std::mutex        telemetry_mutex_;

    void runCircleIKValidationSequence(int leg_id);
    void runBodyCircleValidationSequence();
    void runISO9283RepeatabilitySequence(int leg_id);
};
