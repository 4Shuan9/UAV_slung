#pragma once
// Codex-generated: bound future-dated transport backlog without altering
// sampling timestamps or observer mathematics.
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iterator>

namespace slung_ctrl {
class FutureInputQueue {
public:
    using Clock = std::chrono::steady_clock;
    struct Sample {
        double time, east, north;
        Clock::time_point received;
    };
    enum class Status { empty, waiting, ready, expired };
    void configure(std::size_t capacity, double history_sec) {
        capacity_ = capacity;
        history_sec_ = history_sec;
    }
    bool empty() const { return inputs_.empty(); }
    std::size_t size() const { return inputs_.size(); }
    // A newer known sample beyond the existing future tolerance proves that
    // the source clock is still ahead; do not present an older prefix as fresh.
    bool source_within_tolerance(double now, double future_tolerance) const {
        return inputs_.empty() || inputs_.back().time <= now + future_tolerance;
    }
    void clear() { inputs_.clear(); }
    // Caller already rejects nonfinite values and checks PX4 timestamp order.
    // A duplicate is harmless and must not consume queue capacity.
    bool push(double time, double east, double north) {
        const auto wall_now = Clock::now();
        if (!inputs_.empty()) {
            if (time == inputs_.back().time) return true;
            if (time < inputs_.back().time || time - inputs_.front().time > history_sec_ ||
                std::chrono::duration<double>(wall_now - inputs_.front().received).count() > history_sec_)
                return false;
        }
        if (inputs_.size() >= capacity_) return false;
        inputs_.push_back({time, east, north, wall_now});
        return true;
    }
    Status status(double now, double max_input_age) const {
        if (inputs_.empty()) return Status::empty;
        const auto wall_now = Clock::now();
        if (std::chrono::duration<double>(wall_now - inputs_.front().received).count() > history_sec_)
            return Status::expired;
        // Release only the prefix whose timestamps ROS time has reached.
        // A continuously arriving future tail must not starve this prefix.
        auto end = inputs_.begin();
        while (end != inputs_.end() && end->time <= now) ++end;
        if (end == inputs_.begin()) return Status::waiting;
        const auto &tail = *std::prev(end);
        // A queue is not fresh evidence of an ongoing sensor: the last sample
        // must be current in both simulation and actual receive time.
        if (now - tail.time > max_input_age ||
            std::chrono::duration<double>(wall_now - tail.received).count() > max_input_age)
            return Status::waiting;
        return Status::ready;
    }
    std::deque<Sample> take_ready(double now) {
        std::deque<Sample> result;
        while (!inputs_.empty() && inputs_.front().time <= now) {
            result.push_back(inputs_.front()); inputs_.pop_front();
        }
        return result;
    }
private:
    std::deque<Sample> inputs_;
    std::size_t capacity_ = 0;
    double history_sec_ = 0;
};
} // namespace slung_ctrl
