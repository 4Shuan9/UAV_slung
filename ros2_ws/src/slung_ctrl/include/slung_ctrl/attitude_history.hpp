#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <iterator>

namespace slung_ctrl {

// PX4: FRD body -> NED world. Vision geometry: FLU body -> ENU world.
inline Eigen::Quaterniond px4_to_enu_flu(const Eigen::Quaterniond &q_ned_frd) {
    Eigen::Matrix3d ned_to_enu;
    ned_to_enu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    const Eigen::Matrix3d flu_to_frd = Eigen::Vector3d(1, -1, -1).asDiagonal();
    return Eigen::Quaterniond(ned_to_enu * q_ned_frd.normalized().toRotationMatrix() *
                              flu_to_frd).normalized();
}

// Capture-time SLERP only: never extrapolate, and never interpolate across an
// estimator attitude reset. The time horizon bounds storage, not swing angle.
class AttitudeHistory {
public:
    enum class Match { ready, wait, too_old };
    explicit AttitudeHistory(int64_t history_ns) : history_ns_(history_ns) {}
    void clear() { samples_.clear(); }

    bool add(int64_t stamp_ns, const Eigen::Quaterniond &q, uint8_t reset_counter) {
        if (stamp_ns <= 0 || !q.coeffs().allFinite() || q.norm() < 1e-6) return false;
        if (!samples_.empty() && stamp_ns <= samples_.back().stamp) return false;
        if (!samples_.empty() && reset_counter != samples_.back().reset_counter) clear();
        samples_.push_back({stamp_ns, q.normalized(), reset_counter});
        // Retain one predecessor for interpolation at the history boundary.
        while (samples_.size() > 2 && samples_[1].stamp < stamp_ns - history_ns_)
            samples_.pop_front();
        return true;
    }

    Match at(int64_t capture_ns, Eigen::Quaterniond &q) const {
        if (samples_.empty() || capture_ns > samples_.back().stamp) return Match::wait;
        if (capture_ns < samples_.front().stamp ||
            capture_ns < samples_.back().stamp - history_ns_) return Match::too_old;
        const auto hi = std::lower_bound(samples_.begin(), samples_.end(), capture_ns,
            [](const Sample &s, int64_t t) { return s.stamp < t; });
        if (hi->stamp == capture_ns) {
            q = hi->q;
        } else {
            const auto lo = std::prev(hi);
            const double fraction = static_cast<double>(capture_ns - lo->stamp) /
                                    static_cast<double>(hi->stamp - lo->stamp);
            q = lo->q.slerp(fraction, hi->q).normalized();
        }
        return Match::ready;
    }
private:
    struct Sample { int64_t stamp; Eigen::Quaterniond q; uint8_t reset_counter; };
    int64_t history_ns_;
    std::deque<Sample> samples_;
};
} // namespace slung_ctrl
