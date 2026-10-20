// ==============================================================================
// @File  : ekf3.hpp
// @Brief : 两轴三状态 EKF 数学核心 + 固定窗口事件回放，无 ROS 依赖。
//          Internal units: rad, rad/s, rad/s^2; channel 0 = East, 1 = North.
// ==============================================================================
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace slung_ctrl {

struct EKFParameters {
    double g = 9.81;
    double l = 1.0;
    double q_omega = 0.01;  // rad^2/s^3, effective continuous noise intensity
    double q_f = 0.05;      // rad^2/s^5, disturbance random walk intensity
    double max_step = 0.005;
    double initial_rate_std = 0.5;
    double initial_disturbance_std = 0.5;
    double nis_threshold = 9.0;  // 1 DoF; <= 0 disables gating
    double history_sec = 1.0;
    std::size_t max_events = 2048;

    void validate() const {
        for (double value : {g, l, q_omega, q_f, max_step, initial_rate_std,
                             initial_disturbance_std, nis_threshold, history_sec}) {
            if (!std::isfinite(value)) throw std::invalid_argument("Non-finite EKF parameter");
        }
        if (g <= 0 || l <= 0 || q_omega < 0 || q_f < 0 || max_step <= 0 ||
            initial_rate_std <= 0 || initial_disturbance_std <= 0 ||
            history_sec <= 0 || max_events < 4) {
            throw std::invalid_argument("Invalid EKF parameters");
        }
    }
};

struct AxisState {
    Eigen::Vector3d x = Eigen::Vector3d::Zero();
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
};

inline Eigen::Vector3d drift(const Eigen::Vector3d &x, double a, const EKFParameters &p) {
    return {x(1), -p.g / p.l * std::sin(x(0)) - a / p.l * std::cos(x(0)) + x(2), 0.0};
}

inline Eigen::Matrix3d jacobian(const Eigen::Vector3d &x, double a, const EKFParameters &p) {
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    A(0, 1) = 1.0;
    A(1, 0) = -p.g / p.l * std::cos(x(0)) + a / p.l * std::sin(x(0));
    A(1, 2) = 1.0;
    return A;
}

// Coupled RK4 of mean and P_dot = A P + P A^T + G Qc G^T.
// This is equivalent to Phi P Phi^T + Q at the continuous linearization level.
inline void predict_axis(AxisState &s, double a, double dt, const EKFParameters &p) {
    if (!std::isfinite(dt) || dt < 0 || !std::isfinite(a)) {
        throw std::invalid_argument("Invalid prediction interval/input");
    }
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    W(1, 1) = p.q_omega;
    W(2, 2) = p.q_f;
    const auto derivative = [&](const Eigen::Vector3d &x, const Eigen::Matrix3d &P) {
        const Eigen::Matrix3d A = jacobian(x, a, p);
        return std::make_pair(drift(x, a, p), Eigen::Matrix3d(A * P + P * A.transpose() + W));
    };
    const int steps = std::max(1, static_cast<int>(std::ceil(dt / p.max_step)));
    const double h = dt / steps;
    for (int i = 0; i < steps; ++i) {
        const auto k1 = derivative(s.x, s.P);
        const auto k2 = derivative(s.x + h / 2 * k1.first, s.P + h / 2 * k1.second);
        const auto k3 = derivative(s.x + h / 2 * k2.first, s.P + h / 2 * k2.second);
        const auto k4 = derivative(s.x + h * k3.first, s.P + h * k3.second);
        s.x += h / 6 * (k1.first + 2 * k2.first + 2 * k3.first + k4.first);
        s.P += h / 6 * (k1.second + 2 * k2.second + 2 * k3.second + k4.second);
        s.P = (0.5 * (s.P + s.P.transpose())).eval();
    }
    if (!s.x.allFinite() || !s.P.allFinite()) throw std::runtime_error("Non-finite prediction");
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(s.P);
    if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() < -1e-10) {
        throw std::runtime_error("Prediction covariance lost positive semidefiniteness");
    }
}

inline bool correct_axis(AxisState &s, double y, double R, double threshold, double &nis) {
    if (!std::isfinite(y) || !std::isfinite(R) || R <= 0) {
        throw std::invalid_argument("Invalid visual measurement");
    }
    const double r = y - s.x(0);  // local downward-angle chart; no branch crossing
    const double S = s.P(0, 0) + R;
    if (!std::isfinite(S) || S <= 0) throw std::runtime_error("Invalid innovation covariance");
    nis = r * r / S;
    if (threshold > 0 && nis > threshold) return false;
    const Eigen::Vector3d K = s.P.col(0) / S;
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    M.col(0) -= K;
    s.x += K * r;
    s.P = M * s.P * M.transpose() + R * K * K.transpose();
    s.P = (0.5 * (s.P + s.P.transpose())).eval();
    return true;
}

struct Estimate {
    double time = 0.0;
    std::array<AxisState, 2> axis;
    std::array<double, 2> accel{{0.0, 0.0}};
    std::array<double, 2> last_vision{{-1.0, -1.0}};
    std::array<double, 2> nis{{0.0, 0.0}};
};

enum class EventResult { accepted, duplicate, too_old, no_input, invalid };

class DelayedSwingEKF {
public:
    explicit DelayedSwingEKF(EKFParameters parameters = {}) : p_(parameters) { p_.validate(); }

    bool initialized() const { return initialized_; }
    std::size_t event_count() const { return events_.size(); }
    double base_time() const { return base_.time; }
    void reset() { initialized_ = false; events_.clear(); pending_inputs_.clear(); }

    EventResult add_input(double time, double east, double north) {
        if (!std::isfinite(time) || time < 0 || !std::isfinite(east) || !std::isfinite(north)) {
            return EventResult::invalid;
        }
        Event e; e.time = time; e.input = true; e.values = {{east, north}};
        if (!initialized_) {
            auto it = std::lower_bound(pending_inputs_.begin(), pending_inputs_.end(), e, before);
            if (it != pending_inputs_.end() && it->time == time) return EventResult::duplicate;
            pending_inputs_.insert(it, e);
            while (pending_inputs_.size() > p_.max_events ||
                   pending_inputs_.back().time - pending_inputs_.front().time > p_.history_sec) {
                pending_inputs_.erase(pending_inputs_.begin());
            }
            return EventResult::accepted;
        }
        return insert(e);
    }

    EventResult add_vision(double time, double east, double north,
                           double variance_east, double variance_north, double time_std = 0.0) {
        if (!std::isfinite(time) || time < 0 || !std::isfinite(east) || !std::isfinite(north) ||
            !std::isfinite(variance_east) || !std::isfinite(variance_north) ||
            variance_east <= 0 || variance_north <= 0 || !std::isfinite(time_std) || time_std < 0) {
            return EventResult::invalid;
        }
        Event e; e.time = time; e.values = {{east, north}};
        e.variance = {{variance_east, variance_north}}; e.time_variance = time_std * time_std;
        if (!initialized_) {
            auto after = std::upper_bound(pending_inputs_.begin(), pending_inputs_.end(), time,
                [](double t, const Event &input) { return t < input.time; });
            if (after == pending_inputs_.begin()) return EventResult::no_input;
            base_ = Estimate{};
            base_.time = time;
            base_.accel = std::prev(after)->values;
            for (int i = 0; i < 2; ++i) {
                base_.axis[i].x << e.values[i], 0, 0;
                base_.axis[i].P = Eigen::Vector3d(e.variance[i],
                    p_.initial_rate_std * p_.initial_rate_std,
                    p_.initial_disturbance_std * p_.initial_disturbance_std).asDiagonal();
                base_.last_vision[i] = time;
            }
            // First received usable frame anchors initialization; earlier frames are rejected.
            events_.assign(after, pending_inputs_.end());
            initialized_ = true;
            replay(0);
            pending_inputs_.clear();
            prune();
            return EventResult::accepted;
        }
        return insert(e);
    }

    Estimate estimate_at(double time) const {
        if (!initialized_) throw std::logic_error("EKF not initialized");
        Estimate s = events_.empty() ? base_ : events_.back().after;
        if (!std::isfinite(time) || time < s.time) throw std::invalid_argument("Output time precedes history");
        propagate(s, time);
        return s;  // output-only prediction is never committed to event history
    }

private:
    struct Event {
        double time = 0;
        bool input = false;
        std::array<double, 2> values{{0, 0}};
        std::array<double, 2> variance{{0, 0}};
        double time_variance = 0;
        Estimate after;
    };
    static bool before(const Event &a, const Event &b) {
        if (a.time != b.time) return a.time < b.time;
        return a.input < b.input;  // at same time: vision, then input switch
    }
    void propagate(Estimate &s, double time) const {
        for (int i = 0; i < 2; ++i) predict_axis(s.axis[i], s.accel[i], time - s.time, p_);
        s.time = time;
    }
    void replay(std::size_t first) {
        Estimate s = first == 0 ? base_ : events_[first - 1].after;
        for (std::size_t j = first; j < events_.size(); ++j) {
            auto &event = events_[j];
            propagate(s, event.time);
            if (event.input) {
                s.accel = event.values;
            } else {
                for (int i = 0; i < 2; ++i) {
                    const double omega = s.axis[i].x(1);
                    const double R = event.variance[i] + omega * omega * event.time_variance;
                    if (correct_axis(s.axis[i], event.values[i], R, p_.nis_threshold, s.nis[i])) {
                        s.last_vision[i] = event.time;
                    }
                }
            }
            event.after = s;
        }
    }
    EventResult insert(const Event &e) {
        if (e.time <= base_.time) return EventResult::too_old;
        auto it = std::lower_bound(events_.begin(), events_.end(), e, before);
        if (it != events_.end() && it->time == e.time && it->input == e.input) {
            return EventResult::duplicate;
        }
        const auto index = static_cast<std::size_t>(it - events_.begin());
        // Transactional insertion: a rejected numerical replay cannot corrupt checkpoints.
        const auto saved = events_;
        events_.insert(it, e);
        try { replay(index); } catch (...) { events_ = saved; throw; }
        prune();
        return EventResult::accepted;
    }
    void prune() {
        while (!events_.empty() && (events_.size() > p_.max_events ||
               events_.back().time - events_.front().time > p_.history_sec)) {
            base_ = events_.front().after;
            events_.erase(events_.begin());
        }
    }
    EKFParameters p_;
    bool initialized_ = false;
    Estimate base_;
    std::vector<Event> events_, pending_inputs_;
};

// Nonlinear anti-swing inversion; invalid/out-of-domain states must be rejected by caller.
inline double antiswing_acceleration(double theta, double omega, double f,
                                     double l, double g, double kp, double kd) {
    return (l * kp * theta + l * kd * omega - g * std::sin(theta) + l * f) / std::cos(theta);
}

}  // namespace slung_ctrl
