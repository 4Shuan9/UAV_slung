#pragma once
// Historical visual ZOH LESO, then newest-visual ZOH replay to the current time.
// The disturbance is the LINEAR-model residual, in rad/s^2.
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

namespace slung_ctrl {
struct ESOParameters {
    double g = 9.81, l = 1.0, omega = 8.0, max_step = .005;
    double history_sec = 2.0, max_input_gap = .05, max_visual_gap = .5;
    std::size_t max_inputs = 2048;
    void validate() const {
        for (double v : {g,l,omega,max_step,history_sec,max_input_gap,max_visual_gap})
            if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid ESO parameter");
        if (l <= .01 || max_step*std::max(omega,std::sqrt(g/l)) > .25 ||
            history_sec <= max_visual_gap || max_inputs < 4)
            throw std::invalid_argument("Invalid ESO integration/history configuration");
    }
};
struct ESOEstimate {
    double time = 0;
    std::array<Eigen::Vector3d,2> axis{Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()};
};
enum class ESOResult { accepted, initialized, reinitialized, no_input, old_or_duplicate, invalid };

inline Eigen::Vector3d eso_drift(const Eigen::Vector3d &x, double accel, const ESOParameters &p) {
    return {x(1), -p.g/p.l*x(0)-accel/p.l+x(2), 0};
}
inline Eigen::Vector3d eso_correction(const Eigen::Vector3d &x, double accel, double y,
                                     const ESOParameters &p) {
    const double w = p.omega;
    return eso_drift(x,accel,p) + Eigen::Vector3d(3*w,3*w*w-p.g/p.l,w*w*w)*(y-x(0));
}

class DelayedSwingESO {
public:
    explicit DelayedSwingESO(ESOParameters p = {}) : p_(p) { p_.validate(); }
    void reset() { inputs_.clear(); initialized_=false; }
    bool initialized() const { return initialized_; }
    double anchor_time() const { return initialized_ ? anchor_.time : -1; }
    std::size_t input_count() const { return inputs_.size(); }
    const ESOEstimate &anchor() const { return anchor_; }

    bool add_input(double t, double east, double north) {
        if (!std::isfinite(t) || !std::isfinite(east) || !std::isfinite(north) ||
            (!inputs_.empty() && t <= inputs_.back().t)) return false;
        inputs_.push_back({t,{east,north}});
        // Never retain an observer whose required input history has been evicted.
        const double cutoff=t-p_.history_sec;
        while (inputs_.size()>1 && inputs_[1].t<cutoff) inputs_.pop_front();
        while (inputs_.size()>p_.max_inputs) inputs_.pop_front();
        if (initialized_ && (anchor_.time<inputs_.front().t || t-anchor_.time>p_.history_sec))
            initialized_=false;
        return true;
    }

    ESOResult add_vision(double t, double east, double north) {
        if (!std::isfinite(t) || !std::isfinite(east) || !std::isfinite(north)) return ESOResult::invalid;
        if (initialized_ && t<=anchor_.time+1e-10) return ESOResult::old_or_duplicate;
        // Wait for acceleration coverage at both ends of this past interval.
        if (inputs_.empty() || t<inputs_.front().t || t>inputs_.back().t) return ESOResult::no_input;
        const std::array<double,2> y{east,north};
        if (!initialized_ || t-anchor_.time>p_.max_visual_gap) {
            const bool was_initialized=initialized_;
            anchor_.time=t;
            for (int i=0;i<2;++i) anchor_.axis[i]=Eigen::Vector3d(y[i],0,0);
            measured_=y;initialized_=true;
            return was_initialized ? ESOResult::reinitialized : ESOResult::initialized;
        }
        // Reconstruct the past visual signal with left-endpoint ZOH:
        // y(s)=y_previous for s in [previous capture,current capture).
        // The newly received value applies from its capture time onward;
        // it must not influence the preceding interval or cause a state jump.
        auto next=anchor_;
        const auto held_measurement=measured_;
        integrate(next,t,[&](const Eigen::Vector3d &x,int axis,double accel,double) {
            return eso_correction(x,accel,held_measurement[axis],p_);
        });
        anchor_=next;measured_=y;
        return ESOResult::accepted;
    }

    double input_time_at(double t) const {
        return inputs_[input_index(t)].t;
    }
    ESOEstimate estimate_at(double t) const {
        if (!initialized_ || !std::isfinite(t) || t<anchor_.time)
            throw std::runtime_error("ESO prediction has no valid historical anchor");
        auto predicted=anchor_;
        // Codex-modified, owner-approved: hold the newest visual from its capture
        // time and replay LESO correction with timestamped acceleration to now.
        // No additional prediction horizon; the historical anchor is unchanged.
        integrate(predicted,t,[&](const Eigen::Vector3d &x,int axis,double accel,double) {
            return eso_correction(x,accel,measured_[axis],p_);
        });
        return predicted;  // Publication never advances the historical LESO.
    }

private:
    struct Input { double t; std::array<double,2> a; };
    std::size_t input_index(double t) const {
        auto it=std::upper_bound(inputs_.begin(),inputs_.end(),t,
            [](double value,const Input &input){return value<input.t;});
        if (it==inputs_.begin()) throw std::runtime_error("ESO missing preceding acceleration");
        return static_cast<std::size_t>(it-inputs_.begin()-1);
    }
    template<class Derivative>
    void integrate(ESOEstimate &state,double target,Derivative derivative) const {
        auto idx=input_index(state.time);
        while (state.time<target-1e-12) {
            const auto &input=inputs_[idx];
            const double end=idx+1<inputs_.size() ? std::min(target,inputs_[idx+1].t) : target;
            if (end-input.t>p_.max_input_gap+1e-9)
                throw std::runtime_error("ESO historical acceleration gap exceeds limit");
            while (state.time<end-1e-12) {
                const double s=state.time,h=std::min(p_.max_step,end-s);
                for (int axis=0;axis<2;++axis) {
                    auto &x=state.axis[axis];
                    const Eigen::Vector3d k1=derivative(x,axis,input.a[axis],s);
                    const Eigen::Vector3d k2=derivative(x+.5*h*k1,axis,input.a[axis],s+.5*h);
                    const Eigen::Vector3d k3=derivative(x+.5*h*k2,axis,input.a[axis],s+.5*h);
                    const Eigen::Vector3d k4=derivative(x+h*k3,axis,input.a[axis],s+h);
                    x+=h/6*(k1+2*k2+2*k3+k4);
                    if (!x.allFinite()) throw std::runtime_error("Non-finite ESO state");
                }
                state.time+=h;
            }
            state.time=end;
            if (idx+1<inputs_.size() && inputs_[idx+1].t<=state.time+1e-12) ++idx;
        }
        state.time=target;
    }
    ESOParameters p_;
    std::deque<Input> inputs_;
    ESOEstimate anchor_;
    std::array<double,2> measured_{};
    bool initialized_=false;
};
} // namespace slung_ctrl
