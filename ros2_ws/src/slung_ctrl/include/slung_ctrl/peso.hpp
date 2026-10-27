#pragma once
// Codex-generated: basic output-predictor PESO, owner-approved 2026-10-27.
// Same frozen LESO dynamics/gains as eso_delay. p_dot=omega_hat between frames;
// p(capture+)=visual(capture), while the three observer states remain continuous.
// All states use rad, rad/s, rad/s^2. p is an internal prediction, not a measurement.
#include "slung_ctrl/eso_delay.hpp"

namespace slung_ctrl {
struct PESOEstimate {
    double time = 0;
    // Per axis: [theta_hat, omega_hat, f_hat, predicted_output].
    std::array<Eigen::Vector4d,2> axis{Eigen::Vector4d::Zero(),Eigen::Vector4d::Zero()};
};

class DelayedSwingPESO {
public:
    explicit DelayedSwingPESO(ESOParameters p = {}) : p_(p) { p_.validate(); }
    void reset() { inputs_.clear(); initialized_=false; }
    bool initialized() const { return initialized_; }
    double anchor_time() const { return initialized_ ? anchor_.time : -1; }
    std::size_t input_count() const { return inputs_.size(); }
    const PESOEstimate &anchor() const { return anchor_; }

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
            for (int i=0;i<2;++i) anchor_.axis[i]=Eigen::Vector4d(y[i],0,0,y[i]);
            initialized_=true;
            return was_initialized ? ESOResult::reinitialized : ESOResult::initialized;
        }
        // Advance to this capture with the previous predictor state, then reset
        // ONLY the predictor to the newly received real visual observation.
        auto next=anchor_;
        integrate(next,t);
        for (int i=0;i<2;++i) next.axis[i](3)=y[i];
        anchor_=next;
        return ESOResult::accepted;
    }

    double input_time_at(double t) const {
        return inputs_[input_index(t)].t;
    }
    PESOEstimate estimate_at(double t) const {
        if (!initialized_ || !std::isfinite(t) || t<anchor_.time)
            throw std::runtime_error("PESO prediction has no valid historical anchor");
        auto predicted=anchor_;
        // Replay observer and predictor jointly from the unchanged capture anchor.
        // Publishing does not commit prediction or fuse a visual frame twice.
        integrate(predicted,t);
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
    Eigen::Vector4d derivative(const Eigen::Vector4d &x,double accel) const {
        Eigen::Vector4d dx;
        dx.head<3>()=eso_correction(x.head<3>(),accel,x(3),p_);
        dx(3)=x(1);
        return dx;
    }
    void integrate(PESOEstimate &state,double target) const {
        auto idx=input_index(state.time);
        while (state.time<target-1e-12) {
            const auto &input=inputs_[idx];
            const double end=idx+1<inputs_.size() ? std::min(target,inputs_[idx+1].t) : target;
            if (end-input.t>p_.max_input_gap+1e-9)
                throw std::runtime_error("PESO historical acceleration gap exceeds limit");
            while (state.time<end-1e-12) {
                const double h=std::min(p_.max_step,end-state.time);
                for (int axis=0;axis<2;++axis) {
                    auto &x=state.axis[axis];
                    const Eigen::Vector4d k1=derivative(x,input.a[axis]);
                    const Eigen::Vector4d k2=derivative(x+.5*h*k1,input.a[axis]);
                    const Eigen::Vector4d k3=derivative(x+.5*h*k2,input.a[axis]);
                    const Eigen::Vector4d k4=derivative(x+h*k3,input.a[axis]);
                    x+=h/6*(k1+2*k2+2*k3+k4);
                    if (!x.allFinite()) throw std::runtime_error("Non-finite PESO state");
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
    PESOEstimate anchor_;
    bool initialized_=false;
};
} // namespace slung_ctrl
