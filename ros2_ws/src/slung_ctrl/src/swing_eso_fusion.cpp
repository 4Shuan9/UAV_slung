// === [ ESO Fusion: historical LESO + timestamped input-history predictor ] ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "slung_ctrl/msg/swing_adrc_state.hpp"
#include "slung_ctrl/eso_fusion.hpp"
#include <deque>
#include <memory>
#include <string>

class SwingAngleESOFusion : public rclcpp::Node {
public:
    SwingAngleESOFusion() : Node("swing_eso_fusion") {
        parameters_.g=declare_parameter("g",9.81);
        parameters_.l=declare_parameter("l",1.0);
        parameters_.omega=declare_parameter("omega_o",8.0);
        parameters_.max_step=declare_parameter("dt",.005);
        parameters_.history_sec=declare_parameter("history_sec",2.0);
        parameters_.max_input_gap=declare_parameter("max_input_gap",.05);
        parameters_.max_visual_gap=declare_parameter("max_visual_gap",.5);
        const auto max_inputs=declare_parameter("max_inputs",2048);
        if (max_inputs<4) throw std::invalid_argument("max_inputs must be >=4");
        parameters_.max_inputs=static_cast<std::size_t>(max_inputs);
        parameters_.validate();
        filter_=std::make_unique<slung_ctrl::DelayedSwingESO>(parameters_);
        filter_alpha_=declare_parameter("filter_alpha",.5);
        max_vision_age_=declare_parameter("max_vision_age",.25);
        max_delay_=declare_parameter("max_visual_delay",.5);
        max_input_age_=declare_parameter("max_input_age",.05);
        future_tolerance_=declare_parameter("input_future_tolerance",.02);
        max_angle_=declare_parameter("max_angle_deg",35.0)*M_PI/180;
        input_delay_=declare_parameter("input_delay",0.0);
        input_time_mode_=declare_parameter("input_time_mode",std::string("synchronized"));
        const double hz=declare_parameter("publish_hz",100.0);
        const auto topic=declare_parameter("vision_topic",
            std::string("/uav/swing/vision/angle"));
        for (double v:{filter_alpha_,max_vision_age_,max_delay_,max_input_age_,future_tolerance_,max_angle_,input_delay_,hz})
            if (!std::isfinite(v)) throw std::invalid_argument("Non-finite ESO node parameter");
        if (filter_alpha_<0 || filter_alpha_>1 || max_vision_age_<=0 || max_delay_<=0 ||
            max_delay_>=parameters_.history_sec || max_input_age_<=0 || future_tolerance_<0 ||
            future_tolerance_>max_input_age_ || max_angle_<=0 || max_angle_>=M_PI/2 || input_delay_<0 ||
            hz<=0 || hz>1000 || (input_time_mode_!="synchronized" && input_time_mode_!="offset"))
            throw std::invalid_argument("Invalid ESO timing/domain parameters");

        vision_subscriber_=create_subscription<geometry_msgs::msg::Vector3Stamped>(topic,20,
            std::bind(&SwingAngleESOFusion::vision_callback,this,std::placeholders::_1));
        imu_subscriber_=create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",rclcpp::SensorDataQoS(),
            std::bind(&SwingAngleESOFusion::imu_callback,this,std::placeholders::_1));
        adrc_state_publisher_=create_publisher<slung_ctrl::msg::SwingADRCState>("/uav/swing/eso_fusion/state",10);
        fusion_angle_publisher_=create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/swing/eso_fusion/angle",10);
        fusion_rate_publisher_=create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/swing/eso_fusion/rate",10);
        fusion_dist_publisher_=create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/swing/eso_fusion/disturbance",10);
        diagnostics_publisher_=create_publisher<std_msgs::msg::Float64MultiArray>("/uav/swing/eso_fusion/diagnostics",10);
        timer_=create_wall_timer(std::chrono::duration<double>(1/hz),std::bind(&SwingAngleESOFusion::publish_estimate,this));
        param_subscriber_=add_on_set_parameters_callback(
            std::bind(&SwingAngleESOFusion::parameters_callback,this,std::placeholders::_1));
        RCLCPP_INFO(get_logger(),"ESO ready: historical LESO -> input-history prediction; clock=%s, output LPF alpha=%.2f",
            input_time_mode_.c_str(),filter_alpha_);
    }

private:
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        try {
            auto next=parameters_;double alpha=filter_alpha_;bool model_changed=false;
            for (const auto &param:params) {
                const auto &name=param.get_name();
                if (name=="filter_alpha") alpha=param.as_double();
                else if (name=="g") {next.g=param.as_double();model_changed=true;}
                else if (name=="l") {next.l=param.as_double();model_changed=true;}
                else if (name=="omega_o") {next.omega=param.as_double();model_changed=true;}
                else if (name=="dt") {next.max_step=param.as_double();model_changed=true;}
                else throw std::invalid_argument("Timing/topic parameters require restart; tau is replaced by acquisition timestamps");
            }
            next.validate();
            if (!std::isfinite(alpha) || alpha<0 || alpha>1) throw std::invalid_argument("Invalid filter_alpha");
            // Validate the entire transaction before changing any internal state.
            if (model_changed) {
                filter_=std::make_unique<slung_ctrl::DelayedSwingESO>(next);
                parameters_=next;pending_.clear();first_filter_run_=true;
                RCLCPP_WARN(get_logger(),"ESO model/gains changed; historical observer reset, waiting for input + vision");
            }
            if (alpha!=filter_alpha_) first_filter_run_=true;
            filter_alpha_=alpha;result.successful=true;
        } catch (const std::exception &e) {result.successful=false;result.reason=e.what();}
        return result;
    }
    void reset(const char *reason) {
        filter_->reset();pending_.clear();first_filter_run_=true;offset_set_=false;
        last_px4_stamp_=0;last_input_=last_published_=-1;
        RCLCPP_WARN(get_logger(),"ESO reset: %s",reason);
    }
    bool clock_ready(double t) {
        if (t<=0) return false;
        if (last_clock_>=0 && t<last_clock_-.001) reset("ROS clock moved backwards");
        last_clock_=t;return true;
    }
    void imu_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        const double t=now().seconds();
        if (!clock_ready(t) || !std::isfinite(msg->ax) || !std::isfinite(msg->ay) || msg->timestamp_sample==0) return;
        if (last_px4_stamp_ && msg->timestamp_sample<last_px4_stamp_) reset("PX4 sample clock restarted");
        last_px4_stamp_=msg->timestamp_sample;
        if (!offset_set_) {
            input_offset_=input_time_mode_=="offset" ? t-msg->timestamp_sample*1e-6-input_delay_ : -input_delay_;
            offset_set_=true;
        }
        const double sample=msg->timestamp_sample*1e-6+input_offset_;
        if (sample>t+future_tolerance_ || t-sample>max_input_age_) {
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
                "ESO acceleration timestamp age %.4fs invalid; check PX4/ROS physical clocks",t-sample);
            return;
        }
        // VehicleLocalPosition acceleration, not raw accelerometer specific force.
        // PX4 NED ax=North, ay=East; observer axis 0=East, 1=North.
        if (filter_->add_input(sample,msg->ay,msg->ax)) last_input_=sample;
        process_pending(t);
    }
    struct Vision {double capture,east,north;};
    void vision_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        const double t=now().seconds();
        if (!clock_ready(t)) return;
        if (msg->header.stamp.sec<0 || msg->header.stamp.nanosec>=1000000000u) {
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"ESO visual frame rejected: malformed capture timestamp");return;
        }
        const double capture=rclcpp::Time(msg->header.stamp,get_clock()->get_clock_type()).seconds();
        const double east=msg->vector.x*M_PI/180,north=msg->vector.y*M_PI/180;
        if (!std::isfinite(east) || !std::isfinite(north) || capture<=0 || capture>t ||
            t-capture>max_delay_ || std::abs(east)>max_angle_ || std::abs(north)>max_angle_) {
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"ESO visual frame rejected: time/domain invalid");return;
        }
        // Ordered camera stream: duplicated/out-of-order captures are not fused.
        // This observer intentionally does not claim EKF-style OOS rollback.
        if (capture<=filter_->anchor_time()+1e-10 ||
            (!pending_.empty() && capture<=pending_.back().capture+1e-10)) {++rejected_frames_;return;}
        pending_.push_back({capture,east,north});
        if (pending_.size()>128) {pending_.pop_front();++rejected_frames_;}
        last_delay_=t-capture;process_pending(t);
    }
    void process_pending(double t) {
        while (!pending_.empty()) {
            const auto v=pending_.front();
            if (t-v.capture>max_delay_) {pending_.pop_front();++rejected_frames_;continue;}
            try {
                const auto result=filter_->add_vision(v.capture,v.east,v.north);
                if (result==slung_ctrl::ESOResult::no_input) {
                    // A very first frame can predate our entire input buffer.
                    // Drop it once it is clearly older than the retained horizon.
                    if (last_input_>=0 && last_input_-v.capture>parameters_.history_sec) {
                        pending_.pop_front();++rejected_frames_;continue;
                    }
                    return;
                }
                pending_.pop_front();
                if (result==slung_ctrl::ESOResult::initialized || result==slung_ctrl::ESOResult::reinitialized)
                    first_filter_run_=true;
                if (result==slung_ctrl::ESOResult::accepted || result==slung_ctrl::ESOResult::initialized ||
                    result==slung_ctrl::ESOResult::reinitialized) ++accepted_frames_;
                else ++rejected_frames_;
            } catch (const std::exception &e) {reset(e.what());return;}
        }
    }
    void publish_estimate() {
        const auto stamp=now();const double t=stamp.seconds();
        if (!clock_ready(t)) return;
        process_pending(t);
        if (!filter_->initialized() || t<=last_published_ || last_input_<0 || t<filter_->anchor_time()) return;
        try {
            const double input_age=t-filter_->input_time_at(t),visual_age=t-filter_->anchor_time();
            std_msgs::msg::Float64MultiArray diag;
            diag.data={visual_age,input_age,last_delay_,static_cast<double>(filter_->input_count()),
                static_cast<double>(accepted_frames_),static_cast<double>(rejected_frames_)};
            diagnostics_publisher_->publish(diag);
            if (input_age>max_input_age_ || visual_age>max_vision_age_) {first_filter_run_=true;return;}
            const auto prediction=filter_->estimate_at(t);
            for (const auto &axis:prediction.axis) if (std::abs(axis(0))>max_angle_) {first_filter_run_=true;return;}
            if (first_filter_run_) {filtered_=prediction.axis;first_filter_run_=false;}
            else for (int i=0;i<2;++i) filtered_[i]=filter_alpha_*prediction.axis[i]+(1-filter_alpha_)*filtered_[i];
            slung_ctrl::msg::SwingADRCState state;
            state.header.stamp=stamp;state.header.frame_id="world";
            state.theta_east_deg=filtered_[0](0)*180/M_PI;state.theta_north_deg=filtered_[1](0)*180/M_PI;
            state.rate_east_deg_s=filtered_[0](1)*180/M_PI;state.rate_north_deg_s=filtered_[1](1)*180/M_PI;
            state.disturbance_east_rad_s2=filtered_[0](2);state.disturbance_north_rad_s2=filtered_[1](2);
            adrc_state_publisher_->publish(state);
            geometry_msgs::msg::Vector3Stamped angle,rate,dist;
            angle.header=rate.header=dist.header=state.header;
            angle.vector.x=state.theta_east_deg;angle.vector.y=state.theta_north_deg;
            rate.vector.x=state.rate_east_deg_s;rate.vector.y=state.rate_north_deg_s;
            dist.vector.x=state.disturbance_east_rad_s2;dist.vector.y=state.disturbance_north_rad_s2;
            fusion_angle_publisher_->publish(angle);fusion_rate_publisher_->publish(rate);fusion_dist_publisher_->publish(dist);
            last_published_=t;
        } catch (const std::exception &e) {reset(e.what());}
    }

    slung_ctrl::ESOParameters parameters_;
    std::unique_ptr<slung_ctrl::DelayedSwingESO> filter_;
    std::deque<Vision> pending_;
    std::array<Eigen::Vector3d,2> filtered_;
    double filter_alpha_,max_vision_age_,max_delay_,max_input_age_,future_tolerance_,max_angle_,input_delay_;
    double last_clock_=-1,last_input_=-1,last_published_=-1,last_delay_=0,input_offset_=0;
    uint64_t last_px4_stamp_=0,accepted_frames_=0,rejected_frames_=0;
    bool first_filter_run_=true,offset_set_=false;
    std::string input_time_mode_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr vision_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<slung_ctrl::msg::SwingADRCState>::SharedPtr adrc_state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr fusion_angle_publisher_,fusion_rate_publisher_,fusion_dist_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_subscriber_;
};

int main(int argc,char *argv[]) {
    rclcpp::init(argc,argv);
    try {rclcpp::spin(std::make_shared<SwingAngleESOFusion>());}
    catch (const std::exception &e) {RCLCPP_FATAL(rclcpp::get_logger("swing_eso_fusion"),"%s",e.what());rclcpp::shutdown();return 1;}
    rclcpp::shutdown();return 0;
}
