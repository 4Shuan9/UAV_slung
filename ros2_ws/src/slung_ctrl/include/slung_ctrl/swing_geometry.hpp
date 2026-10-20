#pragma once
// Geometry relative to the Gazebo vehicle MODEL origin (the suspension point).
// base_link is 0.24 m above it; camera_link is at (0,0,0.10), pitch 1.5707 rad.
// Gazebo camera axes: +X forward, +Y left, +Z up.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace slung_ctrl {
inline constexpr double nominal_rope_length = 1.0;
inline Eigen::Vector3d suspension_offset_body() { return {0.0, 0.0, 0.0}; }
inline Eigen::Vector3d camera_offset_body() { return {0.0, 0.0, 0.10}; }
inline Eigen::Matrix3d camera_to_body() {
    return Eigen::AngleAxisd(1.5707, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

// Recover suspension-to-payload displacement, not camera-to-payload direction.
// A taut, straight rope of known length supplies the otherwise missing depth.
inline bool payload_vector_body(double u, double v, double cx, double cy,
                                double fx, double fy, double length,
                                Eigen::Vector3d &payload) {
    if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(cx) ||
        !std::isfinite(cy) || !std::isfinite(fx) || !std::isfinite(fy) ||
        !std::isfinite(length) || fx <= 0 || fy <= 0 || length <= 0) return false;
    const Eigen::Vector3d offset = camera_offset_body() - suspension_offset_body();
    // Camera must be inside the constraint sphere: exactly one forward root.
    if (offset.squaredNorm() >= length * length) return false;
    // OpenCV reports pixel-index coordinates. Gazebo's camera K uses pixel
    // centers at index + 0.5; convert once here, keeping K and focal length intact.
    const Eigen::Vector3d camera_ray(1.0, -(u + 0.5 - cx) / fx, -(v + 0.5 - cy) / fy);
    const Eigen::Vector3d ray = (camera_to_body() * camera_ray).normalized();
    if (!ray.allFinite() || ray.squaredNorm() < 0.5) return false;
    const double projection = offset.dot(ray);
    const double discriminant = projection * projection + length * length - offset.squaredNorm();
    if (!std::isfinite(discriminant) || discriminant < 0) return false;
    const double distance = -projection + std::sqrt(discriminant);
    if (!std::isfinite(distance) || distance <= 0) return false;
    payload = offset + distance * ray;
    return payload.allFinite();
}
} // namespace slung_ctrl
