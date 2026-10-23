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
// Inverse perspective geometry for the center of a spherical silhouette.
// The enclosing circle of the ideal projected ellipse shares its center.
// With camera-frame sphere center (x,y,z), its normalized silhouette center
// is (x*y/(x*x-r*r), x*z/(x*x-r*r)), rather than (y/x,z/x).
// Jointly recover optical depth using the same taut-rope constraint. No
// estimated angle offsets, temporal smoothing, or truth feedback are used.
inline bool payload_sphere_vector_body(double u, double v, double cx, double cy,
                                       double fx, double fy, double length,
                                       double radius, Eigen::Vector3d &payload) {
    if (!std::isfinite(radius) || radius < 0 ||
        !payload_vector_body(u, v, cx, cy, fx, fy, length, payload)) return false;
    const Eigen::Vector3d offset = camera_offset_body() - suspension_offset_body();
    const Eigen::Matrix3d body_to_camera = camera_to_body().transpose();
    // Fixed-point inversion converges rapidly for this camera and 5 cm sphere.
    // Eight iterations also resolve the full visible image to double precision.
    for (int i = 0; i < 8; ++i) {
        const double depth = (body_to_camera * (payload - offset)).x();
        if (!std::isfinite(depth) || depth <= radius) return false;
        const double scale = 1.0 - radius * radius / (depth * depth);
        const double center_u = cx - 0.5 + (u + 0.5 - cx) * scale;
        const double center_v = cy - 0.5 + (v + 0.5 - cy) * scale;
        if (!payload_vector_body(center_u, center_v, cx, cy, fx, fy, length, payload))
            return false;
    }
    return true;
}
} // namespace slung_ctrl
