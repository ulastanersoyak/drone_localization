#ifndef POLE_ESTIMATOR_HPP
#define POLE_ESTIMATOR_HPP

#include "kalman/kalman_filter.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

using vec3 = Eigen::Vector3d;

// The pole's pan-tilt camera: a pinhole. Its pose maps camera coordinates
// (x forward, y left, z up) to the local enu frame; on the pan-tilt mount it
// changes with every frame (the simulation takes it from AirSim through tf2).
// The UWB anchor sits at the camera's position.
struct pole_camera
{
  // Where the camera looks: turned by yaw about up (0 = east,
  // counterclockwise), then by pitch about its left axis (positive tilts the
  // view down).
  static Eigen::Isometry3d
  oriented (const vec3 &position, double yaw_rad, double pitch_rad)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity ();
    pose.translate (position);
    pose.rotate (Eigen::AngleAxisd (yaw_rad, vec3::UnitZ ())
                 * Eigen::AngleAxisd (pitch_rad, vec3::UnitY ()));
    return pose;
  }

  // The rest pose of the simulated pole (localization_core airsim/settings/
  // pole_*.json): 24 m east, 3 m up, looking west and 10 degrees up.
  Eigen::Isometry3d pose{ oriented ({ 24.0, 0.0, 3.0 }, 3.14159265, -0.1745) };
  double focal_px{ 960.0 }; // (width / 2) / tan (hfov / 2)
  double center_u_px{ 960.0 };
  double center_v_px{ 540.0 };

  [[nodiscard]] vec3
  position () const
  {
    return pose.translation ();
  }

  // Unit vector in the world towards a pixel (u right, v down).
  [[nodiscard]] vec3
  ray (double u, double v) const
  {
    return (pose.linear ()
            * vec3{ 1.0, -(u - center_u_px) / focal_px, -(v - center_v_px) / focal_px })
        .normalized ();
  }

  // The pixel a world point lands on, the inverse of ray (). Empty when the
  // point is behind the camera; it can still be outside the image.
  [[nodiscard]] std::optional<std::array<double, 2>>
  pixel (const vec3 &point) const
  {
    const vec3 c = pose.inverse () * point;
    if (c.x () <= 1e-9)
      {
        return std::nullopt;
      }
    return std::array<double, 2>{ center_u_px - (focal_px * c.y () / c.x ()),
                                  center_v_px - (focal_px * c.z () / c.x ()) };
  }

  [[nodiscard]] bool
  sees (const vec3 &point) const
  {
    const auto p = pixel (point);
    return p && (*p)[0] >= 0 && (*p)[0] <= 2 * center_u_px && (*p)[1] >= 0
           && (*p)[1] <= 2 * center_v_px;
  }
};

struct pole_estimator_config
{
  double imu_accel_sigma_mps2{ 0.3 };
  double maneuver_accel_sigma_mps2{ 2.0 };
  double range_sigma_m{ 0.1 };
  double bearing_sigma_rad{ 4.0 / 960.0 }; // about 4 pixels
  double initial_velocity_sigma_mps{ 5.0 };
  // Chi-square gates at 99.9 % for 1, 2 and 3 dof.
  double gate_1d{ 10.83 };
  double gate_2d{ 13.82 };
  double gate_3d{ 16.27 };
};

// Fuses IMU acceleration, UWB range to the pole, camera bearing from the pole
// and GPS in a local ENU frame. State: [x, y, z, v_x, v_y, v_z]. Any subset of
// the sensors works. Range and bearing are nonlinear in the position, so they
// are linearized around the current estimate.
class pole_estimator
{
  pole_estimator_config config_;
  vec3 anchor_;
  using vector6 = kalman_filter<6>::vector;
  using matrix6 = kalman_filter<6>::matrix;

  kalman_filter<6> filter_{};
  bool initialized_{ false };
  double t_{ 0 };
  std::optional<vec3> accel_{};

  void
  propagate (double dt)
  {
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity ();
    matrix6 F = matrix6::Identity ();
    F.topRightCorner<3, 3> () = I * dt;
    const double sigma = accel_ ? config_.imu_accel_sigma_mps2
                                : config_.maneuver_accel_sigma_mps2;
    // Continuous white noise acceleration.
    const double q = sigma * sigma;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    matrix6 Q;
    Q << I * (q * dt3 / 3), I * (q * dt2 / 2), I * (q * dt2 / 2), I * (q * dt);
    vector6 u = vector6::Zero ();
    if (accel_)
      {
        u << *accel_ * (dt2 / 2), *accel_ * dt;
      }
    filter_.predict (F, Q, u);
  }

public:
  pole_estimator (const pole_estimator_config &config, const vec3 &anchor)
      : config_ (config), anchor_ (anchor)
  {
  }

  [[nodiscard]] bool
  initialized () const noexcept
  {
    return initialized_;
  }

  [[nodiscard]] vec3
  position () const
  {
    return filter_.state ().head<3> ();
  }

  [[nodiscard]] vec3
  velocity () const
  {
    return filter_.state ().tail<3> ();
  }

  // The time the state is valid for.
  [[nodiscard]] double
  time () const noexcept
  {
    return t_;
  }

  [[nodiscard]] double
  sigma_position (std::size_t axis) const
  {
    const auto i = static_cast<Eigen::Index> (axis);
    return std::sqrt (filter_.covariance () (i, i));
  }

  void
  initialize (double t, const vec3 &position, double sigma_m)
  {
    vector6 x0;
    x0 << position, vec3::Zero ();
    vector6 variances;
    variances << vec3::Constant (sigma_m * sigma_m),
        vec3::Constant (config_.initial_velocity_sigma_mps
                        * config_.initial_velocity_sigma_mps);
    filter_ = kalman_filter<6>{ x0, variances.asDiagonal () };
    t_ = t;
    initialized_ = true;
  }

  // Forgets the state; the next measurement that can, initializes it again.
  void
  reset () noexcept
  {
    initialized_ = false;
  }

  // False once the state or its uncertainty is no longer a finite number.
  [[nodiscard]] bool
  healthy () const
  {
    const auto &P = filter_.covariance ();
    return filter_.state ().allFinite () && P.diagonal ().allFinite ()
           && (P.diagonal ().array () >= 0).all ();
  }

  // Predicts forward to time t. Out of order times are ignored.
  void
  advance_to (double t)
  {
    if (initialized_ && t > t_)
      {
        propagate (t - t_);
        t_ = t;
      }
  }

  // Acceleration in the local frame, used from now until the next sample.
  void
  set_acceleration (const vec3 &accel)
  {
    accel_ = accel;
  }

  // The update functions return false if the measurement was gated out.
  bool
  add_range (double range_m)
  {
    const vec3 to_tag = position () - anchor_;
    const double predicted = to_tag.norm ();
    if (!std::isfinite (range_m) || predicted < 1e-3)
      {
        return false;
      }
    const vec3 u = to_tag / predicted;
    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero ();
    H.head<3> () = u.transpose ();
    const Eigen::Matrix<double, 1, 1> z{ range_m + u.dot (anchor_) };
    const Eigen::Matrix<double, 1, 1> R{ config_.range_sigma_m * config_.range_sigma_m };
    if (!(filter_.nis (z, H, R) <= config_.gate_1d))
      {
        return false;
      }
    filter_.update (z, H, R);
    return true;
  }

  bool
  add_bearing (const vec3 &ray)
  {
    // Measures the sideways offset of the state from the ray, in the plane
    // across it. Sideways uncertainty grows with distance.
    if (!ray.allFinite ())
      {
        return false;
      }
    // Any axis across the ray works; up fails for a ray close to vertical.
    const vec3 across = std::abs (ray[2]) < 0.9 ? vec3::UnitZ () : vec3::UnitX ();
    const vec3 t1 = ray.cross (across).normalized ();
    const vec3 t2 = ray.cross (t1);
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero ();
    H.block<1, 3> (0, 0) = t1.transpose ();
    H.block<1, 3> (1, 0) = t2.transpose ();
    const Eigen::Vector2d z{ t1.dot (anchor_), t2.dot (anchor_) };
    const double sigma
        = config_.bearing_sigma_rad * std::max ((position () - anchor_).norm (), 0.5);
    const Eigen::Matrix2d R = Eigen::Vector2d::Constant (sigma * sigma).asDiagonal ();
    if (!(filter_.nis (z, H, R) <= config_.gate_2d))
      {
        return false;
      }
    filter_.update (z, H, R);
    return true;
  }

  bool
  add_gps (const vec3 &position, double sigma_m, double sigma_up_m)
  {
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero ();
    H.leftCols<3> () = Eigen::Matrix3d::Identity ();
    if (!position.allFinite ())
      {
        return false;
      }
    const vec3 &z = position;
    const Eigen::Matrix3d R
        = vec3 (sigma_m * sigma_m, sigma_m * sigma_m, sigma_up_m * sigma_up_m)
              .asDiagonal ();
    if (!(filter_.nis (z, H, R) <= config_.gate_3d))
      {
        return false;
      }
    filter_.update (z, H, R);
    return true;
  }

  // Horizontal velocity, what a GPS receiver reports.
  bool
  add_gps_velocity (double v_east, double v_north, double sigma_mps)
  {
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero ();
    H.block<2, 2> (0, 3) = Eigen::Matrix2d::Identity ();
    const Eigen::Vector2d z{ v_east, v_north };
    const Eigen::Matrix2d R
        = Eigen::Vector2d::Constant (sigma_mps * sigma_mps).asDiagonal ();
    if (!(filter_.nis (z, H, R) <= config_.gate_2d))
      {
        return false;
      }
    filter_.update (z, H, R);
    return true;
  }
};

#endif // !POLE_ESTIMATOR_HPP
