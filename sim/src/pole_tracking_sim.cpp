// Tracks the drone from a pole beside its flight path. A ROS 2 node for
// Cosys-AirSim (localization_core airsim/):
//
//   /pole/detections (yolo_msgs/DetectionArray,     -> camera bearing
//     yolo_ros or sim/detector/synthetic_detect.py)
//   simulated UWB anchor on the pole, 20 Hz         -> range
//   /airsim_node/drone/imu/imu, .../gps/gps         -> optional, see no_imu,
//                                                      no_gps
//   /airsim_node/drone/odom_local                   -> the UWB simulation
//                                                      and GPS velocity only
//   /airsim_node/drone/pole_Scene/pose              -> where the pan-tilt
//     (PoseStamped, one per camera image)              camera pointed (into tf2)
//   /airsim_node/drone/pole_Scene/camera_info       -> the camera's intrinsics
//   /airsim_node/gimbal_angle_quat_cmd              <- pan-tilt steering
//     (airsim_interfaces/GimbalAngleQuatCmd)
//   /pole/fused, /pole/ray_uwb, /pole/gps           -> rosbag2, evaluate.py,
//     (geometry_msgs/PoseStamped, frame enu)           rviz2 (sim/rviz/)
//
// Frames come from tf2 (localization_core sim/src/ros_sim.hpp): the truth is
// the TF enu -> drone, and each camera pose AirSim publishes goes into tf2 as
// world -> pole_camera, so the camera is looked up, interpolated, at the
// moment a frame was taken. The pole (camera and UWB anchor) is where AirSim
// says the camera is; nothing about its placement is configured here.
//
// The camera sits on a pan-tilt mount and is turned towards the estimate of
// the drone (never the ground truth), slew rate limited, so it keeps the drone
// in view. Detections arrive late because YOLO takes time. All measurements go
// through a buffer and are fused in the order of their timestamps, a fixed
// time behind the simulation, so each is applied at the moment it was
// measured.
//
// The drone is flown by localization_core's airsim/autopilot.py (a figure
// eight over /cmd_vel; it waits for the first detection too), started by
// the launch file; this program only measures and estimates.
//
// Options are ROS parameters, defined in pole_tracking_sim_parameters.yaml
// (generate_parameter_library; `ros2 param describe /pole_tracking_sim
// <name>`); sim/run_pole.sh turns --name value into name:=value.

#include "pole_estimator.hpp"
#include "ros_sim.hpp"

#include "kalman/measurements.hpp"
#include "utility/queue.hpp"

#include <drone_localization/pole_tracking_sim_parameters.hpp>

#include <airsim_interfaces/msg/gimbal_angle_quat_cmd.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <variant>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void
handle_signal (int /*signal*/)
{
  stop_requested = 1;
}

template <typename... Ts> struct overloaded : Ts...
{
  using Ts::operator()...;
};

constexpr double pi = std::numbers::pi;
constexpr double deg = pi / 180;
// AirSim's gravity (msr::airlib::EarthUtils, SimpleFlight).
constexpr double world_gravity = 9.8;
const std::string camera_frame = "pole_camera";

Eigen::Quaterniond
from_rotation_vector (const vec3 &r)
{
  const double angle = r.norm ();
  return angle < 1e-12 ? Eigen::Quaterniond::Identity ()
                       : Eigen::Quaterniond (Eigen::AngleAxisd (angle, r / angle));
}

// ---- events from the ROS callback thread ----------------------------------

struct imu_event
{
  imu_sample sample;
};

struct navsat_event
{
  double lat, lon, alt;
  vec3 velocity; // enu, from the odometry: a receiver's velocity output
};

struct range_event
{
  double range_m;
};

struct detection_event
{
  double u, v, confidence;
};

using measurement = std::variant<imu_event, navsat_event, range_event, detection_event>;

struct timed_measurement
{
  double t;
  measurement value;
};

struct truth_sample
{
  double t;
  vec3 position; // enu
};

using inbound = std::variant<timed_measurement, truth_sample>;

// ---- the node ---------------------------------------------------------------

using parameters = pole_tracking_sim_parameters::Params;

class pole_tracking_sim
{
  rclcpp::Node::SharedPtr node_;
  parameters opt_;
  ros_sim::frames frames_;
  rclcpp::Publisher<airsim_interfaces::msg::GimbalAngleQuatCmd>::SharedPtr camera_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fused_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ray_uwb_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gps_pub_{};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_{};
  std::thread spin_thread_{};

  // ROS callback thread state, behind ros_mutex_.
  std::mutex ros_mutex_{};
  std::optional<rclcpp::Time> t0_{};
  vec3 velocity_enu_{ vec3::Zero () };
  std::int64_t last_gps_stamp_{ -1 };
  std::optional<pole_camera> camera_{}; // intrinsics and the rest pose

  queue<inbound> inbox_{};
  std::multimap<double, measurement> buffer_{};
  GeographicLib::LocalCartesian frame_;
  std::optional<pole_estimator> estimator_{}; // once the pole's place is known
  vec3 anchor_{ vec3::Zero () };
  std::mt19937 rng_;
  std::normal_distribution<double> noise_{ 0.0, 1.0 };
  vec3 accel_bias_{};
  vec3 orientation_error_{ vec3::Zero () };
  double last_imu_t_{ -1 };

  double command_yaw_{ 0 };
  double command_pitch_{ 0 };
  double last_steer_t_{ -1 };
  std::size_t frames_scored_{ 0 };
  std::size_t frames_in_view_{ 0 };
  double sim_t_{ 0 };
  double last_range_t_{ -1 };
  double last_publish_t_{ -1 };
  double last_print_t_{ 0 };
  std::optional<double> last_error_{};
  // When the flight started (seconds since the first message): at once, or
  // with wait_for_detector at the first detection.
  std::optional<double> start_t_{};
  bool detected_{ false };

  std::optional<double> latest_range_{};
  std::size_t rejected_in_a_row_{ 0 };
  static constexpr std::size_t max_rejected_detections = 10;

  std::size_t ranges_{ 0 };
  std::size_t ranges_rejected_{ 0 };
  std::size_t detections_{ 0 };
  std::size_t detections_rejected_{ 0 };
  std::size_t detections_late_{ 0 };
  std::size_t gps_fixes_{ 0 };
  std::size_t gps_rejected_{ 0 };
  std::size_t resets_{ 0 };
  std::size_t diverged_{ 0 };

public:
  explicit pole_tracking_sim (const rclcpp::Node::SharedPtr &node)
      : node_ (node),
        opt_ (pole_tracking_sim_parameters::ParamListener (node).get_params ()),
        frames_ (*node),
        frame_ (opt_.origin_latitude, opt_.origin_longitude, opt_.origin_altitude),
        rng_ (static_cast<unsigned> (opt_.seed))
  {
    for (auto &b : accel_bias_)
      {
        b = opt_.accel_bias * noise_ (rng_);
      }
  }

  void
  start ()
  {
    using geometry_msgs::msg::PoseStamped;
    camera_pub_ = node_->create_publisher<airsim_interfaces::msg::GimbalAngleQuatCmd> (
        "/airsim_node/gimbal_angle_quat_cmd", 10);
    fused_pub_ = node_->create_publisher<PoseStamped> ("/pole/fused", 100);
    ray_uwb_pub_ = node_->create_publisher<PoseStamped> ("/pole/ray_uwb", 100);
    gps_pub_ = node_->create_publisher<PoseStamped> ("/pole/gps", 10);
    const std::string prefix = "/airsim_node/drone/";
    const std::string camera = prefix + opt_.camera + "_Scene/";
    subscriptions_.push_back (node_->create_subscription<nav_msgs::msg::Odometry> (
        prefix + "odom_local", 100, [this] (const nav_msgs::msg::Odometry &m) { on_odom (m); }));
    subscriptions_.push_back (node_->create_subscription<yolo_msgs::msg::DetectionArray> (
        "/pole/detections", 50,
        [this] (const yolo_msgs::msg::DetectionArray &m) { on_detections (m); }));
    subscriptions_.push_back (node_->create_subscription<sensor_msgs::msg::CameraInfo> (
        camera + "camera_info", 10,
        [this] (const sensor_msgs::msg::CameraInfo &m) { on_camera_info (m); }));
    subscriptions_.push_back (node_->create_subscription<geometry_msgs::msg::PoseStamped> (
        camera + "pose", 100, [this] (const geometry_msgs::msg::PoseStamped &m) { on_camera_pose (m); }));
    if (!opt_.no_imu)
      {
        subscriptions_.push_back (node_->create_subscription<sensor_msgs::msg::Imu> (
            prefix + "imu/imu", 100, [this] (const sensor_msgs::msg::Imu &m) { on_imu (m); }));
      }
    if (!opt_.no_gps)
      {
        subscriptions_.push_back (node_->create_subscription<sensor_msgs::msg::NavSatFix> (
            prefix + "gps/gps", 10, [this] (const sensor_msgs::msg::NavSatFix &m) { on_gps (m); }));
      }
    spin_thread_ = std::thread ([this] { rclcpp::spin (node_); });
  }

  void
  run ()
  {
    std::cout << "waiting for AirSim's ROS 2 node and the pole camera\n"
              << "sensors: uwb, camera" << (!opt_.no_imu ? ", imu" : "")
              << (!opt_.no_gps ? ", gps" : "") << std::endl;
    while (stop_requested == 0)
      {
        auto event = inbox_.pop_for (std::chrono::milliseconds (200));
        if (!event)
          {
            continue;
          }
        if (!estimator_ && !place_pole ())
          {
            continue; // the camera's pose and intrinsics are not there yet
          }
        std::visit (overloaded{
                        [this] (timed_measurement &m) { receive (m); },
                        [this] (const truth_sample &s) { on_truth (s); },
                    },
                    *event);
        process_buffer ();

        if (sim_t_ - last_print_t_ >= 1.0)
          {
            last_print_t_ = sim_t_;
            print_status ();
          }
        if (opt_.duration > 0 && start_t_ && sim_t_ - *start_t_ >= opt_.duration)
          {
            break;
          }
      }
    rclcpp::shutdown ();
    if (spin_thread_.joinable ())
      spin_thread_.join ();
    print_summary ();
  }

private:
  // -- ROS callbacks (spin thread): only convert and enqueue ----------------

  // Seconds since the first message (AirSim stamps run on its own clock).
  double
  sim_time (const rclcpp::Time &stamp)
  {
    std::lock_guard lock (ros_mutex_);
    if (!t0_)
      t0_ = stamp;
    return (stamp - *t0_).seconds ();
  }

  // Whether t is in the flight (the run proper; see wait_for_detector).
  [[nodiscard]] bool
  flying_at (double t) const
  {
    return start_t_ && t >= *start_t_;
  }

  // The stamp of a time on the simulation's clock.
  rclcpp::Time
  stamp_of (double t)
  {
    std::lock_guard lock (ros_mutex_);
    return *t0_ + rclcpp::Duration::from_seconds (t);
  }

  void
  on_imu (const sensor_msgs::msg::Imu &msg)
  {
    // body -> AirSim's world, made body -> enu (see positioning_sim).
    const auto world = frames_.fixed ("world");
    if (!world)
      {
        return;
      }
    Eigen::Quaterniond q;
    tf2::fromMsg (msg.orientation, q);
    q = Eigen::Quaterniond (world->rotation ()) * q;
    const double t = sim_time (rclcpp::Time (msg.header.stamp));
    imu_sample sample{};
    sample.time_s = t;
    sample.orientation = q;
    const auto &a = msg.linear_acceleration;
    sample.specific_force = { a.x, a.y, a.z };
    inbox_.push (timed_measurement{ t, imu_event{ sample } });
  }

  void
  on_gps (const sensor_msgs::msg::NavSatFix &msg)
  {
    // Republished at the node's rate: a new fix has a new stamp.
    const rclcpp::Time stamp (msg.header.stamp);
    if (stamp.nanoseconds () == last_gps_stamp_)
      return;
    last_gps_stamp_ = stamp.nanoseconds ();
    vec3 v;
    {
      std::lock_guard lock (ros_mutex_);
      v = velocity_enu_;
    }
    inbox_.push (timed_measurement{ sim_time (stamp),
                                    navsat_event{ msg.latitude, msg.longitude, msg.altitude, v } });
  }

  // From yolo_ros or synthetic_detect.py, stamped with the frame's time: the
  // centre of the most confident box is the drone's pixel.
  void
  on_detections (const yolo_msgs::msg::DetectionArray &msg)
  {
    const auto best = std::max_element (msg.detections.begin (), msg.detections.end (),
                                        [] (const auto &a, const auto &b) { return a.score < b.score; });
    if (best == msg.detections.end ())
      {
        return;
      }
    inbox_.push (timed_measurement{
        sim_time (rclcpp::Time (msg.header.stamp)),
        detection_event{ best->bbox.center.position.x, best->bbox.center.position.y, best->score } });
  }

  void
  on_odom (const nav_msgs::msg::Odometry &msg)
  {
    // As in positioning_sim: the pose in the odometry frame, which tf2
    // places in enu; the linear velocity in AirSim's world axes.
    const auto odom = frames_.fixed (msg.header.frame_id);
    if (!odom)
      {
        return;
      }
    Eigen::Isometry3d body;
    tf2::fromMsg (msg.pose.pose, body);
    const Eigen::Isometry3d pose = *odom * body;
    vec3 v;
    tf2::fromMsg (msg.twist.twist.linear, v);
    const double t = sim_time (rclcpp::Time (msg.header.stamp));
    {
      std::lock_guard lock (ros_mutex_);
      velocity_enu_ = odom->rotation () * v;
    }
    inbox_.push (truth_sample{ t, pose.translation () });
  }

  // AirSim's pinhole: fx = fy, the principal point in the middle.
  void
  on_camera_info (const sensor_msgs::msg::CameraInfo &msg)
  {
    std::lock_guard lock (ros_mutex_);
    if (!camera_)
      {
        camera_ = pole_camera{};
        camera_->focal_px = msg.k[0];
        camera_->center_u_px = msg.k[2];
        camera_->center_v_px = msg.k[5];
      }
  }

  // The camera's pose for an image (AirSim's world frame, camera axes
  // forward, left, up) goes into tf2, where lookups interpolate it.
  void
  on_camera_pose (const geometry_msgs::msg::PoseStamped &msg)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg.header;
    transform.child_frame_id = camera_frame;
    transform.transform.translation.x = msg.pose.position.x;
    transform.transform.translation.y = msg.pose.position.y;
    transform.transform.translation.z = msg.pose.position.z;
    transform.transform.rotation = msg.pose.orientation;
    frames_.add (transform);
  }

  // -- main thread -----------------------------------------------------

  // Once AirSim has told where the camera is and how it sees: the UWB anchor
  // sits at the camera, and the mount starts from the rest pose.
  bool
  place_pole ()
  {
    std::optional<pole_camera> camera;
    {
      std::lock_guard lock (ros_mutex_);
      camera = camera_;
    }
    const auto pose = camera ? frames_.fixed (camera_frame) : std::nullopt;
    if (!pose)
      {
        return false;
      }
    anchor_ = pose->translation ();
    const vec3 forward = pose->rotation () * vec3::UnitX ();
    command_yaw_ = std::atan2 (forward.y (), forward.x ());
    command_pitch_ = -std::asin (std::clamp (forward.z (), -1.0, 1.0));
    pole_estimator_config config{};
    config.range_sigma_m = std::max (opt_.uwb_noise, 0.02);
    config.bearing_sigma_rad = 4.0 / camera->focal_px; // about 4 pixels
    estimator_.emplace (config, anchor_);
    std::cout << std::fixed << std::setprecision (2) << "pole camera at (" << anchor_.x ()
              << ", " << anchor_.y () << ", " << anchor_.z () << ") m enu, facing "
              << command_yaw_ / deg << " deg, pitch " << command_pitch_ / deg
              << " deg, focal length " << camera->focal_px << " px" << std::endl;
    return true;
  }

  // The camera as it was when it took the frame at time t (tf2 interpolates
  // between the poses AirSim published).
  [[nodiscard]] std::optional<pole_camera>
  camera_at (double t)
  {
    const auto pose = frames_.lookup (camera_frame, stamp_of (t));
    if (!pose)
      {
        return std::nullopt;
      }
    std::lock_guard lock (ros_mutex_);
    pole_camera c = *camera_;
    c.pose = *pose;
    return c;
  }

  // Turns the pan-tilt mount towards where the estimate says the drone is
  // now. The estimate lags the simulation by the fusion latency, so it is
  // carried forward with its velocity, plus a little lead for the mount.
  void
  steer ()
  {
    if (opt_.fixed_camera || !estimator_->initialized () || sim_t_ - last_steer_t_ < 0.02)
      {
        return;
      }
    const auto world = frames_.fixed ("world");
    if (!world)
      {
        return;
      }
    const double dt = last_steer_t_ < 0 ? 0.02 : sim_t_ - last_steer_t_;
    last_steer_t_ = sim_t_;

    constexpr double lead_s = 0.1;
    const double ahead = sim_t_ - estimator_->time () + lead_s;
    const vec3 d = estimator_->position () + (ahead * estimator_->velocity ()) - anchor_;
    if (!d.allFinite ())
      {
        return;
      }
    const double want_yaw = std::atan2 (d.y (), d.x ());
    const double want_pitch
        = std::clamp (-std::atan2 (d.z (), std::hypot (d.x (), d.y ())), -80 * deg, 30 * deg);

    const double step = opt_.slew_rate * deg * dt;
    command_yaw_ = std::remainder (
        command_yaw_ + std::clamp (std::remainder (want_yaw - command_yaw_, 2 * pi), -step, step),
        2 * pi);
    command_pitch_ += std::clamp (want_pitch - command_pitch_, -step, step);

    // The camera orientation in enu, turned into AirSim's world frame (its
    // ROS node turns the camera: the gimbal command).
    const Eigen::Quaterniond q
        = Eigen::Quaterniond (world->rotation ()).conjugate ()
          * Eigen::Quaterniond (
              pole_camera::oriented (vec3::Zero (), command_yaw_, command_pitch_).rotation ());
    airsim_interfaces::msg::GimbalAngleQuatCmd command;
    command.camera_name = opt_.camera;
    command.vehicle_name = "drone";
    command.orientation = tf2::toMsg (q);
    camera_pub_->publish (command);
  }

  void
  receive (timed_measurement &m)
  {
    if (std::holds_alternative<detection_event> (m.value) && m.t < sim_t_ - opt_.latency)
      {
        ++detections_late_;
        return;
      }
    detected_ = detected_ || std::holds_alternative<detection_event> (m.value);
    buffer_.emplace (m.t, std::move (m.value));
  }

  void
  on_truth (const truth_sample &s)
  {
    sim_t_ = std::max (sim_t_, s.t);

    // The flight (the run proper) starts with the first detection, or at
    // once without wait_for_detector; the autopilot waits the same way.
    if (!start_t_ && (detected_ || !opt_.wait_for_detector))
      {
        start_t_ = s.t;
        std::cout << "flight starts at t=" << std::fixed << std::setprecision (1) << s.t
                  << " s" << std::endl;
      }

    // The UWB radio: the true distance plus noise, at 20 Hz.
    if (s.t - last_range_t_ >= 0.05)
      {
        last_range_t_ = s.t;
        const double range = (s.position - anchor_).norm () + (opt_.uwb_noise * noise_ (rng_));
        buffer_.emplace (s.t, range_event{ range });
      }

    steer ();
  }

  void
  process_buffer ()
  {
    while (!buffer_.empty () && buffer_.begin ()->first <= sim_t_ - opt_.latency)
      {
        const auto node = buffer_.extract (buffer_.begin ());
        const double t = node.key ();
        estimator_->advance_to (t);
        std::visit ([this, t] (const auto &m) { handle (t, m); }, node.mapped ());
        if (estimator_->initialized () && !estimator_->healthy ())
          {
            // Start over from the next measurement rather than carry a
            // broken state (and steer the camera with it).
            estimator_->reset ();
            ++diverged_;
            std::cerr << "estimate diverged at t=" << t << ", restarting" << std::endl;
          }
        if (estimator_->initialized ())
          {
            publish (t);
          }
      }
  }

  void
  handle (double t, const imu_event &e)
  {
    estimator_->set_acceleration (with_imu_errors (t, e.sample).acceleration_enu (world_gravity));
  }

  [[nodiscard]] imu_sample
  with_imu_errors (double t, imu_sample sample)
  {
    const vec3 sigma{ opt_.tilt_error * deg, opt_.tilt_error * deg, opt_.heading_error * deg };
    const double phi
        = last_imu_t_ < 0 ? 0 : std::exp (-(t - last_imu_t_) / opt_.orientation_error_tau);
    last_imu_t_ = t;
    for (int i = 0; i < 3; ++i)
      {
        orientation_error_[i] = (phi * orientation_error_[i])
                                + (sigma[i] * std::sqrt (1 - phi * phi) * noise_ (rng_));
      }
    sample.specific_force += accel_bias_;
    // The orientation maps sensor to local, so the local frame error goes on
    // the left.
    sample.orientation = from_rotation_vector (orientation_error_) * sample.orientation;
    return sample;
  }

  void
  handle (double t, const navsat_event &e)
  {
    vec3 fix;
    frame_.Forward (e.lat, e.lon, e.alt, fix.x (), fix.y (), fix.z ());
    fix += vec3{ opt_.gps_noise * noise_ (rng_), opt_.gps_noise * noise_ (rng_),
                 opt_.gps_vfactor * opt_.gps_noise * noise_ (rng_) };
    ++gps_fixes_;
    if (flying_at (t))
      {
        gps_pub_->publish (ros_sim::pose_msg (stamp_of (t), fix));
      }

    const double sigma = std::max (opt_.gps_noise, 1.0);
    if (!estimator_->initialized ())
      {
        estimator_->initialize (t, fix, 2 * sigma);
        return;
      }
    if (!estimator_->add_gps (fix, sigma, sigma * opt_.gps_vfactor))
      {
        ++gps_rejected_;
        return;
      }
    (void)estimator_->add_gps_velocity (e.velocity.x () + (opt_.speed_noise * noise_ (rng_)),
                                        e.velocity.y () + (opt_.speed_noise * noise_ (rng_)),
                                        0.3);
  }

  void
  handle (double /*t*/, const range_event &e)
  {
    latest_range_ = e.range_m;
    if (!estimator_->initialized ())
      {
        return;
      }
    ++ranges_;
    if (!estimator_->add_range (e.range_m))
      {
        ++ranges_rejected_;
      }
  }

  void
  handle (double t, const detection_event &e)
  {
    ++detections_;
    if (!latest_range_)
      {
        return;
      }
    // Where the camera pointed when it took the frame.
    const auto camera = camera_at (t);
    if (!camera)
      {
        return;
      }
    // The direct answer: the point on the camera ray at the UWB distance.
    const vec3 ray = camera->ray (e.u, e.v);
    const vec3 raw = anchor_ + (*latest_range_ * ray);
    if (flying_at (t))
      {
        ray_uwb_pub_->publish (ros_sim::pose_msg (stamp_of (t), raw));
      }

    if (!estimator_->initialized ())
      {
        estimator_->initialize (t, raw, 1.0);
        return;
      }
    if (estimator_->add_bearing (ray))
      {
        rejected_in_a_row_ = 0;
        return;
      }
    ++detections_rejected_;
    if (++rejected_in_a_row_ > max_rejected_detections)
      {
        // Many detections in a row disagree: trust them, not the state.
        estimator_->initialize (t, raw, 1.0);
        rejected_in_a_row_ = 0;
        ++resets_;
      }
  }

  // Publishes the fused estimate (at most 50 Hz) and, for the status line,
  // compares it with the truth; evaluate.py scores the recording.
  void
  publish (double t)
  {
    if (!flying_at (t) || t - last_publish_t_ < 0.02)
      {
        return;
      }
    last_publish_t_ = t;
    const rclcpp::Time stamp = stamp_of (t);
    const vec3 estimate = estimator_->position ();
    fused_pub_->publish (ros_sim::pose_msg (stamp, estimate, ros_sim::heading_of (estimator_->velocity ())));

    const auto truth = frames_.lookup ("drone", stamp);
    if (!truth)
      {
        return;
      }
    last_error_ = (estimate - truth->translation ()).norm ();
    if (const auto camera = camera_at (t))
      {
        ++frames_scored_;
        if (camera->sees (truth->translation ()))
          {
            ++frames_in_view_;
          }
      }
  }

  // -- reporting ---------------------------------------------------------

  void
  print_status () const
  {
    std::cout << std::fixed << std::setprecision (2) << "t=" << std::setw (6) << sim_t_
              << "s | fused error now "
              << (last_error_ ? std::to_string (*last_error_) : std::string ("-"))
              << " m | detections " << detections_ << " (rejected " << detections_rejected_
              << ", late " << detections_late_ << ")" << std::endl;
  }

  [[nodiscard]] double
  in_view_percent () const
  {
    return frames_scored_ == 0 ? 0.0
                               : 100.0 * static_cast<double> (frames_in_view_)
                                     / static_cast<double> (frames_scored_);
  }

  void
  print_summary () const
  {
    std::cout << std::fixed << std::setprecision (1) << "\ndone after " << sim_t_
              << " s, sensors: uwb, camera" << (!opt_.no_imu ? ", imu" : "")
              << (!opt_.no_gps ? ", gps" : "") << "\n  camera "
              << (opt_.fixed_camera ? "fixed" : "tracking") << ", drone in view "
              << in_view_percent () << " % of the time\n  detections: " << detections_
              << " (rejected " << detections_rejected_ << ", too late " << detections_late_
              << ", resets " << resets_ << ", diverged " << diverged_
              << "), uwb ranges: " << ranges_ << " (rejected " << ranges_rejected_ << ")";
    if (!opt_.no_gps)
      {
        std::cout << ", gps fixes: " << gps_fixes_ << " (rejected " << gps_rejected_ << ")";
      }
    std::cout << std::endl;
  }
};
} // namespace

int
main (int argc, char *argv[])
{
  rclcpp::init (argc, argv, rclcpp::InitOptions (), rclcpp::SignalHandlerOptions::None);
  std::signal (SIGINT, handle_signal);
  std::signal (SIGTERM, handle_signal);
  try
    {
      pole_tracking_sim sim (rclcpp::Node::make_shared ("pole_tracking_sim"));
      sim.start ();
      sim.run ();
    }
  catch (const std::exception &e)
    {
      std::cerr << "pole_tracking_sim: " << e.what () << "\n";
      rclcpp::shutdown ();
      return 2;
    }
  return 0;
}
