#include "drone.h"

#include <dji_fc_subscription.h>  // T_DjiFcSubscriptionQuaternion
#include <dji_gimbal_manager.h>   // DjiGimbalManager_Init
#include <dji_mop_channel.h>      // DjiMopChannel_Init
#include <spdlog/spdlog.h>

#include <future>

#include "api_code.h"
#include "converter.h"
#include "mission_builder.h"
#include "olyseus_verify.h"  // OLYSEUS_VERIFY
#include "server.h"
#include "utils.h"  // rad2deg

mission drone::mission_;

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
simulator drone::simulator_;
#endif

std::atomic<bool> drone::sigint_received_{false};

std::atomic<float> drone::drone_yaw_{0.0};
std::atomic<float> drone::drone_pitch_{0.0};
std::atomic<float> drone::drone_roll_{0.0};

std::atomic<double> drone::drone_longitude_{0.0};
std::atomic<double> drone::drone_latitude_{0.0};
std::atomic<float> drone::drone_altitude_{0.0};

std::atomic<float> drone::gimbal_yaw_;
std::atomic<float> drone::gimbal_pitch_;
std::atomic<float> drone::gimbal_roll_;

std::atomic<float> drone::homepoint_altitude_{invalid_homepoint_altitude_};
std::atomic<int16_t> drone::rc_mode_{-1};

std::mutex drone::execute_commands_mutex_;
std::list<interconnection::command_type::command_t> drone::execute_commands_;

condition_flag drone::action_flag_;

template <class T>
const T& drone::cast_dji(const uint8_t* data, uint16_t data_size) {
  OLYSEUS_VERIFY(data != nullptr);
  OLYSEUS_VERIFY(data_size == sizeof(T));

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  return *reinterpret_cast<const T*>(data);
}

auto drone::quaternion_callback(const uint8_t* data, uint16_t data_size,
                                const T_DjiDataTimestamp* timestamp)
    -> T_DjiReturnCode {
  const auto quaternion{
      cast_dji<T_DjiFcSubscriptionQuaternion>(data, data_size)};

  (void)timestamp;

  // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
  // NOLINTBEGIN (readability-magic-numbers)

  // https://github.com/dji-sdk/Onboard-SDK/blob/2c38de17f7aad0064056f27eaa219d4ed30ab82a/sample/platform/STM32/OnBoardSDK_STM32/User/FlightControlSample.cpp#L800-L824
  const float q2sqr{quaternion.q2 * quaternion.q2};
  const float t0{-2.0F * (q2sqr + quaternion.q3 * quaternion.q3) + 1.0F};
  const float t1{
      +2.0F * (quaternion.q1 * quaternion.q2 + quaternion.q0 * quaternion.q3)};
  float t2{-2.0F *
           (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2)};
  const float t3{
      +2.0F * (quaternion.q2 * quaternion.q3 + quaternion.q0 * quaternion.q1)};
  const float t4{-2.0F * (quaternion.q1 * quaternion.q1 + q2sqr) + 1.0F};

  // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)
  // NOLINTEND (readability-magic-numbers)

  t2 = (t2 > 1.0F) ? 1.0F : t2;
  t2 = (t2 < -1.0F) ? -1.0F : t2;

  // https://sdk-forum.dji.net/hc/en-us/requests/74003
  // https://sdk-forum.dji.net/hc/en-us/articles/360023657273
  drone_roll_ = atan2f(t3, t4) * rad2deg_f;  // X
  drone_pitch_ = asinf(t2) * rad2deg_f;      // Y
  drone_yaw_ = atan2f(t1, t0) * rad2deg_f;   // Z

  spdlog::debug("roll: {}, pitch: {}, yaw: {}", drone_roll_, drone_pitch_,
                drone_yaw_);

  constexpr float right_angle{90.0F};
  constexpr float straight_angle{180.0F};

  OLYSEUS_VERIFY(drone_yaw_ >= -straight_angle);
  OLYSEUS_VERIFY(drone_yaw_ <= straight_angle);
  OLYSEUS_VERIFY(drone_pitch_ > -right_angle);
  OLYSEUS_VERIFY(drone_pitch_ < right_angle);
  OLYSEUS_VERIFY(drone_roll_ > -right_angle);
  OLYSEUS_VERIFY(drone_roll_ < right_angle);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

auto drone::rc_callback(const uint8_t* data, uint16_t data_size,
                        const T_DjiDataTimestamp* timestamp)
    -> T_DjiReturnCode {
  const auto rc{cast_dji<T_DjiFcSubscriptionRC>(data, data_size)};

  (void)timestamp;

  rc_mode_ = rc.mode;

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

auto drone::position_fused_callback(const uint8_t* data, uint16_t data_size,
                                    const T_DjiDataTimestamp* timestamp)
    -> T_DjiReturnCode {
  const auto position{
      cast_dji<T_DjiFcSubscriptionPositionFused>(data, data_size)};

  (void)timestamp;

  const double lat = position.latitude * rad2deg;
  const double lon = position.longitude * rad2deg;
  const double alt = position.altitude;  // Altitude, WGS 84 reference ellipsoid

  spdlog::debug("GPS fused, drone latitude: {}, longitude: {}, altitude: {}",
                lat, lon, alt);

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  // RTK is OFF
  drone_latitude_ = lat;
  drone_longitude_ = lon;
  drone_altitude_ = alt;

#if !defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
  simulator_.gps_callback(drone_latitude_, drone_longitude_);
#endif
#else
  // RTK is ON
  if (std::abs(drone_latitude_) > 1e-2 && std::abs(drone_longitude_) > 1e-2) {
    OLYSEUS_VERIFY(std::abs(drone_altitude_ - alt) < 5.0);
  }
#endif

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

#if !defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
auto drone::rtk_position_callback(const uint8_t* data, uint16_t data_size,
                                  const T_DjiDataTimestamp* timestamp)
    -> T_DjiReturnCode {
  const auto position{
      cast_dji<T_DjiFcSubscriptionRtkPosition>(data, data_size)};

  (void)timestamp;

  drone_latitude_ = position.latitude;
  drone_longitude_ = position.longitude;

  // In documentation it's "height above mean sea level" but in fact
  // the type depends on RTK settings
  // - https://sdk-forum.dji.net/hc/en-us/requests/82680
  drone_altitude_ = position.hfsl;

  spdlog::debug("RTK, drone latitude: {}, longitude: {}, altitude: {}",
                drone_latitude_, drone_longitude_, drone_altitude_);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
#endif

auto drone::gimbal_callback(const uint8_t* data, uint16_t data_size,
                            const T_DjiDataTimestamp* timestamp)
    -> T_DjiReturnCode {
  const auto gimbal_three_data{
      cast_dji<T_DjiFcSubscriptionThreeGimbalData>(data, data_size)};
  const GimbalSingleData d{gimbal_three_data.anglesData[0]};

  (void)timestamp;

  gimbal_yaw_ = d.yaw;
  gimbal_pitch_ = d.pitch;
  gimbal_roll_ = d.roll;

  spdlog::debug("gimbal pitch: {}, roll: {}, yaw: {}", gimbal_pitch_,
                gimbal_roll_, gimbal_yaw_);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

auto drone::mission_event_callback(T_DjiWaypointV2MissionEventPush event_data)
    -> T_DjiReturnCode {
  const bool notify{mission_.update(event_data)};
  if (notify) {
    action_flag_.notify();
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

auto drone::mission_state_callback(T_DjiWaypointV2MissionStatePush state_data)
    -> T_DjiReturnCode {
  auto [mission_started, notify] = mission_.update(state_data);

  if (!mission_started) {
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
  }

  if (notify) {
    action_flag_.notify();
  }

  // https://developer.dji.com/onboard-api-reference/structDJI_1_1OSDK_1_1Telemetry_1_1RC.html#a9e69e1b32599986319ad3312ca5723de
  constexpr int16_t expected_rc_mode{8000};
  if (rc_mode_ != expected_rc_mode) {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
    constexpr int16_t expected_sim_rc_mode{-8000};
    OLYSEUS_VERIFY(rc_mode_ == expected_sim_rc_mode);
#else
    // Value received while running tests on simulator
    spdlog::error("Unexpected RC mode: {}", rc_mode_);
    OLYSEUS_UNREACHABLE;
#endif
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

auto drone::homepoint_callback(const uint8_t* data, uint16_t data_size,
                               const T_DjiDataTimestamp* timestamp)
    -> T_DjiReturnCode {
  const auto altitude{
      cast_dji<T_DjiFcSubscriptionAltitudeOfHomePoint>(data, data_size)};

  (void)timestamp;

  homepoint_altitude_ = altitude;
  OLYSEUS_VERIFY(homepoint_altitude_ > invalid_homepoint_altitude_);

  spdlog::debug("home altitude: {}", homepoint_altitude_);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

drone::drone()
    : camera_("/var/opt/i_seed_drone_onboard/best.engine", mission_) {
  OLYSEUS_VERIFY(sigint_received_.is_lock_free());

  constexpr E_DjiDataSubscriptionTopicFreq topic_freq{
      DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ};

  T_DjiReturnCode code{DjiFcSubscription_Init()};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // DjiFcSubscription_SubscribeTopic usage note:
  //   avoid locks in callback, it should exit as soon as possible

  code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                          topic_freq, quaternion_callback);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC,
                                          topic_freq, rc_callback);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code =
      DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                       topic_freq, position_fused_callback);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

#if !defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  // RTK: ON
  code =
      DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                       topic_freq, rtk_position_callback);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
#endif

  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA, topic_freq, gimbal_callback);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // https://sdk-forum.dji.net/hc/en-us/requests/76593
  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
      DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, homepoint_callback);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiGimbalManager_Init();
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiGimbalManager_SetMode(m_pos, DJI_GIMBAL_MODE_FREE);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_Init();
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_RegisterMissionEventCallback(mission_event_callback);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_RegisterMissionStateCallback(mission_state_callback);
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  {
    interconnection::packet_size p_size;
    p_size.set_size(0);
    std::string buffer;
    bool ok{p_size.SerializeToString(&buffer)};
    OLYSEUS_VERIFY(ok);
    packet_size_ = {static_cast<uint32_t>(buffer.size())};
    OLYSEUS_VERIFY(packet_size_ > 0);
  }

  constexpr int sleep_ms{2000};
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
}

void drone::sigint_handler(int signal) {
  (void)signal;
  spdlog::critical("SIGINT received");
  sigint_received_ = true;
}

drone::~drone() {
  T_DjiReturnCode code{DjiFcSubscription_DeInit()};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_Deinit();
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

void drone::start() {
  spdlog::info("Protocol version: {}", protocol_version);
  spdlog::info("Packet size: {}", packet_size_);

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  spdlog::info("SIMULATOR MODE");
#endif

  const T_DjiReturnCode code = DjiMopChannel_Init();
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  std::thread action_thread{&drone::action_job, this};
  std::thread inference_thread{&drone::inference_job, this};

  while (!interrupt_condition()) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast)
    OLYSEUS_VERIFY(signal(SIGINT, SIG_DFL) != SIG_ERR);
    const server server{channel_id};
    channel_handle_ = server.handle();
#if defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
    OLYSEUS_VERIFY(channel_handle_ != nullptr);
#else
    OLYSEUS_VERIFY(channel_handle_ == nullptr);
#endif

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast)
    OLYSEUS_VERIFY(signal(SIGINT, sigint_handler) != SIG_ERR);

    connection_closed_ = false;

    std::thread receive_data_thread{&drone::receive_data_job, this};
    std::thread send_data_thread{&drone::send_data_job, this};

    receive_data_thread.join();
    send_data_thread.join();
  }

  // The main loop is endless so we reach this point only in case of error

  // Wake up all the threads that can possible wait for been notified
  action_flag_.notify();
  action_thread.join();

  // Wait for other threads to finish. No need to nofify
  inference_thread.join();

  if (sigint_received_) {
    throw std::runtime_error("SIGINT received");
  }

  OLYSEUS_VERIFY(exception_caught_);
  throw std::runtime_error("Exception in thread");
}

auto drone::interrupt_condition() const -> bool {
  return exception_caught_ || sigint_received_;
}

void drone::action_job() {
  try {
    while (true) {
      action_flag_.wait();

      if (interrupt_condition()) {
        spdlog::critical("Action job exit");
        return;
      }

      bool already_paused{false};

      while (true) {
        if (already_paused) {
          spdlog::info("Mission is paused, waiting for RESUME...");
          constexpr int sleep_ms{400};
          std::this_thread::sleep_for(std::chrono::milliseconds{sleep_ms});
        }

        const std::lock_guard lock{action_job_mutex_};

        already_paused = mission_.is_paused();

        if (already_paused) {
          continue;
        }

        action_job_internal();
        break;
      }
    }
  } catch (std::exception& e) {
    exception_caught_ = true;
    spdlog::critical("Exception: {}", e.what());
  }
}

void drone::action_job_internal() {
  if (mission_.is_ready()) {
    return;
  }

  if (mission_.is_finished()) {
    next_mission();
    return;
  }

  spdlog::info("Pause mission");
  T_DjiReturnCode code{DjiWaypointV2_Pause()};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // Wait for drone to finish the movement
  constexpr int sleep_ms{1000};
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));

  const auto [action, waypoint_index] =
      mission_.waypoint_reached(latest_laser_range());
  const bool is_forward{mission_.is_forward()};

  constexpr int32_t event_id{mission_state::internal_event_id};

  switch (action) {
    case waypoint_action::ok:
      break;
    case waypoint_action::abort:
      spdlog::info("Abort mission on a last fake waypoint");
      mission_.abort(event_id);
      next_mission();
      return;
    case waypoint_action::restart: {  // restart the mission for altitude tweak
      // altitude tweak is only for forward mission
      OLYSEUS_VERIFY(is_forward);
      mission_.abort(event_id);
      const bool ok{mission_.upload_mission_and_start(event_id)};
      OLYSEUS_VERIFY(ok);
      return;
    }
    default:
      OLYSEUS_UNREACHABLE;
  }

  if (is_forward) {
    align_gimbal();
  }

  OLYSEUS_VERIFY(std::abs(drone_latitude_) > 1e-2);
  OLYSEUS_VERIFY(std::abs(drone_longitude_) > 1e-2);

  spdlog::info("drone latitude: {}, longitude: {}, altitude: {}",
               drone_latitude_, drone_longitude_, drone_altitude_);
  spdlog::info("drone roll: {}, pitch: {}, yaw: {}", drone_roll_, drone_pitch_,
               drone_yaw_);
  spdlog::info("gimbal pitch: {}, roll: {}, yaw: {}", gimbal_pitch_,
               gimbal_roll_, gimbal_yaw_);

  const waypoint w{mission_.get_waypoint_copy(waypoint_index)};

  constexpr double eps{1e-4};
  OLYSEUS_VERIFY(std::abs(w.lat() - drone_latitude_) < eps);
  OLYSEUS_VERIFY(std::abs(w.lon() - drone_longitude_) < eps);

  OLYSEUS_VERIFY(homepoint_altitude_ > invalid_homepoint_altitude_);
  home_altitude_.set_altitude(drone_altitude_, w.altitude(),
                              homepoint_altitude_);

  if (is_forward) {
    const gps_coordinates gps{.longitude = drone_longitude_,
                              .latitude = drone_latitude_,
                              .altitude = drone_altitude_};

    const attitude drone_attitude{
        .pitch = drone_pitch_, .roll = drone_roll_, .yaw = drone_yaw_};

    const attitude gimbal_attitude{
        .pitch = gimbal_pitch_, .roll = gimbal_roll_, .yaw = gimbal_yaw_};

    const double yaw_diff{std::abs(drone_yaw_ - gimbal_yaw_)};
    spdlog::info("Gimbal/drone yaw diff: {}", yaw_diff);
    constexpr double max_diff{0.7};
    OLYSEUS_VERIFY(yaw_diff < max_diff);
    camera_.shoot_photo(gps, drone_attitude, gimbal_attitude, waypoint_index);
  } else {
    OLYSEUS_VERIFY(w.has_detection());
    const detection_result d{w.get_detection()};
    OLYSEUS_VERIFY(!d.pixels.empty());

    for (const detected_pixel& p : d.pixels) {
      attitude pixel_gimbal_attitude{rotate_gimbal(p.x, p.y, drone_yaw_)};
      pixel_gimbal_attitude.yaw -= drone_yaw_;  // yaw relative to drone

      spdlog::info("drone latitude: {}, longitude: {}, altitude: {}",
                   drone_latitude_, drone_longitude_, drone_altitude_);
      spdlog::info("drone roll: {}, pitch: {}, yaw: {}", drone_roll_,
                   drone_pitch_, drone_yaw_);
      spdlog::info("gimbal pitch: {}, roll: {}, yaw: {}", gimbal_pitch_,
                   gimbal_roll_, gimbal_yaw_);

      const gps_coordinates gps{.longitude = drone_longitude_,
                                .latitude = drone_latitude_,
                                .altitude = drone_altitude_};

      const attitude drone_attitude{
          .pitch = drone_pitch_, .roll = drone_roll_, .yaw = drone_yaw_};

      const attitude laser_gimbal_attitude{
          .pitch = gimbal_pitch_,
          .roll = gimbal_roll_,
          .yaw = gimbal_yaw_ - drone_yaw_};  // yaw relative to drone

      const converter_result pixel_result{
          converter::run(d.gps, d.drone_attitude, pixel_gimbal_attitude, 1.0F)};
      const converter_result laser_result{converter::run(
          gps, drone_attitude, laser_gimbal_attitude, latest_laser_range())};

      constexpr double eps{1e-3};
      constexpr double max_dist{5.0};

      OLYSEUS_VERIFY((pixel_result.p - laser_result.p).norm() < max_dist);
      OLYSEUS_VERIFY(std::abs(pixel_result.d.dot(laser_result.d) - 1.0) < eps);

      const Eigen::Vector3d p_laser_end{laser_result.p + laser_result.v};
      spdlog::info("{} {} {} 255 165 0", p_laser_end(0), p_laser_end(1),
                   p_laser_end(2));  // orange

      const double k_num{
          laser_result.d.dot(laser_result.p - pixel_result.p + laser_result.v)};
      const double k_denom{pixel_result.v.dot(laser_result.d)};
      OLYSEUS_VERIFY(k_denom > 1e-2);  // NOLINT(*-magic-numbers)
      const double k{k_num / k_denom};
      OLYSEUS_VERIFY(k > eps);

      const Eigen::Vector3d p_pixel_end{pixel_result.p + k * pixel_result.v};
      spdlog::info("{} {} {} 0 255 0", p_pixel_end(0), p_pixel_end(1),
                   p_pixel_end(2));
      constexpr double sanity_dist{10.0};
      OLYSEUS_VERIFY((p_pixel_end - p_laser_end).norm() < sanity_dist);

      // FIXME (save to file)
    }
  }

  spdlog::info("Resume mission");
  code = DjiWaypointV2_Resume();
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

void drone::align_gimbal() {
  constexpr float expected_gimbal_pitch{-90.0F};  // down

  constexpr int time_ms{500};
  constexpr int time_wait_ms{3 * time_ms};
  const double ms{1.0 / 1000.0};

  constexpr float expected_gimbal_roll{0.0F};
  const float expected_gimbal_yaw{drone_yaw_};

  T_DjiGimbalManagerRotation rotation;
  rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
  rotation.pitch = expected_gimbal_pitch - gimbal_pitch_;
  rotation.roll = expected_gimbal_roll - gimbal_roll_;
  rotation.yaw = expected_gimbal_yaw - gimbal_yaw_;
  rotation.time = time_ms * ms;

  constexpr float eps{1e-3};
  constexpr float rough_eps{0.1 + eps};
  const double d_roll{std::abs(rotation.roll)};
  const double d_pitch{std::abs(rotation.pitch)};
  const double d_yaw{std::abs(rotation.yaw)};
  if (d_roll < rough_eps && d_pitch < rough_eps && d_yaw < rough_eps) {
    return;
  }

  spdlog::info("Current gimbal yaw: {}, expected: {}", gimbal_yaw_,
               expected_gimbal_yaw);
  spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}",
               rotation.yaw, rotation.roll, rotation.pitch);

  const T_DjiReturnCode code{DjiGimbalManager_Rotate(m_pos, rotation)};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));

  constexpr double expected_eps{0.3};
  const double d_yaw_2{expected_gimbal_yaw - gimbal_yaw_};

  if (std::abs(d_yaw_2) > expected_eps) {
    spdlog::info("Gimbal rotation failed, trying another direction, diff: {}",
                 d_yaw_2);
    constexpr double rotate{178.0};

    T_DjiGimbalManagerRotation rotation;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.pitch = 0.0;
    rotation.roll = 0.0;
    rotation.yaw = (d_yaw_2 > 0.0) ? -rotate : rotate;
    rotation.time = time_ms * ms;

    spdlog::info("Current gimbal yaw: {}", gimbal_yaw_);
    spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}",
                 rotation.yaw, rotation.roll, rotation.pitch);

    T_DjiReturnCode code{DjiGimbalManager_Rotate(m_pos, rotation)};
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));

    const float expected_gimbal_yaw{drone_yaw_};

    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.pitch = expected_gimbal_pitch - gimbal_pitch_;
    rotation.roll = expected_gimbal_roll - gimbal_roll_;
    rotation.yaw = expected_gimbal_yaw - gimbal_yaw_;
    rotation.time = time_ms * ms;

    spdlog::info("Current gimbal yaw: {}", gimbal_yaw_);
    spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}",
                 rotation.yaw, rotation.roll, rotation.pitch);

    code = DjiGimbalManager_Rotate(m_pos, rotation);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));
  }
}

auto drone::rotate_gimbal(float x, float y, double drone_heading_degree)
    -> attitude {
  spdlog::info("Gimbal rotate to pixel {}, {}", x, y);

  attitude expected{.pitch = 0.0, .roll = 0.0, .yaw = 0.0};
  std::tie(expected.yaw, expected.pitch) =
      gimbal_rotation_params(x, y, drone_heading_degree);

  constexpr int time_ms{500};
  constexpr int time_wait_ms{3 * time_ms};
  const double ms{1.0 / 1000.0};

  T_DjiGimbalManagerRotation rotation;
  rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
  rotation.pitch = expected.pitch - gimbal_pitch_;
  rotation.roll = expected.roll - gimbal_roll_;
  rotation.yaw = expected.yaw - gimbal_yaw_;
  rotation.time = time_ms * ms;

  spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}",
               rotation.yaw, rotation.roll, rotation.pitch);

  const T_DjiReturnCode code{DjiGimbalManager_Rotate(m_pos, rotation)};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));

  constexpr double expected_eps{0.3};
  const double d_roll{expected.roll - gimbal_roll_};
  const double d_pitch{expected.pitch - gimbal_pitch_};
  const double d_yaw{expected.yaw - gimbal_yaw_};

  if (std::abs(d_roll) > expected_eps || std::abs(d_pitch) > expected_eps ||
      std::abs(d_yaw) > expected_eps) {
    spdlog::info(
        "Gimbal rotation failed, trying another direction, diff roll: {}, "
        "pitch: {}, yaw: {}",
        d_roll, d_pitch, d_yaw);

    const double ms{1.0 / 1000.0};
    constexpr double rotate{178.0};

    T_DjiGimbalManagerRotation rotation;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.pitch = 0.0;
    rotation.roll = 0.0;
    rotation.yaw = (d_yaw > 0.0) ? -rotate : rotate;
    rotation.time = time_ms * ms;

    spdlog::info("Current gimbal yaw: {}", gimbal_yaw_);
    spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}",
                 rotation.yaw, rotation.roll, rotation.pitch);

    T_DjiReturnCode code{DjiGimbalManager_Rotate(m_pos, rotation)};
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));

    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.pitch = expected.pitch - gimbal_pitch_;
    rotation.roll = expected.roll - gimbal_roll_;
    rotation.yaw = expected.yaw - gimbal_yaw_;
    rotation.time = time_ms * ms;

    spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}",
                 rotation.yaw, rotation.roll, rotation.pitch);

    code = DjiGimbalManager_Rotate(m_pos, rotation);
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));
  }

  return expected;
}

void drone::inference_job() {
  try {
    while (true) {
      if (interrupt_condition()) {
        spdlog::critical("Inference job exit");
        return;
      }
      constexpr bool debug_launch{false};
      camera_.check_sdcard(debug_launch);
      std::this_thread::sleep_for(std::chrono::seconds{1});
    }
  } catch (std::exception& e) {
    exception_caught_ = true;
    spdlog::critical("Exception: {}", e.what());
  }
}

void drone::receive_data_job() {
  try {
    receive_data_job_internal();
  } catch (job_interrupted_event&) {
    OLYSEUS_VERIFY(interrupt_condition());
  } catch (pipeline_closed&) {
    connection_closed_ = true;
  } catch (std::exception& e) {
    exception_caught_ = true;
    spdlog::critical("Exception: {}", e.what());
    OLYSEUS_VERIFY(interrupt_condition());
  }
}

void drone::receive_data_job_internal() {
  spdlog::info("Received data job started");

  std::string buffer;

  while (true) {
    buffer.resize(receive_next_packet_size());
    receive_data(&buffer);

    interconnection::command_type command;
    const bool ok{command.ParseFromString(buffer)};
    OLYSEUS_VERIFY(ok);

    switch (command.type()) {
      case interconnection::command_type::PING: {
        const std::lock_guard lock{execute_commands_mutex_};
        execute_commands_.push_back(interconnection::command_type::PONG);
      } break;
      case interconnection::command_type::BUILD_MISSION: {
        buffer.resize(receive_next_packet_size());
        receive_data(&buffer);

        interconnection::input_polygon input_polygon_proto;
        const bool ok{input_polygon_proto.ParseFromString(buffer)};
        OLYSEUS_VERIFY(ok);

        const int32_t event_id{input_polygon_proto.event_id()};
        OLYSEUS_VERIFY(event_id != mission_state::internal_event_id);

        OLYSEUS_VERIFY(input_polygon_proto.vertices_size() > 0);
        std::vector<lat_lon> input_polygon;
        input_polygon.reserve(input_polygon_proto.vertices_size());
        for (int i{0}; i < input_polygon_proto.vertices_size(); ++i) {
          const interconnection::coordinate c{input_polygon_proto.vertices(i)};
          input_polygon.emplace_back(c.latitude(), c.longitude());
        }
        OLYSEUS_VERIFY(!input_polygon.empty());
        spdlog::info("Input polygon:");
        for (const lat_lon& p : input_polygon) {
          spdlog::info("  ({}, {})", p.latitude(), p.longitude());
        }

        const interconnection::coordinate c_home{input_polygon_proto.home()};
        const lat_lon home{c_home.latitude(), c_home.longitude()};
        std::vector<lat_lon> mission_path{
            mission_builder::make(input_polygon, home)};

        spdlog::info("Mission path:");
        for (const lat_lon& p : mission_path) {
          spdlog::info("  ({}, {})", p.latitude(), p.longitude());
        }

        mission_.mission_path_ready(std::move(mission_path), event_id);
      } break;
      case interconnection::command_type::MISSION_PATH_CANCEL: {
        mission_.mission_path_cancel(receive_event_id());
      } break;
      case interconnection::command_type::MISSION_START: {
        mission_.init();

        const bool start_ok{
            mission_.upload_mission_and_start(receive_event_id())};
        OLYSEUS_VERIFY(start_ok);

        home_altitude_.mission_start();
      } break;
      case interconnection::command_type::MISSION_PAUSE: {
        const int32_t event_id{receive_event_id()};
        while (true) {
          if (!mission_.user_cmd_accepted()) {
            spdlog::info("Waiting for stable state...");
            constexpr int sleep_sec{1};
            std::this_thread::sleep_for(std::chrono::seconds{sleep_sec});
            continue;
          }

          const std::lock_guard lock{action_job_mutex_};
          if (!mission_.user_cmd_accepted()) {
            continue;
          }

          if (mission_.is_ready()) {
            mission_.update_event_id(event_id);
          } else {
            mission_.pause(event_id);
          }
          break;
        }
      } break;
      case interconnection::command_type::MISSION_CONTINUE:
        mission_.resume(receive_event_id());
        break;
      case interconnection::command_type::MISSION_ABORT: {
        const int32_t event_id{receive_event_id()};
        while (true) {
          if (!mission_.user_cmd_accepted()) {
            spdlog::info("Waiting for stable state...");
            constexpr int sleep_sec{1};
            std::this_thread::sleep_for(std::chrono::seconds{sleep_sec});
            continue;
          }

          const std::lock_guard lock{action_job_mutex_};
          if (!mission_.user_cmd_accepted()) {
            continue;
          }

          if (mission_.is_ready()) {
            mission_.update_event_id(event_id);
          } else {
            mission_.abort(event_id);
            mission_.stop(home_altitude_);
          }
          break;
        }
      } break;
      case interconnection::command_type::LASER_RANGE_RESPONSE: {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
        OLYSEUS_UNREACHABLE;
#else
        buffer.resize(receive_next_packet_size());
        receive_data(&buffer);

        interconnection::laser_range laser_range;
        const bool ok{laser_range.ParseFromString(buffer)};
        OLYSEUS_VERIFY(ok);

        laser_range_.value_received(laser_range.range());
#endif
      } break;
      default:
        spdlog::error("Unexpected command type: {}", command.type());
        OLYSEUS_UNREACHABLE;
    }
  }
}

void drone::send_data_job() {
  try {
    send_data_job_internal();
  } catch (job_interrupted_event&) {
    OLYSEUS_VERIFY(interrupt_condition());
  } catch (pipeline_closed&) {
    connection_closed_ = true;
  } catch (std::exception& e) {
    exception_caught_ = true;
    spdlog::critical("Exception: {}", e.what());
    OLYSEUS_VERIFY(interrupt_condition());
  }
}

void drone::send_data_job_internal() {
  spdlog::info("Send data job started");

  std::optional<interconnection::command_type::command_t> command;

  while (true) {
    command.reset();

    {
      const std::lock_guard lock{execute_commands_mutex_};
      if (!execute_commands_.empty()) {
        command = execute_commands_.front();
      }
    }

    if (!command.has_value()) {
      constexpr int wait_ms{200};
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
      const std::lock_guard lock{execute_commands_mutex_};
      execute_commands_.push_back(interconnection::command_type::DRONE_INFO);
      continue;
    }

    send_command(command.value());

    switch (command.value()) {
      case interconnection::command_type::PONG:
        spdlog::info("PONG command sent");
        break;
      case interconnection::command_type::DRONE_INFO: {
        const auto state{send_drone_info()};
        if (state == interconnection::drone_info::PATH_DATA) {
          spdlog::info("Sending mission path for PATH_DATA");
          const std::vector<lat_lon> mission_path{mission_.get_mission_path()};

          interconnection::mission_path m_path;
          for (const lat_lon& p : mission_path) {
            interconnection::coordinate* w{m_path.add_waypoints()};
            OLYSEUS_VERIFY(w != nullptr);
            w->set_latitude(p.latitude());
            w->set_longitude(p.longitude());
          }

          std::string buffer;
          const bool ok{m_path.SerializeToString(&buffer)};
          OLYSEUS_VERIFY(ok);

          send_next_packet_size(buffer.size());
          send_data(buffer);

          send_command(interconnection::command_type::DRONE_INFO);
          const auto state_next{send_drone_info()};
          OLYSEUS_VERIFY(state_next == interconnection::drone_info::PATH);
        }
      } break;
      case interconnection::command_type::LASER_RANGE_REQUEST:
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
        OLYSEUS_UNREACHABLE;
#else
        spdlog::info("Laser range request sent");
#endif
        break;
      default:
        spdlog::error("Unexpected command: {}", command.value());
        OLYSEUS_UNREACHABLE;
    }

    {
      const std::lock_guard lock{execute_commands_mutex_};
      execute_commands_.pop_front();
    }
  }
}

void drone::send_command(
    interconnection::command_type::command_t command_type) {
  interconnection::command_type command;
  command.set_type(command_type);
  command.set_version(protocol_version);
  std::string buffer;
  const bool ok{command.SerializeToString(&buffer)};
  OLYSEUS_VERIFY(ok);

  send_next_packet_size(buffer.size());
  send_data(buffer);
}

void drone::receive_data(std::string* buffer) {
  OLYSEUS_VERIFY(buffer != nullptr);
  OLYSEUS_VERIFY(!buffer->empty());

  while (true) {
    if (interrupt_condition()) {
      throw job_interrupted_event();
    }
    if (connection_closed_) {
      throw pipeline_closed();
    }

    uint32_t real_len{0};

#if defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
    char* char_buffer{buffer->data()};
    static_assert(sizeof(char) == sizeof(uint8_t));
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    uint8_t* recv_buf{reinterpret_cast<uint8_t*>(char_buffer)};

    OLYSEUS_VERIFY(channel_handle_ != nullptr);
    const api_code code{DjiMopChannel_RecvData(channel_handle_, recv_buf,
                                               buffer->size(), &real_len)};
#elif defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
    OLYSEUS_VERIFY(channel_handle_ == nullptr);
    const api_code code{simulator_.receive_data(buffer)};
    real_len = buffer->size();
#else
#error "Invalid configuration"
#endif

    if (code.retry()) {
      continue;
    }
    OLYSEUS_VERIFY(code.success());
    OLYSEUS_VERIFY(real_len == buffer->size());
    spdlog::debug("{} bytes received", real_len);
    return;
  }
}

auto drone::receive_next_packet_size() -> uint32_t {
  std::string buffer;
  buffer.resize(packet_size_);

  receive_data(&buffer);

  interconnection::packet_size p_size;
  const bool ok{p_size.ParseFromString(buffer)};
  OLYSEUS_VERIFY(ok);

  return p_size.size();
}

void drone::send_data(std::string& buffer) {
  OLYSEUS_VERIFY(!buffer.empty());

  while (true) {
    if (interrupt_condition()) {
      throw job_interrupted_event();
    }
    if (connection_closed_) {
      throw pipeline_closed();
    }

#if defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
    char* char_buffer{buffer.data()};
    static_assert(sizeof(char) == sizeof(uint8_t));
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    uint8_t* send_buf{reinterpret_cast<uint8_t*>(char_buffer)};

    uint32_t real_len{0};
    OLYSEUS_VERIFY(channel_handle_ != nullptr);
    const api_code code{DjiMopChannel_SendData(channel_handle_, send_buf,
                                               buffer.size(), &real_len)};
    if (code.retry()) {
      continue;
    }
    OLYSEUS_VERIFY(code.success());
    OLYSEUS_VERIFY(real_len == buffer.size());
    spdlog::debug("{} bytes sent", real_len);
    return;
#else
    // Act like data was successfully sent
    return;
#endif
  }
}

void drone::send_next_packet_size(uint32_t size) {
  interconnection::packet_size p_size;
  p_size.set_size(size);
  std::string buffer;
  const bool ok{p_size.SerializeToString(&buffer)};
  OLYSEUS_VERIFY(ok);
  OLYSEUS_VERIFY(buffer.size() == packet_size_);
  send_data(buffer);
}

auto drone::receive_event_id() -> int32_t {
  std::string buffer;
  buffer.resize(receive_next_packet_size());

  receive_data(&buffer);

  interconnection::event_id_message event_id;
  const bool ok{event_id.ParseFromString(buffer)};
  OLYSEUS_VERIFY(ok);

  const int32_t res{event_id.event_id()};
  OLYSEUS_VERIFY(res != mission_state::internal_event_id);
  OLYSEUS_VERIFY(res > 0);
  return res;
}

auto drone::send_drone_info() -> interconnection::drone_info::state_t {
  interconnection::drone_info d_info;
  d_info.set_latitude(drone_latitude_);
  d_info.set_longitude(drone_longitude_);
  d_info.set_heading(drone_yaw_);

  auto [event_id, state] = mission_.get_state();
  d_info.set_state(state);
  d_info.set_event_id(event_id);

  std::string buffer;
  const bool ok{d_info.SerializeToString(&buffer)};
  OLYSEUS_VERIFY(ok);

  send_next_packet_size(buffer.size());
  send_data(buffer);

  spdlog::debug("Drone coordinates sent: lat:{}, lon:{}, head:{}, state:{}",
                drone_latitude_, drone_longitude_, drone_yaw_, state);

  return state;
}

void drone::next_mission() {
  if (!mission_.is_forward()) {
    spdlog::info("MISSION FINISHED");
    mission_.stop(home_altitude_);
    return;
  }

  // Forward mission is finished and we can run backward mission
  while (!camera_.queue_is_empty()) {
    spdlog::info("Wait for inference to finish...");
    constexpr int sleep_sec{5};
    std::this_thread::sleep_for(std::chrono::seconds{sleep_sec});
  }

  spdlog::info("Start backward mission");

  mission_.set_backward();
  constexpr int32_t event_id{mission_state::internal_event_id};
  if (!mission_.upload_mission_and_start(event_id)) {
    spdlog::info("Backward mission canceled: No objects detected");
    mission_.stop(home_altitude_);
  }
}

auto drone::latest_laser_range() -> float {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  return simulator_.laser_range();
#else
  return laser_range_.latest(execute_commands_mutex_, execute_commands_);
#endif
}
