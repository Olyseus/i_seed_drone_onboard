#include "drone.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>

#include <future>

#include <dji_fc_subscription.h> // T_DjiFcSubscriptionQuaternion
#include <dji_gimbal_manager.h> // DjiGimbalManager_Init
#include <dji_mop_channel.h> // DjiMopChannel_Init

#include "api_code.h"
#include "converter.h"
#include "server.h"
#include "utils.h" // rad2deg

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
simulator drone::simulator_;
#endif

std::atomic<bool> drone::sigint_received_{false};

std::atomic<double> drone::drone_yaw_{0.0};
std::atomic<double> drone::drone_pitch_{0.0};
std::atomic<double> drone::drone_roll_{0.0};
std::atomic<double> drone::drone_longitude_{0.0};
std::atomic<double> drone::drone_latitude_{0.0};
std::atomic<double> drone::drone_altitude_{0.0};
std::atomic<double> drone::gimbal_yaw_;
std::atomic<double> drone::gimbal_pitch_;
std::atomic<double> drone::gimbal_roll_;
std::atomic<double> drone::homepoint_altitude_{invalid_homepoint_altitude_};
std::atomic<int16_t> drone::rc_mode_{-1};

mission_state drone::mission_state_;
std::mutex drone::execute_commands_mutex_;
std::list<interconnection::command_type::command_t> drone::execute_commands_;

std::mutex drone::action_mutex_;
std::condition_variable drone::action_condition_variable_;
bool drone::run_action_{false};

T_DjiReturnCode drone::quaternion_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto quaternion{*(const T_DjiFcSubscriptionQuaternion*)data};
  (void)data_size;
  (void)timestamp;

  // https://github.com/dji-sdk/Onboard-SDK/blob/2c38de17f7aad0064056f27eaa219d4ed30ab82a/sample/platform/STM32/OnBoardSDK_STM32/User/FlightControlSample.cpp#L800-L824
  const double q2sqr{quaternion.q2 * quaternion.q2};
  const double t0{-2.0 * (q2sqr + quaternion.q3 * quaternion.q3) + 1.0};
  const double t1{+2.0 * (quaternion.q1 * quaternion.q2 + quaternion.q0 * quaternion.q3)};
  double t2{-2.0 * (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2)};
  const double t3{+2.0 * (quaternion.q2 * quaternion.q3 + quaternion.q0 * quaternion.q1)};
  const double t4{-2.0 * (quaternion.q1 * quaternion.q1 + q2sqr) + 1.0};

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  // https://sdk-forum.dji.net/hc/en-us/requests/74003
  // https://sdk-forum.dji.net/hc/en-us/articles/360023657273
  drone_roll_ = atan2(t3, t4) * rad2deg; // X
  drone_pitch_ = asin(t2) * rad2deg; // Y
  drone_yaw_ = atan2(t1, t0) * rad2deg; // Z

  spdlog::debug("roll: {}, pitch: {}, yaw: {}", drone_roll_, drone_pitch_, drone_yaw_);

  BOOST_VERIFY(drone_yaw_ >= -180.0);
  BOOST_VERIFY(drone_yaw_ <= 180.0);
  BOOST_VERIFY(drone_pitch_ > -90.0);
  BOOST_VERIFY(drone_pitch_ < 90.0);
  BOOST_VERIFY(drone_roll_ > -90.0);
  BOOST_VERIFY(drone_roll_ < 90.0);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::rc_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto rc{*(const T_DjiFcSubscriptionRC*)data};
  (void)data_size;
  (void)timestamp;

  rc_mode_ = rc.mode;

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::position_fused_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto position{*(const T_DjiFcSubscriptionPositionFused*)data};
  (void)data_size;
  (void)timestamp;

  drone_latitude_ = position.latitude * rad2deg;
  drone_longitude_ = position.longitude * rad2deg;
  drone_altitude_ = position.altitude;

  spdlog::debug("drone latitude: {}, longitude: {}, altitude: {}", drone_latitude_, drone_longitude_, drone_altitude_);

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  simulator_.gps_callback(drone_latitude_, drone_longitude_);
#endif

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::gimbal_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto gimbal_three_data{(const T_DjiFcSubscriptionThreeGimbalData*)data};
  const GimbalSingleData d{gimbal_three_data->gbData[0]};

  (void)data_size;
  (void)timestamp;

  gimbal_yaw_ = d.yaw;
  gimbal_pitch_ = d.pitch;
  gimbal_roll_ = d.roll;

  spdlog::debug("gimbal pitch: {}, roll: {}, yaw: {}", gimbal_pitch_, gimbal_roll_, gimbal_yaw_);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::mission_event_callback(T_DjiWaypointV2MissionEventPush event_data) {
  if (!mission_state_.is_started()) {
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
  }

  mission_state_.update(event_data);

  if (!mission_state_.is_started()) {
    std::lock_guard<std::mutex> lock(execute_commands_mutex_);
    execute_commands_.push_back(
        interconnection::command_type::MISSION_FINISHED);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::mission_state_callback(T_DjiWaypointV2MissionStatePush state_data) {
  if (!mission_state_.is_started()) {
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
  }

  const bool run_action{mission_state_.update(state_data)};

  if (run_action) {
    spdlog::info("RUN ACTION FOR WAYPOINT");
    {
      std::lock_guard lock(action_mutex_);
      BOOST_VERIFY(!run_action_);
      run_action_ = true;
    }
    action_condition_variable_.notify_one();
  }

  if (!mission_state_.is_started()) {
    BOOST_VERIFY(!run_action);
    std::lock_guard<std::mutex> lock(execute_commands_mutex_);
    execute_commands_.push_back(
        interconnection::command_type::MISSION_FINISHED);
  }

  // https://developer.dji.com/onboard-api-reference/structDJI_1_1OSDK_1_1Telemetry_1_1RC.html#a9e69e1b32599986319ad3312ca5723de
  if (rc_mode_ != 8000) {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
    BOOST_VERIFY(rc_mode_ == -8000);
#else
    // Value received while running tests on simulator
    spdlog::error("Unexpected RC mode: {}", rc_mode_);
    BOOST_VERIFY(false);
#endif
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::homepoint_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto altitude{*(const T_DjiFcSubscriptionAltitudeOfHomePoint*)data};
  (void)data_size;
  (void)timestamp;

  homepoint_altitude_ = altitude;
  BOOST_VERIFY(homepoint_altitude_ > invalid_homepoint_altitude_);

  spdlog::debug("home altitude: {}", homepoint_altitude_);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

drone::drone() :
    mission_(mission_state_),
    camera_psdk_("/var/opt/i_seed_drone_onboard/best.engine", mission_)
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
    , laser_range_(simulator_)
#endif
    {
  BOOST_VERIFY(sigint_received_.is_lock_free());

  constexpr E_DjiDataSubscriptionTopicFreq topic_freq{DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ};

  T_DjiReturnCode code{DjiFcSubscription_Init()};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, topic_freq,
      quaternion_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_RC, topic_freq,
      rc_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, topic_freq,
      position_fused_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA,
      topic_freq,
      gimbal_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // https://sdk-forum.dji.net/hc/en-us/requests/76593
  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
      DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
      homepoint_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiGimbalManager_Init();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiGimbalManager_SetMode(m_pos, DJI_GIMBAL_MODE_FREE);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_Init();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_RegisterMissionEventCallback(mission_event_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_RegisterMissionStateCallback(mission_state_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  interconnection::command_type command;
  command.set_type(interconnection::command_type::PING);
  command.set_version(protocol_version);
  std::string buffer_1;
  bool ok{command.SerializeToString(&buffer_1)};
  BOOST_VERIFY(ok);
  command_bytes_size_ = {static_cast<uint32_t>(buffer_1.size())};
  BOOST_VERIFY(command_bytes_size_ > 0);

  interconnection::pin_coordinates pin_coordinates;
  pin_coordinates.set_latitude(0.0);
  pin_coordinates.set_longitude(0.0);
  std::string buffer_2;
  ok = pin_coordinates.SerializeToString(&buffer_2);
  BOOST_VERIFY(ok);
  pin_coordinates_bytes_size_ = {static_cast<uint32_t>(buffer_2.size())};
  BOOST_VERIFY(pin_coordinates_bytes_size_ > 0);

  {
    interconnection::laser_range laser_range;
    laser_range.set_range(0.0);
    std::string buffer;
    const bool ok{laser_range.SerializeToString(&buffer)};
    BOOST_VERIFY(ok);
    laser_range_bytes_size_ = static_cast<uint32_t>(buffer.size());
    BOOST_VERIFY(laser_range_bytes_size_ > 0);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void drone::sigint_handler(int signal) {
  (void)signal;
  spdlog::critical("SIGINT received");
  sigint_received_ = true;
}

drone::~drone() {
  T_DjiReturnCode code{DjiFcSubscription_DeInit()};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_Deinit();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

void drone::start() {
  spdlog::info("Protocol version: {}", protocol_version);
  spdlog::info("Command bytes size: {}", command_bytes_size_);

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  spdlog::info("SIMULATOR MODE");
#endif

  T_DjiReturnCode code = DjiMopChannel_Init();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  std::thread action_thread{&drone::action_job, this};
  std::thread inference_thread{&drone::inference_job, this};

  while (!interrupt_condition()) {
    BOOST_VERIFY(signal(SIGINT, SIG_DFL) != SIG_ERR);
    server server{channel_id};
    channel_handle_ = server.handle();
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
    BOOST_VERIFY(channel_handle_ == nullptr);
#else
    BOOST_VERIFY(channel_handle_ != nullptr);
#endif
    BOOST_VERIFY(signal(SIGINT, sigint_handler) != SIG_ERR);

    connection_closed_ = false;

    std::thread receive_data_thread{&drone::receive_data_job, this};
    std::thread send_data_thread{&drone::send_data_job, this};

    receive_data_thread.join();
    send_data_thread.join();
  }

  // The main loop is endless so we reach this point only in case of error

  // Wake up all the threads that can possible wait for been notified
  {
    std::lock_guard lock{action_mutex_};
    run_action_ = true;
  }
  action_condition_variable_.notify_one();
  action_thread.join();

  // Wait for other threads to finish. No need to nofify
  inference_thread.join();

  if (sigint_received_) {
    throw std::runtime_error("SIGINT received");
  }

  BOOST_VERIFY(exception_caught_);
  throw std::runtime_error("Exception in thread");
}

auto drone::interrupt_condition() const -> bool {
  return exception_caught_ || sigint_received_;
}

void drone::action_job() {
  try {
    while (true) {
      std::unique_lock lock{action_mutex_};
      action_condition_variable_.wait(lock, [this]{
          return run_action_;
      });
      BOOST_VERIFY(run_action_);

      if (interrupt_condition()) {
        spdlog::critical("Action job exit");
        return;
      }
      action_job_internal();
    }
  } catch (std::exception& e) {
    exception_caught_ = true;
    spdlog::critical("Exception: {}", e.what());
  }
}

void drone::action_job_internal() {
  spdlog::info("Pause mission");
  T_DjiReturnCode code{DjiWaypointV2_Pause()};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // Wait for drone to finish the movement
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::size_t waypoint_index{0};

  switch (mission_.waypoint_reached(laser_range_.latest(), &waypoint_index)) {
    case waypoint_action::ok:
      break;
    case waypoint_action::abort:
      spdlog::info("Abort mission on a last fake waypoint");
      abort_mission();
      {
        std::lock_guard<std::mutex> lock(execute_commands_mutex_);
        execute_commands_.push_back(
            interconnection::command_type::MISSION_FINISHED);
      }
      return;
    case waypoint_action::restart:
      BOOST_VERIFY(mission_.is_forward());
      abort_mission();
      mission_.upload_mission_and_start();
      return;
    default:
      BOOST_VERIFY(false);
  }

  if (mission_.is_forward()) {
    align_gimbal();
  }

  spdlog::info("drone latitude: {}, longitude: {}, altitude: {}", drone_latitude_, drone_longitude_, drone_altitude_);
  spdlog::info("drone roll: {}, pitch: {}, yaw: {}", drone_roll_, drone_pitch_, drone_yaw_);
  spdlog::info("gimbal pitch: {}, roll: {}, yaw: {}", gimbal_pitch_, gimbal_roll_, gimbal_yaw_);

  waypoint w{mission_.get_waypoint_copy(waypoint_index)};

  BOOST_VERIFY(std::abs(w.lat() - drone_latitude_) < 1e-4);
  BOOST_VERIFY(std::abs(w.lon() - drone_longitude_) < 1e-4);

  BOOST_VERIFY(homepoint_altitude_ > invalid_homepoint_altitude_);
  home_altitude_.set_altitude(drone_altitude_, w.altitude(), homepoint_altitude_);

  if (mission_.is_forward()) {
    gps_coordinates gps;
    gps.longitude = drone_longitude_;
    gps.latitude = drone_latitude_;
    gps.altitude = drone_altitude_;
    gps.relative_altitude = w.altitude();

    attitude drone_attitude;
    drone_attitude.pitch = drone_pitch_;
    drone_attitude.roll = drone_roll_;
    drone_attitude.yaw = drone_yaw_;

    attitude gimbal_attitude;
    gimbal_attitude.pitch = gimbal_pitch_;
    gimbal_attitude.roll = gimbal_roll_;
    gimbal_attitude.yaw = gimbal_yaw_;

    const double yaw_diff{std::abs(drone_yaw_ - gimbal_yaw_)};
    spdlog::info("Gimbal/drone yaw diff: {}", yaw_diff);
    BOOST_VERIFY(yaw_diff < 0.7);
    camera_psdk_.shoot_photo(gps, drone_attitude, gimbal_attitude, waypoint_index);
  }
  else {
    BOOST_VERIFY(w.has_detection());
    detection_result d{w.get_detection()};
    BOOST_VERIFY(!d.pixels.empty());

    for (const detected_pixel& p : d.pixels) {
      attitude pixel_gimbal_attitude{rotate_gimbal(p.x, p.y, drone_yaw_)};
      pixel_gimbal_attitude.yaw -= drone_yaw_; // yaw relative to drone

      spdlog::info("drone latitude: {}, longitude: {}, altitude: {}", drone_latitude_, drone_longitude_, drone_altitude_);
      spdlog::info("drone roll: {}, pitch: {}, yaw: {}", drone_roll_, drone_pitch_, drone_yaw_);
      spdlog::info("gimbal pitch: {}, roll: {}, yaw: {}", gimbal_pitch_, gimbal_roll_, gimbal_yaw_);

      gps_coordinates gps;
      gps.longitude = drone_longitude_;
      gps.latitude = drone_latitude_;
      gps.altitude = drone_altitude_;
      gps.relative_altitude = 0.0; // not used

      attitude drone_attitude;
      drone_attitude.pitch = drone_pitch_;
      drone_attitude.roll = drone_roll_;
      drone_attitude.yaw = drone_yaw_;

      attitude laser_gimbal_attitude;
      laser_gimbal_attitude.pitch = gimbal_pitch_;
      laser_gimbal_attitude.roll = gimbal_roll_;
      laser_gimbal_attitude.yaw = gimbal_yaw_ - drone_yaw_; // yaw relative to drone

      const converter_result pixel_result{converter::run(d.gps, d.drone_attitude, pixel_gimbal_attitude, 1.0)};
      const converter_result laser_result{converter::run(gps, drone_attitude, laser_gimbal_attitude, laser_range_.latest())};

      BOOST_VERIFY((pixel_result.p - laser_result.p).norm() < 5.0);
      BOOST_VERIFY(std::abs(pixel_result.d.dot(laser_result.d) - 1.0) < 1e-3);

      const Eigen::Vector3d p_laser_end{laser_result.p + laser_result.v};
      spdlog::info("{} {} {} 255 165 0", p_laser_end(0), p_laser_end(1), p_laser_end(2)); // orange

      const double k_num{laser_result.d.dot(laser_result.p - pixel_result.p + laser_result.v)};
      const double k_denom{pixel_result.v.dot(laser_result.d)};
      BOOST_VERIFY(k_denom > 1e-2);
      const double k{k_num / k_denom};
      BOOST_VERIFY(k > 1e-3);

      const Eigen::Vector3d p_pixel_end{pixel_result.p + k * pixel_result.v};
      spdlog::info("{} {} {} 0 255 0", p_pixel_end(0), p_pixel_end(1), p_pixel_end(2));
      BOOST_VERIFY((p_pixel_end - p_laser_end).norm() < 10.0);

      // FIXME (save to file)
    }
  }

  spdlog::info("Resume mission");
  code = DjiWaypointV2_Resume();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // mark as processed
  run_action_ = false;
}

void drone::align_gimbal() {
  constexpr double expected_gimbal_pitch{-90.0}; // down

  constexpr int time_ms{500};
  constexpr int time_wait_ms{3 * time_ms};

  constexpr double expected_gimbal_roll{0.0};
  const double expected_gimbal_yaw{drone_yaw_};

  T_DjiGimbalManagerRotation rotation;
  rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
  rotation.pitch = expected_gimbal_pitch - gimbal_pitch_;
  rotation.roll = expected_gimbal_roll - gimbal_roll_;
  rotation.yaw = expected_gimbal_yaw - gimbal_yaw_;
  rotation.time = time_ms / 1000.0;

  constexpr float eps{1e-3};
  constexpr float rough_eps{0.1 + eps};
  const double d_roll{std::abs(rotation.roll)};
  const double d_pitch{std::abs(rotation.pitch)};
  const double d_yaw{std::abs(rotation.yaw)};
  if (d_roll < rough_eps && d_pitch < rough_eps && d_yaw < rough_eps) {
    return;
  }

  spdlog::info("Current gimbal yaw: {}, expected: {}", gimbal_yaw_, expected_gimbal_yaw);
  spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}", rotation.yaw, rotation.roll, rotation.pitch);

  const T_DjiReturnCode code{DjiGimbalManager_Rotate(m_pos, rotation)};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));

  constexpr double expected_eps{0.3};
  const double d_yaw_2{expected_gimbal_yaw - gimbal_yaw_};

  if (std::abs(d_yaw_2) > expected_eps) {
    spdlog::info("Gimbal rotation failed, trying another direction, diff: {}", d_yaw_2);

    T_DjiGimbalManagerRotation rotation;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.pitch = 0.0;
    rotation.roll = 0.0;
    rotation.yaw = (d_yaw_2 > 0.0) ? -178.0 : 178.0;
    rotation.time = time_ms / 1000.0;

    spdlog::info("Current gimbal yaw: {}", gimbal_yaw_);
    spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}", rotation.yaw, rotation.roll, rotation.pitch);

    T_DjiReturnCode code{DjiGimbalManager_Rotate(m_pos, rotation)};
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));

    const double expected_gimbal_yaw{drone_yaw_};

    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.pitch = expected_gimbal_pitch - gimbal_pitch_;
    rotation.roll = expected_gimbal_roll - gimbal_roll_;
    rotation.yaw = expected_gimbal_yaw - gimbal_yaw_;
    rotation.time = time_ms / 1000.0;

    spdlog::info("Current gimbal yaw: {}", gimbal_yaw_);
    spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}", rotation.yaw, rotation.roll, rotation.pitch);

    code = DjiGimbalManager_Rotate(m_pos, rotation);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));
  }
}

auto drone::rotate_gimbal(float x, float y, double drone_heading_degree) -> attitude {
  spdlog::info("Gimbal rotate to pixel {}, {}", x, y);

  attitude expected;
  expected.roll = 0.0;
  std::tie(expected.yaw, expected.pitch) = gimbal_rotation_params(x, y, drone_heading_degree);

  constexpr int time_ms{500};
  constexpr int time_wait_ms{3 * time_ms};

  T_DjiGimbalManagerRotation rotation;
  rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
  rotation.pitch = expected.pitch - gimbal_pitch_;
  rotation.roll = expected.roll - gimbal_roll_;
  rotation.yaw = expected.yaw - gimbal_yaw_;
  rotation.time = time_ms / 1000.0;

  spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}", rotation.yaw, rotation.roll, rotation.pitch);

  const T_DjiReturnCode code{DjiGimbalManager_Rotate(m_pos, rotation)};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));

  constexpr double expected_eps{0.3};
  const double d_roll{expected.roll - gimbal_roll_};
  const double d_pitch{expected.pitch - gimbal_pitch_};
  const double d_yaw{expected.yaw - gimbal_yaw_};

  if (std::abs(d_roll) > expected_eps || std::abs(d_pitch) > expected_eps || std::abs(d_yaw) > expected_eps) {
    spdlog::info("Gimbal rotation failed, trying another direction, diff roll: {}, pitch: {}, yaw: {}", d_roll, d_pitch, d_yaw);

    T_DjiGimbalManagerRotation rotation;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.pitch = 0.0;
    rotation.roll = 0.0;
    rotation.yaw = (d_yaw > 0.0) ? -178.0 : 178.0;
    rotation.time = time_ms / 1000.0;

    spdlog::info("Current gimbal yaw: {}", gimbal_yaw_);
    spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}", rotation.yaw, rotation.roll, rotation.pitch);

    T_DjiReturnCode code{DjiGimbalManager_Rotate(m_pos, rotation)};
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_ms));

    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE;
    rotation.pitch = expected.pitch - gimbal_pitch_;
    rotation.roll = expected.roll - gimbal_roll_;
    rotation.yaw = expected.yaw - gimbal_yaw_;
    rotation.time = time_ms / 1000.0;

    spdlog::info("Run gimbal rotation, yaw: {}, roll: {}, pitch: {}", rotation.yaw, rotation.roll, rotation.pitch);

    code = DjiGimbalManager_Rotate(m_pos, rotation);
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

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
      camera_psdk_.check_sdcard();
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
    BOOST_VERIFY(interrupt_condition());
  } catch (pipeline_closed&) {
    connection_closed_ = true;
  } catch (std::exception& e) {
    exception_caught_ = true;
    spdlog::critical("Exception: {}", e.what());
    BOOST_VERIFY(interrupt_condition());
  }
}

void drone::receive_data_job_internal() {
  spdlog::info("Received data job started");

  std::string buffer;
  buffer.resize(command_bytes_size_);

  while (true) {
    BOOST_VERIFY(command_bytes_size_ == buffer.size());
    receive_data(&buffer);

    interconnection::command_type command;
    const bool ok{command.ParseFromString(buffer)};
    BOOST_VERIFY(ok);

    switch (command.type()) {
      case interconnection::command_type::PING: {
        std::lock_guard<std::mutex> lock(execute_commands_mutex_);
        execute_commands_.push_back(command.type());
      } break;
      case interconnection::command_type::MISSION_START: {
        std::string buffer;
        buffer.resize(pin_coordinates_bytes_size_);
        receive_data(&buffer);

        if (mission_state_.is_started()) {
          spdlog::info("Mission resume");
          T_DjiReturnCode code{DjiWaypointV2_Resume()};
          BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
          // FIXME (verify mission state)
          continue;
        }

        interconnection::pin_coordinates pin_coordinates;
        const bool ok{pin_coordinates.ParseFromString(buffer)};
        BOOST_VERIFY(ok);

        mission_.init(pin_coordinates.latitude(), pin_coordinates.longitude());
        mission_.upload_mission_and_start();

        home_altitude_.mission_start();
        // FIXME (verify mission state)
      } break;
      case interconnection::command_type::MISSION_PAUSE: {
        spdlog::info("Mission pause");
        T_DjiReturnCode code{DjiWaypointV2_Pause()};
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        // FIXME (verify mission state)
      } break;
      case interconnection::command_type::MISSION_ABORT: {
        {
          std::lock_guard lock(action_mutex_);
          abort_mission();
        }
        home_altitude_.mission_stop();
        // FIXME (verify mission state)
      } break;
      case interconnection::command_type::LASER_RANGE: {
        std::string buffer;
        buffer.resize(laser_range_bytes_size_);
        receive_data(&buffer);

        interconnection::laser_range laser_range;
        const bool ok{laser_range.ParseFromString(buffer)};
        BOOST_VERIFY(ok);

        laser_range_.value_received(laser_range.range());
      } break;
      default:
        spdlog::error("Unexpected command type: {}", command.type());
        BOOST_VERIFY(false);
    }
  }
}

void drone::send_data_job() {
  try {
    send_data_job_internal();
  } catch (job_interrupted_event&) {
    BOOST_VERIFY(interrupt_condition());
  } catch (pipeline_closed&) {
    connection_closed_ = true;
  } catch (std::exception& e) {
    exception_caught_ = true;
    spdlog::critical("Exception: {}", e.what());
    BOOST_VERIFY(interrupt_condition());
  }
}

void drone::send_data_job_internal() {
  spdlog::info("Send data job started");

  std::optional<interconnection::command_type::command_t> command;

  while (true) {
    command.reset();

    {
      std::lock_guard<std::mutex> lock(execute_commands_mutex_);
      if (!execute_commands_.empty()) {
        command = execute_commands_.front();
      }
    }

    if (!command.has_value()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      std::lock_guard<std::mutex> lock(execute_commands_mutex_);
      execute_commands_.push_back(
          interconnection::command_type::DRONE_COORDINATES);
      continue;
    }

    switch (command.value()) {
      case interconnection::command_type::PING: {
        spdlog::info("Execute PING command");
        send_command(interconnection::command_type::PING);
        break;
      }
      case interconnection::command_type::MISSION_FINISHED: {
        if (mission_.is_forward()) {
          // Forward mission is finished and we can run backward mission
          while (!camera_psdk_.queue_is_empty()) {
            spdlog::info("Wait for inference to finish...");
            std::this_thread::sleep_for(std::chrono::seconds{5});
          }
          mission_.set_backward();
          mission_.upload_mission_and_start();
        }
        else {
          spdlog::info("Execute MISSION_FINISHED command");
          home_altitude_.mission_stop();
          send_command(interconnection::command_type::MISSION_FINISHED);
        }
      } break;
      case interconnection::command_type::DRONE_COORDINATES: {
        send_command(interconnection::command_type::DRONE_COORDINATES);

        interconnection::drone_coordinates dc;
        dc.set_latitude(drone_latitude_);
        dc.set_longitude(drone_longitude_);
        dc.set_heading(drone_yaw_);
        std::string buffer;
        const bool ok{dc.SerializeToString(&buffer)};
        BOOST_VERIFY(ok);

        send_data(buffer);

        spdlog::debug("Drone coordinates sent: lat:{}, lon:{}, head:{}",
                     drone_latitude_, drone_longitude_, drone_yaw_);
        break;
      }
      default:
        spdlog::error("Unexpected command: {}", command.value());
        BOOST_VERIFY(false);
    }

    {
      std::lock_guard<std::mutex> lock(execute_commands_mutex_);
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
  BOOST_VERIFY(ok);
  BOOST_VERIFY(buffer.size() == command_bytes_size_);

  send_data(buffer);
}

void drone::receive_data(std::string* buffer) {
  BOOST_VERIFY(buffer != nullptr);
  BOOST_VERIFY(buffer->size() > 0);

  while (true) {
    if (interrupt_condition()) {
      throw job_interrupted_event();
    }
    if (connection_closed_) {
      throw pipeline_closed();
    }

    uint32_t real_len{0};

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
    BOOST_VERIFY(channel_handle_ == nullptr);
    const api_code code{simulator_.receive_data(buffer)};
    real_len = buffer->size();
#else
    char* char_buffer{buffer->data()};
    static_assert(sizeof(char) == sizeof(uint8_t));
    uint8_t* recv_buf{reinterpret_cast<uint8_t*>(char_buffer)};

    BOOST_VERIFY(channel_handle_ != nullptr);
    const api_code code{DjiMopChannel_RecvData(channel_handle_, recv_buf, buffer->size(), &real_len)};
#endif

    if (code.retry()) {
      continue;
    }
    BOOST_VERIFY(code.success());
    BOOST_VERIFY(real_len == buffer->size());
    spdlog::debug("{} bytes received", real_len);
    return;
  }
}

void drone::send_data(std::string& buffer) {
  BOOST_VERIFY(buffer.size() > 0);

  while (true) {
    if (interrupt_condition()) {
      throw job_interrupted_event();
    }
    if (connection_closed_) {
      throw pipeline_closed();
    }

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
    // Act like data was successfully sent
    return;
#else
    char* char_buffer{buffer.data()};
    static_assert(sizeof(char) == sizeof(uint8_t));
    uint8_t* send_buf{reinterpret_cast<uint8_t*>(char_buffer)};

    uint32_t real_len{0};
    BOOST_VERIFY(channel_handle_ != nullptr);
    const api_code code{DjiMopChannel_SendData(channel_handle_, send_buf, buffer.size(), &real_len)};
    if (code.retry()) {
      continue;
    }
    BOOST_VERIFY(code.success());
    BOOST_VERIFY(real_len == buffer.size());
    spdlog::debug("{} bytes sent", real_len);
    return;
#endif
  }
}

void drone::abort_mission() {
  // Avoid condition trigger
  run_action_ = false;

  spdlog::info("Mission ABORT");

  // Call it first to block the update callbacks
  mission_state_.finish();

  T_DjiReturnCode code{DjiWaypointV2_Stop()};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}
