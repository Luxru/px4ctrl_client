#pragma once

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include "json.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace px4ctrl {
namespace ui {

struct ServerPayload {
  uint8_t id;
  uint64_t timestamp;
  float pos[3];
  float vel[3];
  float omega[3];
  float quat[4];

  float thrust_setpoint;
  float omega_setpoint[3];

  float battery_voltage;
  int32_t mission_phase;
  int32_t offboard_state;
  int32_t armed_state;

  float thrust_map[3];
  float hover_pos[3];
  float hover_quat[4];
  float odom_hz;
  float cmdctrl_hz;

  uint32_t telemetry_seq;
  uint32_t guard_flags;
  float odom_age_ms;
  float client_cmd_age_ms;
  float battery_remaining;
  float speed_norm;
  float tilt_deg;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float cmd_age_ms;     // age of external ctrl_command stream
  float omega_min;      // min bodyrate for plot scaling
  float omega_max;      // max bodyrate for plot scaling

  float geofence_min[3];
  float geofence_max[3];
  float max_roll_deg;
  float max_pitch_deg;
  float max_yaw_deg;
  uint8_t enable_geofence;
  uint8_t enable_attitude_fence;
  uint8_t use_rc;
  uint8_t reserved0;
};

enum class MissionPhase {
  STANDBY,
  TAKEOFF,
  HOVER,
  CMD_CTRL,
  LANDING,
  FAILSAFE,
};

static constexpr const char *MissionPhaseName[] = {
    "STANDBY", "TAKEOFF", "HOVER", "CMD_CTRL", "LANDING", "FAILSAFE",
};

enum class ClientCommand : uint32_t {
  HEARTBEAT,
  ARM,
  ENTER_OFFBOARD,
  EXIT_OFFBOARD,
  TAKEOFF,
  LAND,
  FORCE_HOVER,
  ALLOW_CMD_CTRL,
  FORCE_DISARM,
  CHANGE_HOVER_POS,
  SET_SAFETY_LIMITS,
};

static constexpr const char *CommandStr[] = {
    "HEARTBEAT",      "ARM",           "ENTER_OFFBOARD",
    "EXIT_OFFBOARD",  "TAKEOFF",       "LAND",
    "FORCE_HOVER",    "ALLOW_CMD_CTRL", "FORCE_DISARM",
    "CHANGE_HOVER_POS", "SET_SAFETY_LIMITS",
};

struct SafetyLimitsPayload {
  float geofence_min[3];
  float geofence_max[3];
  float max_roll_deg;
  float max_pitch_deg;
  float max_yaw_deg;
  uint8_t enable_geofence;
  uint8_t enable_attitude_fence;
  uint8_t reserved[2];
};

struct ClientPayload {
  uint8_t id;
  uint64_t timestamp;
  ClientCommand command;
  uint8_t data[64];
};

static_assert(std::is_trivially_copyable_v<ServerPayload>,
              "ServerPayload must be trivially copyable for wire transport");
static_assert(std::is_trivially_copyable_v<ClientPayload>,
              "ClientPayload must be trivially copyable for wire transport");
static_assert(sizeof(SafetyLimitsPayload) <= sizeof(ClientPayload::data),
              "SafetyLimitsPayload exceeds client payload data area");
static_assert(sizeof(ClientCommand) == sizeof(uint32_t),
              "ClientCommand wire size must stay 4 bytes");
static_assert(sizeof(ServerPayload) == 240,
              "ServerPayload wire size changed; update client/server together");
static_assert(sizeof(ClientPayload) == 88,
              "ClientPayload wire size changed; update client/server together");

template <typename T>
inline void unpack_raw(const uint8_t *data, const size_t size, T &out) {
  if (size != sizeof(T)) {
    throw std::runtime_error("Message size does not match");
  }
  std::memcpy(&out, data, sizeof(T));
}

enum class CommBackend {
  ZENOH,
};

inline CommBackend backendFromString(const std::string &backend) {
  if (backend == "zenoh" || backend == "ZENOH") {
    return CommBackend::ZENOH;
  }
  throw std::runtime_error("Invalid backend: " + backend + " (expected zenoh)");
}

struct TransportParas {
  CommBackend backend = CommBackend::ZENOH;

  std::string server_topic = "px4s";
  std::string client_topic = "px4c";
  std::string log_topic = "px4log";
  uint32_t telemetry_hz = 200;

  std::string zenoh_mode = "peer";
  std::string zenoh_connect;
  std::string zenoh_listen;
  bool zenoh_multicast_scouting = true;
  uint32_t zenoh_scouting_timeout_ms = 1000;

  // keyboard control defaults (can be adjusted online in ImGui)
  float keyboard_vel_xy = 1.0F;  // m/s
  float keyboard_vel_z = 0.2F;   // m/s
  float keyboard_vel_yaw = 2.0F; // rad/s

  inline static TransportParas load(const std::string &file) {
    TransportParas paras;
    const auto ext = std::filesystem::path(file).extension().string();
    if (ext != ".json") {
      throw std::runtime_error("Unsupported config extension: " + ext +
                               " (expected .json)");
    }

    std::ifstream ifs(file);
    if (!ifs.is_open()) {
      throw std::runtime_error("Failed to open config file: " + file);
    }

    try {
      nlohmann::json config;
      ifs >> config;

      if (config.contains("backend")) {
        paras.backend =
            backendFromString(config.at("backend").get<std::string>());
      }
      paras.server_topic = config.value("server_topic", paras.server_topic);
      paras.client_topic = config.value("client_topic", paras.client_topic);
      paras.log_topic = config.value("log_topic", paras.log_topic);
      paras.telemetry_hz = config.value("telemetry_hz", paras.telemetry_hz);

      if (config.contains("zenoh")) {
        const auto &z = config.at("zenoh");
        paras.zenoh_mode = z.value("mode", paras.zenoh_mode);
        paras.zenoh_connect = z.value("connect", paras.zenoh_connect);
        paras.zenoh_listen = z.value("listen", paras.zenoh_listen);
        paras.zenoh_multicast_scouting =
            z.value("multicast_scouting", paras.zenoh_multicast_scouting);
        paras.zenoh_scouting_timeout_ms =
            z.value("scouting_timeout_ms", paras.zenoh_scouting_timeout_ms);
      }

      if (config.contains("keyboard")) {
        const auto &k = config.at("keyboard");
        paras.keyboard_vel_xy = k.value("vel_xy", paras.keyboard_vel_xy);
        paras.keyboard_vel_z = k.value("vel_z", paras.keyboard_vel_z);
        paras.keyboard_vel_yaw = k.value("vel_yaw", paras.keyboard_vel_yaw);
      }
    } catch (const nlohmann::json::exception &e) {
      spdlog::error("error: {}", e.what());
      throw;
    }
    return paras;
  }
};

} // namespace ui
} // namespace px4ctrl
