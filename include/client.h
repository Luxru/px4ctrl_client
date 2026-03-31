#pragma once

#if defined(__linux__) && !defined(ZENOH_LINUX)
#define ZENOH_LINUX 1
#endif

#include "datas.h"
#include "types.h"

#include <imgui.h>
#include <array>
#include <atomic>
#include <cfloat>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <spdlog/common.h>
#include <string>
#include <vector>
#include <zenoh.h>

namespace px4ctrl {
namespace ui {

class Px4Client {
public:
  struct LogEntry {
    int level = static_cast<int>(spdlog::level::info);
    std::string text;
  };

  explicit Px4Client(const TransportParas &paras);
  ~Px4Client();

  Px4Data<ServerPayload> server_data;
  Px4Data<LogEntry> log_data;
  void pub_client(const ClientPayload &payload);
  [[nodiscard]] const TransportParas &transport_paras() const { return paras_; }

private:
  TransportParas paras_;
  z_owned_session_t session_{};
  z_owned_publisher_t client_pub_{};
  z_owned_subscriber_t server_sub_{};
  z_owned_subscriber_t log_sub_{};

  std::atomic<bool> ok_{false};

  bool init_zenoh();
  void close_zenoh();

  static void server_sample_callback(z_loaned_sample_t *sample, void *context);
  static void log_sample_callback(z_loaned_sample_t *sample, void *context);
};

class ImguiClient {
public:
  explicit ImguiClient(Px4Client &px4_client);
  void render_window();

private:
  struct TelemetryHistory {
    std::deque<float> x;
    std::deque<float> y;
    std::deque<float> z;
    std::deque<ImVec2> xy_trace;

    std::deque<float> thrust;
    std::deque<float> omega_x;
    std::deque<float> omega_y;
    std::deque<float> omega_z;
    std::deque<ImVec2> omega_xy_trace;

    void push(const ServerPayload &p, size_t max_points);
  };

  struct SafetyEditorState {
    float geofence_min[3] = {-10.0F, -10.0F, -1.0F};
    float geofence_max[3] = {10.0F, 10.0F, 6.0F};
    float max_roll_deg = -1.0F;
    float max_pitch_deg = -1.0F;
    float max_yaw_deg = -1.0F;
    bool enable_geofence = false;
    bool enable_attitude_fence = false;
    bool initialized_from_telemetry = false;
  };

  std::vector<ClientCommand> command_vec_;
  std::map<uint8_t, ServerPayload> server_data_map_;
  std::map<uint8_t, TelemetryHistory> history_map_;
  std::map<uint8_t, SafetyEditorState> safety_editor_map_;
  std::map<uint8_t, std::array<float, 4>> hover_input_map_;
  Px4DataObserver log_observer_;
  Px4DataObserver server_observer_;
  std::deque<Px4Client::LogEntry> log_data_;

  Px4Client &px4_client_;
  bool ctrl_in_world_ = false;
  bool keyboard_listener_active_ = false;
  int keyboard_target_id_ = -1;
  float keyboard_vel_xy_ = 1.0F;
  float keyboard_vel_z_ = 0.2F;
  float keyboard_vel_yaw_ = 2.0F;

  static bool valid_limit(float limit);
  static void trim_deque(std::deque<float> &q, size_t max_points);
  static void trim_deque(std::deque<ImVec2> &q, size_t max_points);
  static void render_line_plot(const char *label, const std::deque<float> &series,
                               ImVec2 size, float sample_hz,
                               float min_v = FLT_MAX, float max_v = FLT_MAX);
  static void render_xy_plot(const char *label, const std::deque<ImVec2> &series,
                             ImVec2 size, const float *geofence_min = nullptr,
                             const float *geofence_max = nullptr,
                             bool geofence_enabled = false);

  void render_status_panel(uint8_t id, const ServerPayload &drone);
  void render_command_panel(uint8_t id, const ServerPayload &drone);
  void render_safety_panel(uint8_t id, const ServerPayload &drone);
  void render_plot_panel(uint8_t id, const ServerPayload &drone);
  void handle_keyboard_control();
  void publish_heartbeat();
  void send_hover_target(uint8_t id, const std::array<float, 4> &hover);
  void send_simple_command(uint8_t id, ClientCommand cmd);

  clock::time_point last_heartbeat_time_ = clock::now();
  double heartbeat_interval_ms_ = 200.0;
  mutable std::mutex data_mutex_;
};

inline std::array<double, 4> from_yaw(double yaw) {
  return {std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)};
}

inline double to_yaw(const std::array<double, 4> &q) {
  return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                    q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
}

} // namespace ui
} // namespace px4ctrl
