#include "client.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <imgui.h>
#include <mutex>
#include <stdexcept>

namespace px4ctrl {
namespace ui {
namespace {
std::string json5_string(const std::string &value) { return "'" + value + "'"; }

std::string json5_singleton_list(const std::string &value) {
  return "['" + value + "']";
}

bool configure_zenoh(const TransportParas &paras, z_owned_config_t &config) {
  if (z_config_default(&config) < 0) {
    return false;
  }

  const std::string mode =
      (paras.zenoh_mode == "client") ? "client" : "peer";
  if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_MODE_KEY,
                             json5_string(mode).c_str()) < 0) {
    return false;
  }

  if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_MULTICAST_SCOUTING_KEY,
                             paras.zenoh_multicast_scouting ? "true" : "false") <
      0) {
    return false;
  }

  const auto scouting_timeout = std::to_string(paras.zenoh_scouting_timeout_ms);
  if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_SCOUTING_TIMEOUT_KEY,
                             scouting_timeout.c_str()) < 0) {
    return false;
  }

  if (!paras.zenoh_connect.empty()) {
    if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_CONNECT_KEY,
                               json5_singleton_list(paras.zenoh_connect).c_str()) <
        0) {
      return false;
    }
  }

  if (!paras.zenoh_listen.empty()) {
    if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_LISTEN_KEY,
                               json5_singleton_list(paras.zenoh_listen).c_str()) <
        0) {
      return false;
    }
  }

  return true;
}

bool payload_to_bytes(const void *data, const size_t size, z_owned_bytes_t &bytes) {
  return z_bytes_copy_from_buf(&bytes, reinterpret_cast<const uint8_t *>(data),
                               size) >= 0;
}

bool bytes_to_struct(const z_loaned_bytes_t *bytes, void *out, size_t out_size) {
  const auto size = z_bytes_len(bytes);
  if (size != out_size) {
    return false;
  }
  z_bytes_reader_t reader = z_bytes_get_reader(bytes);
  const auto copied = z_bytes_reader_read(&reader, reinterpret_cast<uint8_t *>(out),
                                          out_size);
  return copied == out_size;
}

std::string bytes_to_string(const z_loaned_bytes_t *bytes) {
  const auto size = z_bytes_len(bytes);
  std::string out(size, '\0');
  z_bytes_reader_t reader = z_bytes_get_reader(bytes);
  const auto copied =
      z_bytes_reader_read(&reader, reinterpret_cast<uint8_t *>(out.data()), size);
  if (copied != size) {
    out.resize(copied);
  }
  return out;
}

std::array<double, 4> q_inv(const std::array<double, 4> &q) {
  return {q[0], -q[1], -q[2], -q[3]};
}

std::array<double, 4> q_mul(const std::array<double, 4> &q1,
                            const std::array<double, 4> &q2) {
  return {q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
          q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
          q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
          q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]};
}

std::array<double, 3> q_rot(const std::array<double, 4> &q,
                            const std::array<double, 3> &v) {
  const std::array<double, 4> q_v = {0.0, v[0], v[1], v[2]};
  const std::array<double, 4> q_out = q_mul(q, q_mul(q_v, q_inv(q)));
  return {q_out[1], q_out[2], q_out[3]};
}

Px4Client::LogEntry decode_remote_log(const z_loaned_bytes_t *bytes) {
  Px4Client::LogEntry entry;
  entry.text = bytes_to_string(bytes);

  try {
    const auto parsed = nlohmann::json::parse(entry.text);
    if (!parsed.is_object()) {
      return entry;
    }

    if (parsed.contains("level")) {
      entry.level = parsed.at("level").get<int>();
    }
    if (parsed.contains("text")) {
      entry.text = parsed.at("text").get<std::string>();
    }
  } catch (...) {
    // Backward compatibility: plain text logs from old server versions.
  }

  return entry;
}

ImVec4 log_color_for_level(const int level) {
  switch (static_cast<spdlog::level::level_enum>(level)) {
  case spdlog::level::critical:
  case spdlog::level::err:
    return ImVec4(1.00F, 0.35F, 0.35F, 1.0F);
  case spdlog::level::warn:
    return ImVec4(1.00F, 0.75F, 0.30F, 1.0F);
  case spdlog::level::info:
    return ImVec4(0.65F, 0.90F, 0.65F, 1.0F);
  case spdlog::level::debug:
    return ImVec4(0.60F, 0.80F, 1.00F, 1.0F);
  case spdlog::level::trace:
    return ImVec4(0.75F, 0.75F, 0.75F, 1.0F);
  default:
    break;
  }
  return ImGui::GetStyleColorVec4(ImGuiCol_Text);
}
} // namespace

Px4Client::Px4Client(const TransportParas &paras) : paras_(paras) {
  if (paras_.backend != CommBackend::ZENOH) {
    throw std::runtime_error("Only zenoh backend is supported");
  }

  z_internal_null(&session_);
  z_internal_null(&client_pub_);
  z_internal_null(&server_sub_);
  z_internal_null(&log_sub_);

  if (!init_zenoh()) {
    throw std::runtime_error("Px4Client init failed");
  }
}

bool Px4Client::init_zenoh() {
  static std::once_flag zenoh_log_once;
  std::call_once(zenoh_log_once, []() { zc_init_log_from_env_or("error"); });

  auto open_session = [&](const TransportParas &p) {
    z_owned_config_t config;
    z_internal_null(&config);
    if (!configure_zenoh(p, config)) {
      spdlog::error("Failed to configure zenoh in client");
      z_drop(z_move(config));
      return static_cast<z_result_t>(-1);
    }
    return z_open(&session_, z_move(config), nullptr);
  };

  z_result_t open_ret = open_session(paras_);
  if (open_ret < 0) {
    spdlog::error("Failed to open zenoh session in client, ret={}",
                  static_cast<int>(open_ret));
    return false;
  }

  z_view_keyexpr_t client_key;
  if (z_view_keyexpr_from_str(&client_key, paras_.client_topic.c_str()) < 0) {
    spdlog::error("Invalid client topic keyexpr: {}", paras_.client_topic);
    close_zenoh();
    return false;
  }
  if (z_declare_publisher(z_loan(session_), &client_pub_, z_loan(client_key),
                          nullptr) < 0) {
    spdlog::error("Failed to declare client publisher on {}", paras_.client_topic);
    close_zenoh();
    return false;
  }

  z_owned_closure_sample_t server_closure;
  z_internal_null(&server_closure);
  z_closure_sample(&server_closure, Px4Client::server_sample_callback, nullptr,
                   this);

  z_view_keyexpr_t server_key;
  if (z_view_keyexpr_from_str(&server_key, paras_.server_topic.c_str()) < 0) {
    spdlog::error("Invalid server topic keyexpr: {}", paras_.server_topic);
    close_zenoh();
    return false;
  }
  if (z_declare_subscriber(z_loan(session_), &server_sub_, z_loan(server_key),
                           z_move(server_closure), nullptr) < 0) {
    spdlog::error("Failed to declare server subscriber on {}", paras_.server_topic);
    close_zenoh();
    return false;
  }

  z_owned_closure_sample_t log_closure;
  z_internal_null(&log_closure);
  z_closure_sample(&log_closure, Px4Client::log_sample_callback, nullptr, this);

  z_view_keyexpr_t log_key;
  if (z_view_keyexpr_from_str(&log_key, paras_.log_topic.c_str()) < 0) {
    spdlog::error("Invalid log topic keyexpr: {}", paras_.log_topic);
    close_zenoh();
    return false;
  }
  if (z_declare_subscriber(z_loan(session_), &log_sub_, z_loan(log_key),
                           z_move(log_closure), nullptr) < 0) {
    spdlog::error("Failed to declare log subscriber on {}", paras_.log_topic);
    close_zenoh();
    return false;
  }

  ok_.store(true);
  spdlog::info("Zenoh client ready, pub:{}, sub:{}, log:{}", paras_.client_topic,
               paras_.server_topic, paras_.log_topic);
  return true;
}

void Px4Client::close_zenoh() {
  if (z_internal_check(log_sub_)) {
    (void)z_undeclare_subscriber(z_move(log_sub_));
  }
  if (z_internal_check(server_sub_)) {
    (void)z_undeclare_subscriber(z_move(server_sub_));
  }
  if (z_internal_check(client_pub_)) {
    (void)z_undeclare_publisher(z_move(client_pub_));
  }
  if (z_internal_check(session_)) {
    z_drop(z_move(session_));
  }
}

void Px4Client::pub_client(const ClientPayload &payload) {
  if (!ok_.load()) {
    return;
  }

  z_owned_bytes_t bytes;
  z_internal_null(&bytes);
  if (!payload_to_bytes(&payload, sizeof(ClientPayload), bytes)) {
    spdlog::warn("Failed to serialize client payload");
    return;
  }

  if (z_publisher_put(z_loan(client_pub_), z_move(bytes), nullptr) < 0) {
    spdlog::warn("Failed to publish client payload");
  }
}

void Px4Client::server_sample_callback(z_loaned_sample_t *sample, void *context) {
  auto *self = reinterpret_cast<Px4Client *>(context);
  if (self == nullptr) {
    return;
  }

  const auto *payload_bytes = z_sample_payload(sample);
  if (payload_bytes == nullptr) {
    return;
  }

  ServerPayload payload{};
  if (!bytes_to_struct(payload_bytes, &payload, sizeof(payload))) {
    spdlog::warn("ServerPayload size mismatch");
    return;
  }
  self->server_data.post(payload);
}

void Px4Client::log_sample_callback(z_loaned_sample_t *sample, void *context) {
  auto *self = reinterpret_cast<Px4Client *>(context);
  if (self == nullptr) {
    return;
  }

  const auto *payload_bytes = z_sample_payload(sample);
  if (payload_bytes == nullptr) {
    return;
  }
  self->log_data.post(decode_remote_log(payload_bytes));
}

Px4Client::~Px4Client() {
  ok_.store(false);
  close_zenoh();
  spdlog::info("zenoh client exit");
}

void ImguiClient::TelemetryHistory::push(const ServerPayload &p, size_t max_points) {
  x.push_back(p.pos[0]);
  y.push_back(p.pos[1]);
  z.push_back(p.pos[2]);
  xy_trace.emplace_back(p.pos[0], p.pos[1]);

  thrust.push_back(p.thrust_setpoint);
  omega_x.push_back(p.omega_setpoint[0]);
  omega_y.push_back(p.omega_setpoint[1]);
  omega_z.push_back(p.omega_setpoint[2]);
  omega_xy_trace.emplace_back(p.omega_setpoint[0], p.omega_setpoint[1]);

  while (x.size() > max_points) {
    x.pop_front();
    y.pop_front();
    z.pop_front();
    xy_trace.pop_front();
    thrust.pop_front();
    omega_x.pop_front();
    omega_y.pop_front();
    omega_z.pop_front();
    omega_xy_trace.pop_front();
  }
}

ImguiClient::ImguiClient(Px4Client &px4_client) : px4_client_(px4_client) {
  const auto &transport = px4_client_.transport_paras();
  keyboard_vel_xy_ = std::max(0.0F, transport.keyboard_vel_xy);
  keyboard_vel_z_ = std::max(0.0F, transport.keyboard_vel_z);
  keyboard_vel_yaw_ = std::max(0.0F, transport.keyboard_vel_yaw);

  log_observer_ = px4_client_.log_data.observe([&](const Px4Client::LogEntry &data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    log_data_.push_back(data);
    while (log_data_.size() > 2000) {
      log_data_.pop_front();
    }
  });

  server_observer_ = px4_client_.server_data.observe([&](const ServerPayload &data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    server_data_map_[data.id] = data;
    history_map_[data.id].push(data, 1200);
    if (hover_input_map_.find(data.id) == hover_input_map_.end()) {
      hover_input_map_[data.id] = {
          data.hover_pos[0],
          data.hover_pos[1],
          data.hover_pos[2],
          static_cast<float>(to_yaw({data.hover_quat[0], data.hover_quat[1],
                                     data.hover_quat[2], data.hover_quat[3]})),
      };
    }

    auto &safety = safety_editor_map_[data.id];
    if (!safety.initialized_from_telemetry) {
      for (int i = 0; i < 3; ++i) {
        safety.geofence_min[i] = data.geofence_min[i];
        safety.geofence_max[i] = data.geofence_max[i];
      }
      safety.max_roll_deg = data.max_roll_deg;
      safety.max_pitch_deg = data.max_pitch_deg;
      safety.max_yaw_deg = data.max_yaw_deg;
      safety.enable_geofence = data.enable_geofence != 0;
      safety.enable_attitude_fence = data.enable_attitude_fence != 0;
      safety.initialized_from_telemetry = true;
    }
  });

  command_vec_ = {
      ClientCommand::ARM,          ClientCommand::ENTER_OFFBOARD,
      ClientCommand::EXIT_OFFBOARD, ClientCommand::TAKEOFF,
      ClientCommand::LAND,         ClientCommand::FORCE_HOVER,
      ClientCommand::ALLOW_CMD_CTRL, ClientCommand::FORCE_DISARM,
  };
}

bool ImguiClient::valid_limit(float limit) {
  return limit == -1.0F || (limit > 0.0F && limit <= 180.0F);
}

void ImguiClient::trim_deque(std::deque<float> &q, size_t max_points) {
  while (q.size() > max_points) {
    q.pop_front();
  }
}

void ImguiClient::trim_deque(std::deque<ImVec2> &q, size_t max_points) {
  while (q.size() > max_points) {
    q.pop_front();
  }
}

void ImguiClient::render_line_plot(const char *label, const std::deque<float> &series,
                                   ImVec2 size, float sample_hz, float min_v,
                                   float max_v) {
  if (series.empty()) {
    ImGui::Text("%s: no data", label);
    return;
  }
  ImGui::Text("%s", label);
  ImGui::BeginChild(label, size, true);

  std::vector<float> values(series.begin(), series.end());
  if (min_v == FLT_MAX || max_v == FLT_MAX) {
    const auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
    min_v = *min_it;
    max_v = *max_it;
  }
  if (std::abs(max_v - min_v) < 1e-6F) {
    min_v -= 1.0F;
    max_v += 1.0F;
  }

  ImDrawList *draw = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 p0 = origin;
  const ImVec2 p1 = ImVec2(origin.x + avail.x, origin.y + avail.y);

  draw->AddRectFilled(p0, p1, IM_COL32(20, 20, 24, 255));
  draw->AddRect(p0, p1, IM_COL32(80, 80, 80, 255));

  const float pad_l = 42.0F;
  const float pad_r = 10.0F;
  const float pad_t = 8.0F;
  const float pad_b = 18.0F;
  const float plot_w = std::max(1.0F, avail.x - pad_l - pad_r);
  const float plot_h = std::max(1.0F, avail.y - pad_t - pad_b);

  auto to_screen = [&](const int idx, const float v) {
    const float tx = (values.size() <= 1)
                         ? 1.0F
                         : static_cast<float>(idx) /
                               static_cast<float>(values.size() - 1);
    const float ty = (v - min_v) / (max_v - min_v);
    return ImVec2(p0.x + pad_l + tx * plot_w,
                  p1.y - pad_b - ty * plot_h);
  };

  sample_hz = std::max(1.0F, sample_hz);
  const int kTickCount = (plot_h >= 90.0F) ? 4 : 3;
  for (int i = 0; i <= kTickCount; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kTickCount);
    const float x = p0.x + pad_l + t * plot_w;
    const float y = p1.y - pad_b - t * plot_h;

    draw->AddLine(ImVec2(x, p0.y + pad_t), ImVec2(x, p1.y - pad_b),
                  IM_COL32(55, 55, 65, 180), 1.0F);
    draw->AddLine(ImVec2(p0.x + pad_l, y), ImVec2(p1.x - pad_r, y),
                  IM_COL32(55, 55, 65, 180), 1.0F);

    const float y_val = min_v + t * (max_v - min_v);
    char y_tick[24];
    std::snprintf(y_tick, sizeof(y_tick), "%.2f", y_val);
    draw->AddText(ImVec2(p0.x + 2.0F, y - 7.0F),
                  IM_COL32(150, 150, 160, 220), y_tick);

    const int n = static_cast<int>(values.size());
    const int sample_idx = (n <= 1) ? 0 : static_cast<int>(std::round(t * (n - 1)));
    const int samples_ago = std::max(0, n - 1 - sample_idx);
    const float sec_ago = static_cast<float>(samples_ago) / sample_hz;
    char x_tick[24];
    if (samples_ago == 0 || sec_ago < 0.05F) {
      std::snprintf(x_tick, sizeof(x_tick), "0s");
    } else if (sec_ago >= 10.0F) {
      std::snprintf(x_tick, sizeof(x_tick), "-%.0fs", sec_ago);
    } else {
      std::snprintf(x_tick, sizeof(x_tick), "-%.1fs", sec_ago);
    }
    draw->AddText(ImVec2(x - 12.0F, p1.y - pad_b + 2.0F),
                  IM_COL32(150, 150, 160, 220), x_tick);
  }

  if (min_v < 0.0F && max_v > 0.0F) {
    const float t0 = static_cast<float>((0.0 - min_v) / (max_v - min_v));
    const float y0 = p1.y - pad_b - t0 * plot_h;
    draw->AddLine(ImVec2(p0.x + pad_l, y0), ImVec2(p1.x - pad_r, y0),
                  IM_COL32(110, 110, 135, 220), 1.4F);
  }

  if (values.size() >= 2) {
    ImVec2 last = to_screen(0, values[0]);
    for (int i = 1; i < static_cast<int>(values.size()); ++i) {
      const ImVec2 cur = to_screen(i, values[i]);
      draw->AddLine(last, cur, IM_COL32(80, 220, 120, 255), 1.5F);
      last = cur;
    }
    draw->AddCircleFilled(to_screen(static_cast<int>(values.size() - 1), values.back()),
                          3.0F, IM_COL32(255, 120, 80, 255));
  } else {
    draw->AddCircleFilled(to_screen(0, values[0]), 3.0F,
                          IM_COL32(255, 120, 80, 255));
  }

  ImGui::EndChild();
}

void ImguiClient::render_xy_plot(const char *label, const std::deque<ImVec2> &series,
                                 ImVec2 size, const float *geofence_min,
                                 const float *geofence_max,
                                 const bool geofence_enabled) {
  ImGui::Text("%s", label);
  ImGui::BeginChild(label, size, true);

  ImDrawList *draw = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const ImVec2 p0 = origin;
  const ImVec2 p1 = ImVec2(origin.x + avail.x, origin.y + avail.y);

  draw->AddRectFilled(p0, p1, IM_COL32(20, 20, 24, 255));
  draw->AddRect(p0, p1, IM_COL32(80, 80, 80, 255));

  const bool geofence_valid =
      geofence_min != nullptr && geofence_max != nullptr &&
      geofence_min[0] <= geofence_max[0] && geofence_min[1] <= geofence_max[1];

  float min_x = -1.0F;
  float max_x = 1.0F;
  float min_y = -1.0F;
  float max_y = 1.0F;
  if (!series.empty()) {
    min_x = series.front().x;
    max_x = series.front().x;
    min_y = series.front().y;
    max_y = series.front().y;
    for (const auto &pt : series) {
      min_x = std::min(min_x, pt.x);
      max_x = std::max(max_x, pt.x);
      min_y = std::min(min_y, pt.y);
      max_y = std::max(max_y, pt.y);
    }
  }
  if (geofence_valid) {
    min_x = std::min(min_x, geofence_min[0]);
    max_x = std::max(max_x, geofence_max[0]);
    min_y = std::min(min_y, geofence_min[1]);
    max_y = std::max(max_y, geofence_max[1]);
  }

  if (std::abs(max_x - min_x) < 1e-6F) {
    min_x -= 1.0F;
    max_x += 1.0F;
  }
  if (std::abs(max_y - min_y) < 1e-6F) {
    min_y -= 1.0F;
    max_y += 1.0F;
  }

  const float pad = 8.0F;
  auto to_screen = [&](const ImVec2 &pt) {
    const float x = (pt.x - min_x) / (max_x - min_x);
    const float y = (pt.y - min_y) / (max_y - min_y);
    return ImVec2(p0.x + pad + x * (avail.x - 2 * pad),
                  p1.y - pad - y * (avail.y - 2 * pad));
  };

  constexpr int kTickCount = 4;
  for (int i = 0; i <= kTickCount; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kTickCount);
    const float x_val = min_x + t * (max_x - min_x);
    const float y_val = min_y + t * (max_y - min_y);

    const float x_screen = p0.x + pad + t * (avail.x - 2.0F * pad);
    const float y_screen = p1.y - pad - t * (avail.y - 2.0F * pad);

    draw->AddLine(ImVec2(x_screen, p0.y + pad), ImVec2(x_screen, p1.y - pad),
                  IM_COL32(55, 55, 65, 180), 1.0F);
    draw->AddLine(ImVec2(p0.x + pad, y_screen), ImVec2(p1.x - pad, y_screen),
                  IM_COL32(55, 55, 65, 180), 1.0F);

    char x_tick[24];
    char y_tick[24];
    std::snprintf(x_tick, sizeof(x_tick), "%.1f", x_val);
    std::snprintf(y_tick, sizeof(y_tick), "%.1f", y_val);
    draw->AddText(ImVec2(x_screen - 10.0F, p1.y - pad + 2.0F),
                  IM_COL32(150, 150, 160, 220), x_tick);
    draw->AddText(ImVec2(p0.x + 2.0F, y_screen - 7.0F),
                  IM_COL32(150, 150, 160, 220), y_tick);
  }

  if (min_x < 0.0F && max_x > 0.0F) {
    const float t0 = static_cast<float>((0.0 - min_x) / (max_x - min_x));
    const float x0 = p0.x + pad + t0 * (avail.x - 2.0F * pad);
    draw->AddLine(ImVec2(x0, p0.y + pad), ImVec2(x0, p1.y - pad),
                  IM_COL32(110, 110, 135, 220), 1.4F);
  }
  if (min_y < 0.0F && max_y > 0.0F) {
    const float t0 = static_cast<float>((0.0 - min_y) / (max_y - min_y));
    const float y0 = p1.y - pad - t0 * (avail.y - 2.0F * pad);
    draw->AddLine(ImVec2(p0.x + pad, y0), ImVec2(p1.x - pad, y0),
                  IM_COL32(110, 110, 135, 220), 1.4F);
  }

  if (geofence_valid) {
    const ImVec2 gf_min = to_screen(ImVec2(geofence_min[0], geofence_min[1]));
    const ImVec2 gf_max = to_screen(ImVec2(geofence_max[0], geofence_max[1]));
    const ImVec2 rect_tl(std::min(gf_min.x, gf_max.x),
                         std::min(gf_min.y, gf_max.y));
    const ImVec2 rect_br(std::max(gf_min.x, gf_max.x),
                         std::max(gf_min.y, gf_max.y));
    const ImU32 border_color = geofence_enabled ? IM_COL32(255, 210, 80, 255)
                                                : IM_COL32(140, 140, 140, 180);
    const ImU32 fill_color = geofence_enabled ? IM_COL32(255, 210, 80, 30)
                                              : IM_COL32(140, 140, 140, 16);
    draw->AddRectFilled(rect_tl, rect_br, fill_color);
    draw->AddRect(rect_tl, rect_br, border_color, 0.0F, 0, 1.5F);
  }

  if (series.size() >= 2) {
    ImVec2 last = to_screen(series.front());
    for (size_t i = 1; i < series.size(); ++i) {
      const ImVec2 cur = to_screen(series[i]);
      draw->AddLine(last, cur, IM_COL32(80, 220, 120, 255), 1.5F);
      last = cur;
    }

    draw->AddCircleFilled(to_screen(series.back()), 3.0F,
                          IM_COL32(255, 120, 80, 255));
  } else if (series.size() == 1) {
    draw->AddCircleFilled(to_screen(series.front()), 3.0F,
                          IM_COL32(255, 120, 80, 255));
  }

  ImGui::EndChild();
}

void ImguiClient::render_status_panel(uint8_t id, const ServerPayload &drone) {
  ImGui::SeparatorText("Status");

  int phase_idx = drone.mission_phase;
  const bool phase_ok = phase_idx >= 0 &&
                        phase_idx < static_cast<int>(std::size(MissionPhaseName));
  ImGui::Text("Drone ID: %u", id);
  ImGui::Text("Phase: %s", phase_ok ? MissionPhaseName[phase_idx] : "UNKNOWN");
  ImGui::Text("Offboard:");
  ImGui::SameLine();
  ImGui::TextColored(drone.offboard_state ? ImVec4(0.20F, 0.95F, 0.35F, 1.0F)
                                          : ImVec4(0.95F, 0.20F, 0.20F, 1.0F),
                     drone.offboard_state ? "ON" : "OFF");
  ImGui::Text("Armed:");
  ImGui::SameLine();
  ImGui::TextColored(drone.armed_state ? ImVec4(0.20F, 0.95F, 0.35F, 1.0F)
                                       : ImVec4(0.95F, 0.20F, 0.20F, 1.0F),
                     drone.armed_state ? "ARMED" : "DISARMED");
  ImGui::Text("RC Gate: %s", drone.use_rc ? "ON" : "OFF");
  ImGui::Text("Battery: %.2fV (%.1f%%)", drone.battery_voltage,
              drone.battery_remaining * 100.0f);
  ImGui::Text("Pos xyz: %.2f %.2f %.2f", drone.pos[0], drone.pos[1], drone.pos[2]);
  ImGui::Text("Vel xyz: %.2f %.2f %.2f", drone.vel[0], drone.vel[1], drone.vel[2]);
  ImGui::Text("RPY deg: %.1f %.1f %.1f", drone.roll_deg, drone.pitch_deg,
              drone.yaw_deg);
  ImGui::Text("odom hz: %.1f, cmd hz: %.1f", drone.odom_hz, drone.cmdctrl_hz);
  ImGui::Text("odom age: %.1f ms", drone.odom_age_ms);
}

void ImguiClient::render_command_panel(uint8_t id, const ServerPayload &drone) {
  ImGui::SeparatorText("Commands");

  const bool active = keyboard_listener_active_ && keyboard_target_id_ == id;
  ImGui::Text("Keyboard Listener:");
  ImGui::SameLine();
  ImGui::TextColored(active ? ImVec4(0.1F, 0.9F, 0.2F, 1.0F)
                            : ImVec4(0.95F, 0.25F, 0.25F, 1.0F),
                     active ? "Active" : "Deactive");
  if (ImGui::Button(active ? "Deactivate Keyboard" : "Activate Keyboard")) {
    if (active) {
      keyboard_listener_active_ = false;
      keyboard_target_id_ = -1;
    } else {
      keyboard_listener_active_ = true;
      keyboard_target_id_ = id;
    }
  }

  ImGui::Text("Ctrl Frame: %s", ctrl_in_world_ ? "WORLD" : "BODY");
  if (ImGui::Button("Toggle Ctrl Frame (C)")) {
    ctrl_in_world_ = !ctrl_in_world_;
  }
  ImGui::TextUnformatted("Keys: W/S/A/D XY, R/F Z, Q/E Yaw, Space Hover, J Disarm, L Land");
  ImGui::SetNextItemWidth(140.0F);
  ImGui::InputFloat("Vel XY (m/s)", &keyboard_vel_xy_, 0.1F, 0.5F, "%.2f");
  ImGui::SetNextItemWidth(140.0F);
  ImGui::InputFloat("Vel Z (m/s)", &keyboard_vel_z_, 0.05F, 0.2F, "%.2f");
  ImGui::SetNextItemWidth(140.0F);
  ImGui::InputFloat("Vel Yaw (rad/s)", &keyboard_vel_yaw_, 0.1F, 0.5F, "%.2f");
  keyboard_vel_xy_ = std::max(0.0F, keyboard_vel_xy_);
  keyboard_vel_z_ = std::max(0.0F, keyboard_vel_z_);
  keyboard_vel_yaw_ = std::max(0.0F, keyboard_vel_yaw_);

  for (const auto &cmd : command_vec_) {
    if (ImGui::Button(CommandStr[static_cast<int>(cmd)])) {
      send_simple_command(id, cmd);
    }
  }

  std::array<float, 4> h{};
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (hover_input_map_.find(id) == hover_input_map_.end()) {
      hover_input_map_[id] = {
          drone.hover_pos[0],
          drone.hover_pos[1],
          drone.hover_pos[2],
          static_cast<float>(to_yaw({drone.hover_quat[0], drone.hover_quat[1],
                                     drone.hover_quat[2], drone.hover_quat[3]})),
      };
    }
    h = hover_input_map_[id];
  }

  ImGui::InputFloat3("Hover xyz", h.data(), "%.2f");
  ImGui::InputFloat("Hover yaw", &h[3], 0.05f, 0.2f, "%.2f");
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    hover_input_map_[id] = h;
  }
  if (ImGui::Button("Send Hover Target")) {
    send_hover_target(id, h);
  }
}

void ImguiClient::send_simple_command(uint8_t id, ClientCommand cmd) {
  ClientPayload payload{};
  payload.id = id;
  payload.command = cmd;
  payload.timestamp = to_uint64(clock::now());
  px4_client_.pub_client(payload);
}

void ImguiClient::send_hover_target(uint8_t id, const std::array<float, 4> &hover) {
  ClientPayload payload{};
  payload.id = id;
  payload.command = ClientCommand::CHANGE_HOVER_POS;
  payload.timestamp = to_uint64(clock::now());

  double data[7];
  data[0] = hover[0];
  data[1] = hover[1];
  data[2] = hover[2];
  const auto quat = from_yaw(hover[3]);
  data[3] = quat[0];
  data[4] = quat[1];
  data[5] = quat[2];
  data[6] = quat[3];
  std::memcpy(payload.data, data, sizeof(data));
  px4_client_.pub_client(payload);
}

void ImguiClient::handle_keyboard_control() {
  if (!keyboard_listener_active_ || keyboard_target_id_ < 0) {
    return;
  }
  const uint8_t target_id = static_cast<uint8_t>(keyboard_target_id_);
  ServerPayload drone{};
  std::array<float, 4> hover{};
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto drone_it = server_data_map_.find(target_id);
    if (drone_it == server_data_map_.end()) {
      keyboard_listener_active_ = false;
      keyboard_target_id_ = -1;
      return;
    }
    drone = drone_it->second;

    if (hover_input_map_.find(target_id) == hover_input_map_.end()) {
      hover_input_map_[target_id] = {
          drone.hover_pos[0],
          drone.hover_pos[1],
          drone.hover_pos[2],
          static_cast<float>(to_yaw({drone.hover_quat[0], drone.hover_quat[1],
                                     drone.hover_quat[2], drone.hover_quat[3]})),
      };
    }
    hover = hover_input_map_[target_id];
  }

  ImGuiIO &io = ImGui::GetIO();
  if (io.WantTextInput) {
    return;
  }
  if (ImGui::IsKeyPressed(ImGuiKey_C)) {
    ctrl_in_world_ = !ctrl_in_world_;
  }

  const double dt = io.DeltaTime;
  if (dt <= 0.0) {
    return;
  }

  const std::array<double, 3> vel_body_x = {keyboard_vel_xy_, 0.0, 0.0};
  const std::array<double, 3> vel_body_y = {0.0, keyboard_vel_xy_, 0.0};
  const std::array<double, 4> quat = {drone.quat[0], drone.quat[1], drone.quat[2],
                                      drone.quat[3]};

  std::array<double, 3> vel_world_x = vel_body_x;
  std::array<double, 3> vel_world_y = vel_body_y;
  if (!ctrl_in_world_) {
    vel_world_x = q_rot(quat, vel_body_x);
    vel_world_y = q_rot(quat, vel_body_y);
  }

  bool hover_changed = false;
  if (ImGui::IsKeyDown(ImGuiKey_W)) {
    hover[0] += static_cast<float>(vel_world_x[0] * dt);
    hover[1] += static_cast<float>(vel_world_x[1] * dt);
    hover_changed = true;
  }
  if (ImGui::IsKeyDown(ImGuiKey_S)) {
    hover[0] -= static_cast<float>(vel_world_x[0] * dt);
    hover[1] -= static_cast<float>(vel_world_x[1] * dt);
    hover_changed = true;
  }
  if (ImGui::IsKeyDown(ImGuiKey_A)) {
    hover[0] += static_cast<float>(vel_world_y[0] * dt);
    hover[1] += static_cast<float>(vel_world_y[1] * dt);
    hover_changed = true;
  }
  if (ImGui::IsKeyDown(ImGuiKey_D)) {
    hover[0] -= static_cast<float>(vel_world_y[0] * dt);
    hover[1] -= static_cast<float>(vel_world_y[1] * dt);
    hover_changed = true;
  }
  if (ImGui::IsKeyDown(ImGuiKey_R)) {
    hover[2] += static_cast<float>(keyboard_vel_z_ * dt);
    hover_changed = true;
  }
  if (ImGui::IsKeyDown(ImGuiKey_F)) {
    hover[2] -= static_cast<float>(keyboard_vel_z_ * dt);
    hover_changed = true;
  }
  if (ImGui::IsKeyDown(ImGuiKey_Q)) {
    hover[3] += static_cast<float>(keyboard_vel_yaw_ * dt);
    hover_changed = true;
  }
  if (ImGui::IsKeyDown(ImGuiKey_E)) {
    hover[3] -= static_cast<float>(keyboard_vel_yaw_ * dt);
    hover_changed = true;
  }
  if (hover_changed) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      hover_input_map_[target_id] = hover;
    }
    send_hover_target(target_id, hover);
  }

  if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
    send_simple_command(target_id, ClientCommand::FORCE_HOVER);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_J)) {
    send_simple_command(target_id, ClientCommand::FORCE_DISARM);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_L)) {
    send_simple_command(target_id, ClientCommand::LAND);
  }
}

void ImguiClient::render_safety_panel(uint8_t id, const ServerPayload &drone) {
  SafetyEditorState s{};
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    s = safety_editor_map_[id];
  }
  ImGui::SeparatorText("Safety Online Config");

  ImGui::Checkbox("Enable Geofence", &s.enable_geofence);
  ImGui::Checkbox("Enable Attitude Fence", &s.enable_attitude_fence);
  ImGui::InputFloat3("Geofence Min", s.geofence_min, "%.2f");
  ImGui::InputFloat3("Geofence Max", s.geofence_max, "%.2f");
  ImGui::InputFloat("Max Roll Deg", &s.max_roll_deg, 1.0f, 5.0f, "%.1f");
  ImGui::InputFloat("Max Pitch Deg", &s.max_pitch_deg, 1.0f, 5.0f, "%.1f");
  ImGui::InputFloat("Max Yaw Deg", &s.max_yaw_deg, 1.0f, 5.0f, "%.1f");

  const bool geo_valid = s.geofence_min[0] <= s.geofence_max[0] &&
                         s.geofence_min[1] <= s.geofence_max[1] &&
                         s.geofence_min[2] <= s.geofence_max[2];
  const bool rpy_valid = valid_limit(s.max_roll_deg) && valid_limit(s.max_pitch_deg) &&
                         valid_limit(s.max_yaw_deg);
  const bool valid = geo_valid && rpy_valid;

  if (!valid) {
    ImGui::TextColored(ImVec4(1, 0.2f, 0.2f, 1),
                       "Invalid limits: require min<=max; limit==-1 or (0,180]");
  }

  ImGui::BeginDisabled(!valid);
  if (ImGui::Button("Apply Safety Limits")) {
    ClientPayload payload{};
    payload.id = id;
    payload.command = ClientCommand::SET_SAFETY_LIMITS;
    payload.timestamp = to_uint64(clock::now());

    SafetyLimitsPayload limits{};
    for (int i = 0; i < 3; ++i) {
      limits.geofence_min[i] = s.geofence_min[i];
      limits.geofence_max[i] = s.geofence_max[i];
    }
    limits.max_roll_deg = s.max_roll_deg;
    limits.max_pitch_deg = s.max_pitch_deg;
    limits.max_yaw_deg = s.max_yaw_deg;
    limits.enable_geofence = s.enable_geofence ? 1 : 0;
    limits.enable_attitude_fence = s.enable_attitude_fence ? 1 : 0;

    std::memcpy(payload.data, &limits, sizeof(limits));
    px4_client_.pub_client(payload);
  }
  ImGui::EndDisabled();

  ImGui::Text("Current telemetry limits: R=%.1f P=%.1f Y=%.1f", drone.max_roll_deg,
              drone.max_pitch_deg, drone.max_yaw_deg);
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    safety_editor_map_[id] = s;
  }
}

void ImguiClient::render_plot_panel(uint8_t id, const ServerPayload &drone) {
  TelemetryHistory h{};
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto it = history_map_.find(id);
    if (it == history_map_.end()) {
      return;
    }
    h = it->second;
  }
  const float sample_hz =
      static_cast<float>(std::max<uint32_t>(1, px4_client_.transport_paras().telemetry_hz));
  const ImVec2 line_sz(0, 92);

  ImGui::SeparatorText("Plots");
  ImGui::BeginChild("PlotPanel", ImVec2(0, 0), true);
  render_xy_plot("XY Position Trace", h.xy_trace, ImVec2(340, 240),
                 drone.geofence_min, drone.geofence_max,
                 drone.enable_geofence != 0);
  ImGui::Dummy(ImVec2(0, 4));

  render_line_plot("x", h.x, line_sz, sample_hz);
  render_line_plot("y", h.y, line_sz, sample_hz);
  render_line_plot("z", h.z, line_sz, sample_hz);
  ImGui::Dummy(ImVec2(0, 4));

  render_line_plot("cmd thrust", h.thrust, line_sz, sample_hz);
  render_line_plot("cmd omega_x", h.omega_x, line_sz, sample_hz);
  render_line_plot("cmd omega_y", h.omega_y, line_sz, sample_hz);
  render_line_plot("cmd omega_z", h.omega_z, line_sz, sample_hz);
  ImGui::EndChild();
}

void ImguiClient::publish_heartbeat() {
  const auto now = clock::now();
  if (timeDuration(last_heartbeat_time_, now) < heartbeat_interval_ms_) {
    return;
  }
  last_heartbeat_time_ = now;

  ClientPayload payload{};
  payload.command = ClientCommand::HEARTBEAT;
  payload.timestamp = to_uint64(now);

  std::vector<uint8_t> ids;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ids.reserve(server_data_map_.size());
    for (const auto &[id, _] : server_data_map_) {
      ids.push_back(id);
    }
  }

  if (ids.empty()) {
    payload.id = 0;
    px4_client_.pub_client(payload);
    return;
  }

  for (const auto id : ids) {
    payload.id = id;
    px4_client_.pub_client(payload);
  }
}

void ImguiClient::render_window() {
  ImGuiIO &io = ImGui::GetIO();
  (void)io;

  ImGuiViewport *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(viewport->WorkSize);

  ImGui::Begin("UISC Px4 Client", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                   ImGuiWindowFlags_NoBackground);

  ImGui::Text("UISC Px4 Client (Zenoh)");
  ImGui::Text("Frame: %.2f ms (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);

  const float avail_h = ImGui::GetContentRegionAvail().y;
  const float logs_h = std::clamp(avail_h * 0.28F, 180.0F, 320.0F);
  const float top_h = std::max(220.0F, avail_h - logs_h - ImGui::GetStyle().ItemSpacing.y);

  ImGui::BeginChild("MainContent", ImVec2(0, top_h), true);
  {
    std::vector<std::pair<uint8_t, ServerPayload>> drones;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      drones.assign(server_data_map_.begin(), server_data_map_.end());
    }

    if (drones.empty()) {
      ImGui::Text("No drone telemetry yet");
    }

    for (const auto &[id, drone] : drones) {
      ImGui::PushID(id);
      ImGui::Separator();
      ImGui::Text("Drone %u", id);

      if (ImGui::BeginTable("DroneMain", 2,
                            ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersV |
                                ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableNextColumn();
        render_status_panel(id, drone);
        render_safety_panel(id, drone);

        ImGui::TableNextColumn();
        render_command_panel(id, drone);
        render_plot_panel(id, drone);

        ImGui::EndTable();
      }

      ImGui::PopID();
    }
  }
  ImGui::EndChild();

  handle_keyboard_control();
  publish_heartbeat();

  ImGui::SeparatorText("Logs");
  ImGui::BeginChild("LogContainer", ImVec2(0, 0), true);
  {
    std::deque<Px4Client::LogEntry> logs_snapshot;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      logs_snapshot = log_data_;
    }

    const float prev_scroll_y = ImGui::GetScrollY();
    const float prev_scroll_max = ImGui::GetScrollMaxY();
    const bool stick_to_bottom = (prev_scroll_y >= prev_scroll_max - 8.0F);

    for (const auto &log : logs_snapshot) {
      ImGui::PushStyleColor(ImGuiCol_Text, log_color_for_level(log.level));
      ImGui::TextUnformatted(log.text.c_str());
      ImGui::PopStyleColor();
    }
    if (stick_to_bottom) {
      ImGui::SetScrollHereY(1.0f);
    }
  }
  ImGui::EndChild();

  ImGui::End();
}

} // namespace ui
} // namespace px4ctrl
