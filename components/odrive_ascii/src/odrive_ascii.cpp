#include "odrive_ascii.hpp"

#include <charconv>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace espp {

static bool parse_int(std::string_view sv, int &out) {
  std::string tmp(sv);
  char *end = nullptr;
  long val = strtol(tmp.c_str(), &end, 0);
  if (end == tmp.c_str())
    return false;
  out = static_cast<int>(val);
  return true;
}

static bool parse_float(std::string_view sv, float &out) {
  std::string tmp(sv);
  char *end = nullptr;
  double val = strtod(tmp.c_str(), &end);
  if (end == tmp.c_str())
    return false;
  out = static_cast<float>(val);
  return true;
}

std::vector<uint8_t> OdriveAscii::process_bytes(std::span<const uint8_t> data) {
  std::vector<uint8_t> out;
  if (data.empty())
    return out;

  std::string local;
  local.assign(reinterpret_cast<const char *>(data.data()), data.size());

  std::vector<std::string> responses;
  {
    std::scoped_lock<std::mutex> lk(buf_mutex_);
    // Append and cap to max_line_length * 4 to avoid unbounded memory growth
    inbuf_.append(local);
    if (inbuf_.size() > config_.max_line_length * 4) {
      // keep only the tail in case of garbage flood
      inbuf_.erase(0, inbuf_.size() - config_.max_line_length * 4);
    }

    size_t start = 0;
    while (start < inbuf_.size()) {
      // Accept both \n and \r as terminators; collapse CRLF and LFCR
      size_t nl = inbuf_.find_first_of("\r\n", start);
      if (nl == std::string::npos)
        break;
      // Extract line [start, nl)
      std::string_view line(&inbuf_[start], nl - start);
      // Advance start beyond any contiguous CR/LF
      size_t next = inbuf_.find_first_not_of("\r\n", nl);
      if (next == std::string::npos)
        next = inbuf_.size();

      start = next;

      // Guard too-long line attack
      if (line.size() > config_.max_line_length) {
        logger_.warn("ASCII line too long: {} bytes", line.size());
        responses.emplace_back("ERR: line too long\n");
        continue;
      }

      auto resp = handle_line(line);
      if (resp.has_value()) {
        responses.push_back(std::move(resp.value()));
      }
    }

    // Erase processed data
    if (start > 0) {
      inbuf_.erase(0, start);
    }
  }

  // Concatenate responses into out buffer
  size_t total = 0;
  for (const auto &r : responses)
    total += r.size(); // cppcheck-suppress useStlAlgorithm
  out.resize(total);
  size_t off = 0;
  for (const auto &r : responses) {
    memcpy(out.data() + off, r.data(), r.size());
    off += r.size();
  }
  return out;
}

void OdriveAscii::clear_buffer() {
  std::scoped_lock<std::mutex> lk(buf_mutex_);
  inbuf_.clear();
}

std::optional<std::string> OdriveAscii::handle_line(std::string_view raw) {
  auto line = trim(raw);
  if (line.empty())
    return std::nullopt;

  auto toks = split_ws(line, /*max_parts=*/6);
  if (toks.empty())
    return std::nullopt;

  const auto cmd = toks[0];
  if (cmd == "help") {
    return std::string(
        "ODrive ASCII: r <path> | w <path> <val> | p <axis> <pos> [vel_ff [torque_ff]] | v <axis> "
        "<vel> [torque_ff] | t <axis> <goal_pos_turns> | c <axis> <torque_nm> | f <axis> | es "
        "<axis> <abs_pos_turns>\n");
  }
  if (cmd == "r") {
    if (toks.size() < 2)
      return std::string("ERR: r takes one argument\n");
    return handle_read(toks[1]);
  }
  if (cmd == "w") {
    if (toks.size() < 3)
      return std::string("ERR: w takes 2 arguments\n");
    return handle_write(toks[1], toks[2]);
  }
  if (cmd == "p") {
    return handle_position_cmd({toks.begin(), toks.end()});
  }
  if (cmd == "v") {
    return handle_velocity_cmd({toks.begin(), toks.end()});
  }
  if (cmd == "c") { // torque command in Nm
    return handle_torque_cmd({toks.begin(), toks.end()});
  }
  if (cmd == "t") { // trajectory goal position in turns
    return handle_trajectory_cmd({toks.begin(), toks.end()});
  }
  if (cmd == "f") { // feedback request
    return handle_feedback_cmd({toks.begin(), toks.end()});
  }
  if (cmd == "es") { // encoder set absolute position
    return handle_encoder_set_abs_cmd({toks.begin(), toks.end()});
  }

  return fmt::format("ERR: unknown command '{}'\n", cmd);
}

std::optional<std::string> OdriveAscii::handle_read(std::string_view path) {
  std::scoped_lock<std::mutex> lk(prop_mutex_);
  auto it = properties_.find(std::string(path));
  if (it == properties_.end() || !it->second.read)
    return fmt::format("ERR: unknown property '{}'\n", path);
  std::error_code ec;
  auto val = it->second.read(ec);
  if (ec)
    return fmt::format("ERR: {}\n", ec.message());
  // ODrive ASCII returns value followed by \n
  val.push_back('\n');
  return val;
}

std::optional<std::string> OdriveAscii::handle_write(std::string_view path,
                                                     std::string_view value) {
  std::scoped_lock<std::mutex> lk(prop_mutex_);
  auto it = properties_.find(std::string(path));
  if (it == properties_.end() || !it->second.write)
    return fmt::format("ERR: unknown property '{}'\n", path);
  std::error_code ec;
  bool ok = it->second.write(value, ec);
  return ok ? std::string("OK\n") : fmt::format("ERR: {}\n", ec.message());
}

std::optional<std::string> OdriveAscii::handle_position_cmd(std::span<std::string_view> toks) {
  // p <axis> <pos> [vel_ff [torque_ff]]
  if (toks.size() < 3)
    return fmt::format("ERR: position command requires at least 2 arguments\n");
  int axis = 0;
  float pos = 0;
  std::optional<float> vel_ff;
  std::optional<float> torque_ff;
  if (!parse_int(toks[1], axis))
    return fmt::format("ERR: cannot convert axis '{}' to int\n", toks[1]);
  if (!parse_float(toks[2], pos))
    return fmt::format("ERR: cannot convert pos '{}' to float\n", toks[2]);
  if (toks.size() >= 4) {
    float v = 0;
    if (!parse_float(toks[3], v))
      return fmt::format("ERR: cannot convert vel_ff '{}' to float\n", toks[3]);
    vel_ff = v;
  }
  if (toks.size() >= 5) {
    float t = 0;
    if (!parse_float(toks[4], t))
      return fmt::format("ERR: cannot convert torque_ff '{}' to float\n", toks[4]);
    torque_ff = t;
  }
  position_command_fn cb;
  {
    std::scoped_lock<std::mutex> lk(cb_mutex_);
    cb = pos_cb_;
  }
  if (!cb)
    return fmt::format("ERR: position command callback not set\n");
  std::error_code ec;
  bool ok = cb(axis, pos, vel_ff, torque_ff, ec);
  return ok ? std::string("OK\n") : fmt::format("ERR: {}\n", ec.message());
}

std::optional<std::string> OdriveAscii::handle_velocity_cmd(std::span<std::string_view> toks) {
  // v <axis> <vel> [torque_ff]
  if (toks.size() < 3)
    return fmt::format("ERR: velocity command requires at least 2 arguments\n");
  int axis = 0;
  float vel = 0;
  std::optional<float> torque_ff;
  if (!parse_int(toks[1], axis))
    return fmt::format("ERR: cannot convert axis '{}' to int\n", toks[1]);
  if (!parse_float(toks[2], vel))
    return fmt::format("ERR: cannot convert vel '{}' to float\n", toks[2]);
  if (toks.size() >= 4) {
    float t = 0;
    if (!parse_float(toks[3], t))
      return fmt::format("ERR: cannot convert torque_ff '{}' to float\n", toks[3]);
    torque_ff = t;
  }
  velocity_command_fn cb;
  {
    std::scoped_lock<std::mutex> lk(cb_mutex_);
    cb = vel_cb_;
  }
  if (!cb)
    return fmt::format("ERR: velocity command callback not set\n");
  std::error_code ec;
  bool ok = cb(axis, vel, torque_ff, ec);
  return ok ? std::string("OK\n") : fmt::format("ERR: {}\n", ec.message());
}

std::optional<std::string> OdriveAscii::handle_torque_cmd(std::span<std::string_view> toks) {
  // c <axis> <torque_nm>
  if (toks.size() < 3)
    return fmt::format("ERR: torque command requires 2 arguments\n");
  int axis = 0;
  float tq_nm = 0;
  if (!parse_int(toks[1], axis))
    return fmt::format("ERR: cannot convert axis '{}' to int\n", toks[1]);
  if (!parse_float(toks[2], tq_nm))
    return fmt::format("ERR: cannot convert torque_nm '{}' to float\n", toks[2]);
  torque_command_fn cb;
  {
    std::scoped_lock<std::mutex> lk(cb_mutex_);
    cb = torque_cb_;
  }
  if (!cb)
    return fmt::format("ERR: torque command callback not set\n");
  std::error_code ec;
  bool ok = cb(axis, tq_nm, ec);
  return ok ? std::string("OK\n") : fmt::format("ERR: {}\n", ec.message());
}

std::optional<std::string> OdriveAscii::handle_trajectory_cmd(std::span<std::string_view> toks) {
  // t <axis> <goal_pos_turns>
  if (toks.size() < 3)
    return fmt::format("ERR: trajectory command requires 2 arguments\n");
  int axis = 0;
  float goal = 0;
  if (!parse_int(toks[1], axis))
    return fmt::format("ERR: cannot convert axis '{}' to int\n", toks[1]);
  if (!parse_float(toks[2], goal))
    return fmt::format("ERR: cannot convert goal_pos_turns '{}' to float\n", toks[2]);
  trajectory_command_fn cb;
  {
    std::scoped_lock<std::mutex> lk(cb_mutex_);
    cb = traj_cb_;
  }
  if (!cb)
    return fmt::format("ERR: trajectory command callback not set\n");
  std::error_code ec;
  bool ok = cb(axis, goal, ec);
  return ok ? std::string("OK\n") : fmt::format("ERR: {}\n", ec.message());
}

std::optional<std::string> OdriveAscii::handle_feedback_cmd(std::span<std::string_view> toks) {
  // f <axis>
  if (toks.size() < 2)
    return fmt::format("ERR: feedback command requires 1 argument\n");
  int axis = 0;
  if (!parse_int(toks[1], axis))
    return fmt::format("ERR: cannot convert axis '{}' to int\n", toks[1]);
  feedback_command_fn cb;
  {
    std::scoped_lock<std::mutex> lk(cb_mutex_);
    cb = fb_cb_;
  }
  if (!cb)
    return fmt::format("ERR: feedback command callback not set\n");
  float pos = 0, vel = 0;
  std::error_code ec;
  bool ok = cb(axis, pos, vel, ec);
  if (!ok || ec)
    return fmt::format("ERR: {}\n", ec.message());
  // Return "<pos> <vel>\n"
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%.6g %.6g\n", (double)pos, (double)vel);
  return std::string(buf, n > 0 ? (size_t)n : 0U);
}

std::optional<std::string>
OdriveAscii::handle_encoder_set_abs_cmd(std::span<std::string_view> toks) {
  // es <axis> <abs_pos_turns>
  if (toks.size() < 3)
    return fmt::format("ERR: encoder set absolute command requires 2 arguments\n");
  int axis = 0;
  float abs_pos = 0;
  if (!parse_int(toks[1], axis))
    return fmt::format("ERR: cannot convert axis '{}' to int\n", toks[1]);
  if (!parse_float(toks[2], abs_pos))
    return fmt::format("ERR: cannot convert abs_pos_turns '{}' to float\n", toks[2]);
  encoder_set_abs_position_fn cb;
  {
    std::scoped_lock<std::mutex> lk(cb_mutex_);
    cb = enc_set_abs_position_cb_;
  }
  if (!cb)
    return fmt::format("ERR: encoder set absolute command callback not set\n");
  std::error_code ec;
  bool ok = cb(axis, abs_pos, ec);
  return ok ? std::string("OK\n") : fmt::format("ERR: {}\n", ec.message());
}

std::string OdriveAscii::trim(std::string_view sv) {
  size_t b = 0;
  while (b < sv.size() && (sv[b] == ' ' || sv[b] == '\t'))
    ++b;
  size_t e = sv.size();
  while (e > b && (sv[e - 1] == ' ' || sv[e - 1] == '\t'))
    --e;
  return std::string(sv.substr(b, e - b));
}

std::vector<std::string_view> OdriveAscii::split_ws(std::string_view sv, size_t max_parts) {
  std::vector<std::string_view> out;
  size_t i = 0;
  while (i < sv.size() && out.size() + 1 < max_parts) {
    while (i < sv.size() && (sv[i] == ' ' || sv[i] == '\t'))
      ++i;
    if (i >= sv.size())
      break;
    size_t j = i;
    while (j < sv.size() && sv[j] != ' ' && sv[j] != '\t')
      ++j;
    out.emplace_back(sv.substr(i, j - i));
    i = j;
  }
  // tail
  while (i < sv.size() && (sv[i] == ' ' || sv[i] == '\t'))
    ++i;
  if (i < sv.size())
    out.emplace_back(sv.substr(i));
  return out;
}

} // namespace espp
