#include "odrive_ascii.hpp"

#include <charconv>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace espp {

static bool parse_int(std::string_view sv, int &out) {
  // strict: the whole token must be a valid integer (no trailing garbage), no alloc
  auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
  return ec == std::errc() && ptr == sv.data() + sv.size();
}

static bool parse_float(std::string_view sv, float &out) {
  // std::from_chars for float is not available on every toolchain we target, so
  // fall back to strtod but reject trailing garbage. Short numeric tokens use
  // small-string optimization, so this does not heap-allocate on the hot path.
  std::string tmp(sv);
  char *end = nullptr;
  double val = strtod(tmp.c_str(), &end);
  if (end == tmp.c_str() || end != tmp.c_str() + tmp.size())
    return false; // no conversion, or trailing garbage
  out = static_cast<float>(val);
  return true;
}

std::vector<uint8_t> OdriveAscii::process_bytes(std::span<const uint8_t> data) {
  std::vector<uint8_t> out;
  if (data.empty())
    return out;

  // Under the buffer lock we only touch the buffer: append the new bytes and
  // split out any complete lines (copied into `lines`). The actual command
  // handling - which invokes user callbacks - happens AFTER the lock is
  // released, so a callback can never run while buf_mutex_ is held (avoids
  // re-entrancy deadlock and cross-transport stalls).
  std::vector<std::string> lines;
  {
    std::scoped_lock<std::mutex> lk(buf_mutex_);
    // Append and cap to max_line_length * 4 to avoid unbounded memory growth
    inbuf_.append(reinterpret_cast<const char *>(data.data()), data.size());
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
      // Extract line [start, nl) as an owned copy so it outlives the lock
      lines.emplace_back(inbuf_, start, nl - start);
      // Advance start beyond any contiguous CR/LF
      size_t next = inbuf_.find_first_not_of("\r\n", nl);
      if (next == std::string::npos)
        next = inbuf_.size();
      start = next;
    }

    // Erase processed data
    if (start > 0) {
      inbuf_.erase(0, start);
    }
  }

  // Handle the complete lines outside the buffer lock.
  std::vector<std::string> responses;
  responses.reserve(lines.size());
  for (const auto &line : lines) {
    auto resp = handle_line(line);
    if (resp.has_value()) {
      responses.push_back(std::move(resp.value()));
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

std::optional<std::string> OdriveAscii::command_ack(bool ok, const std::error_code &ec) const {
  if (ok)
    return config_.acknowledge_commands ? std::optional<std::string>("OK\n") : std::nullopt;
  // Errors are always reported, even when acknowledgements are disabled. Guard
  // against a callback that returns false without setting ec ("ERR: Success").
  return fmt::format("ERR: {}\n", ec ? ec.message() : std::string("command failed"));
}

std::optional<std::string> OdriveAscii::handle_line(std::string_view raw) {
  // Guard too-long line attack (buffer may hold up to 4x max_line_length).
  if (raw.size() > config_.max_line_length) {
    logger_.warn("ASCII line too long: {} bytes", raw.size());
    return std::string("ERR: line too long\n");
  }

  auto line = trim(raw);
  if (line.empty())
    return std::nullopt;

  // Strip an ODrive GCODE-style ';' comment (everything to end of line).
  if (auto semi = line.find(';'); semi != std::string::npos)
    line.erase(semi);
  // Strip an optional GCODE-style '*<checksum>' suffix. The checksum is the XOR
  // of the payload bytes. Only treat a trailing '*' as a checksum delimiter when
  // the suffix actually parses as an integer -- otherwise a '*' that is part of a
  // path or value must be left in the line intact. We verify leniently (warn on
  // mismatch) but still process the payload, so checksummed clients are accepted.
  if (auto star = line.rfind('*'); star != std::string::npos) {
    int provided = 0;
    std::string sum = trim(std::string_view(line).substr(star + 1));
    if (parse_int(sum, provided)) {
      uint8_t computed = 0;
      for (size_t i = 0; i < star; ++i)
        computed ^= static_cast<uint8_t>(line[i]);
      if ((provided & 0xFF) != computed)
        logger_.warn("ASCII checksum mismatch (got {}, computed {})", provided & 0xFF, computed);
      line.erase(star); // valid checksum -> strip "*<checksum>", keep the payload
    }
    // else: '*' is part of the payload; leave the line intact.
  }
  line = trim(line);
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
    // Re-split with max 3 parts so the value keeps everything after the path
    // (including embedded spaces) rather than being truncated at the first space.
    auto wtoks = split_ws(line, /*max_parts=*/3);
    if (wtoks.size() < 3)
      return std::string("ERR: w takes 2 arguments\n");
    return handle_write(wtoks[1], wtoks[2]);
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
  // Snapshot the accessor under the lock, then invoke it without the lock held
  // so a getter can safely re-enter the API (e.g. register another property).
  read_fn rf;
  {
    std::scoped_lock<std::mutex> lk(prop_mutex_);
    auto it = properties_.find(std::string(path));
    if (it == properties_.end() || !it->second.read)
      return fmt::format("ERR: unknown property '{}'\n", path);
    rf = it->second.read;
  }
  std::error_code ec;
  auto val = rf(ec);
  if (ec)
    return fmt::format("ERR: {}\n", ec.message());
  // ODrive ASCII returns value followed by \n
  val.push_back('\n');
  return val;
}

std::optional<std::string> OdriveAscii::handle_write(std::string_view path,
                                                     std::string_view value) {
  // Snapshot the accessor under the lock, then invoke it without the lock held.
  write_fn wf;
  {
    std::scoped_lock<std::mutex> lk(prop_mutex_);
    auto it = properties_.find(std::string(path));
    if (it == properties_.end() || !it->second.write)
      return fmt::format("ERR: unknown property '{}'\n", path);
    wf = it->second.write;
  }
  std::error_code ec;
  bool ok = wf(value, ec);
  return command_ack(ok, ec);
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
  return command_ack(ok, ec);
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
  return command_ack(ok, ec);
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
  return command_ack(ok, ec);
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
  return command_ack(ok, ec);
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
    return fmt::format("ERR: {}\n", ec ? ec.message() : std::string("feedback failed"));
  // Feedback is a query and always responds with "<pos> <vel>\n".
  return fmt::format("{:.6g} {:.6g}\n", static_cast<double>(pos), static_cast<double>(vel));
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
  return command_ack(ok, ec);
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
