#include "rtsp_auth.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <unordered_map>

namespace espp::rtsp {

static std::string toLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

static std::unordered_map<std::string, std::string> parseKvList(const std::string &s) {
  std::unordered_map<std::string, std::string> kv;
  size_t i = 0, n = s.size();
  while (i < n) {
    while (i < n && (s[i] == ',' || s[i] == ' ' || s[i] == '\t'))
      ++i;
    size_t kstart = i;
    while (i < n && s[i] != '=' && s[i] != ',')
      ++i;
    if (i >= n || s[i] != '=')
      break;
    std::string key = s.substr(kstart, i - kstart);
    while (!key.empty() && std::isspace(static_cast<unsigned char>(key.back())))
      key.pop_back();
    std::string lkey = toLower(key);
    ++i;
    std::string value;
    if (i < n && s[i] == '"') {
      ++i;
      while (i < n && s[i] != '"') {
        if (s[i] == '\\' && i + 1 < n) {
          value.push_back(s[i + 1]);
          i += 2;
        } else {
          value.push_back(s[i++]);
        }
      }
      if (i < n && s[i] == '"')
        ++i;
    } else {
      size_t vstart = i;
      while (i < n && s[i] != ',')
        ++i;
      value = s.substr(vstart, i - vstart);
      while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
        value.pop_back();
    }
    size_t p = 0;
    while (p < value.size() && std::isspace(static_cast<unsigned char>(value[p])))
      ++p;
    if (p > 0)
      value.erase(0, p);
    kv[lkey] = value;
    if (i < n && s[i] == ',')
      ++i;
  }
  return kv;
}

std::string generateNonce(const CryptoCallbacks &crypto, size_t num_bytes) {
  if (crypto.random_hex)
    return crypto.random_hex(num_bytes);
  static unsigned long counter = 0x12345678;
  counter += 0x9e3779b9u;
  const char *hex = "0123456789abcdef";
  std::string out;
  out.reserve(num_bytes * 2);
  for (size_t i = 0; i < num_bytes; ++i) {
    unsigned char b = static_cast<unsigned char>((counter >> ((i & 3) * 8)) & 0xFF);
    out.push_back(hex[(b >> 4) & 0xF]);
    out.push_back(hex[b & 0xF]);
  }
  return out;
}

bool parseWwwAuthenticate(const std::vector<std::string> &headers,
                          std::vector<AuthChallenge> &out) {
  out.clear();
  for (const auto &h : headers) {
    size_t sp = h.find(' ');
    std::string scheme = (sp == std::string::npos) ? h : h.substr(0, sp);
    std::string rest = (sp == std::string::npos) ? "" : h.substr(sp + 1);
    std::string lscheme = toLower(scheme);
    AuthChallenge ch;
    if (lscheme == "basic") {
      ch.scheme = "Basic";
      auto kv = parseKvList(rest);
      if (kv.count("realm"))
        ch.realm = kv["realm"];
      out.push_back(ch);
    } else if (lscheme == "digest") {
      ch.scheme = "Digest";
      auto kv = parseKvList(rest);
      if (kv.count("realm"))
        ch.realm = kv["realm"];
      if (kv.count("nonce"))
        ch.nonce = kv["nonce"];
      if (kv.count("qop"))
        ch.qop = kv["qop"];
      ch.algorithm = kv.count("algorithm") ? kv["algorithm"] : "MD5";
      if (kv.count("opaque"))
        ch.opaque = kv["opaque"];
      if (kv.count("stale"))
        ch.stale = (toLower(kv["stale"]) == "true");
      out.push_back(ch);
    }
  }
  return !out.empty();
}

std::string buildBasicAuthorization(const std::string &username, const std::string &password,
                                    const CryptoCallbacks &crypto) {
  if (!crypto.base64_encode)
    return {};
  const std::string merged = username + ":" + password;
  return std::string("Basic ") + crypto.base64_encode(merged);
}

bool parseBasicAuthorization(const std::string &header, std::string &username,
                             std::string &password, const CryptoCallbacks &crypto) {
  if (header.size() < 6)
    return false;
  std::string prefix = toLower(header.substr(0, 6));
  if (prefix != "basic ")
    return false;
  if (!crypto.base64_decode)
    return false;
  std::string b64 = header.substr(6);
  std::string decoded = crypto.base64_decode(b64);
  auto pos = decoded.find(':');
  if (pos == std::string::npos)
    return false;
  username = decoded.substr(0, pos);
  password = decoded.substr(pos + 1);
  return true;
}

std::string computeDigestResponse(const std::string &username, const std::string &password,
                                  const std::string &realm, const std::string &method,
                                  const std::string &uri, const std::string &nonce,
                                  const std::string &nc, const std::string &cnonce,
                                  const std::string &qop, const CryptoCallbacks &crypto) {
  if (!crypto.md5_hex)
    return {};
  const std::string ha1 = crypto.md5_hex(username + ":" + realm + ":" + password);
  const std::string ha2 = crypto.md5_hex(method + ":" + uri);
  if (!qop.empty()) {
    return crypto.md5_hex(ha1 + ":" + nonce + ":" + nc + ":" + cnonce + ":" + qop + ":" + ha2);
  }
  return crypto.md5_hex(ha1 + ":" + nonce + ":" + ha2);
}

std::string buildDigestAuthorization(const std::string &username, const std::string &password,
                                     const AuthChallenge &ch, const std::string &method,
                                     const std::string &uri, const CryptoCallbacks &crypto,
                                     const std::string &nc, const std::string &cnonce_in) {
  const std::string qop = ch.qop.empty() ? "" : "auth";
  const std::string cnonce = !cnonce_in.empty() ? cnonce_in : generateNonce(crypto, 16);
  const std::string response = computeDigestResponse(username, password, ch.realm, method, uri,
                                                     ch.nonce, nc, cnonce, qop, crypto);
  std::ostringstream oss;
  oss << "Digest username=\"" << username << "\""
      << ", realm=\"" << ch.realm << "\""
      << ", nonce=\"" << ch.nonce << "\""
      << ", uri=\"" << uri << "\""
      << ", response=\"" << response << "\"";
  if (!qop.empty()) {
    oss << ", qop=" << qop << ", nc=" << nc << ", cnonce=\"" << cnonce << "\"";
  }
  if (!ch.algorithm.empty())
    oss << ", algorithm=" << ch.algorithm;
  if (!ch.opaque.empty())
    oss << ", opaque=\"" << ch.opaque << "\"";
  return oss.str();
}

bool parseDigestAuthorization(const std::string &header, DigestParams &out) {
  if (header.size() < 7)
    return false;
  std::string prefix = toLower(header.substr(0, 7));
  if (prefix != "digest ")
    return false;
  auto kv = parseKvList(header.substr(7));
  auto get = [&](const char *k) -> std::string {
    auto it = kv.find(toLower(std::string(k)));
    return it == kv.end() ? std::string() : it->second;
  };
  out.username = get("username");
  out.realm = get("realm");
  out.nonce = get("nonce");
  out.uri = get("uri");
  out.response = get("response");
  out.qop = get("qop");
  out.nc = get("nc");
  out.cnonce = get("cnonce");
  out.algorithm = get("algorithm");
  out.opaque = get("opaque");
  if (out.algorithm.empty())
    out.algorithm = "MD5";
  return !out.username.empty() && !out.realm.empty() && !out.nonce.empty() && !out.uri.empty() &&
         !out.response.empty();
}

static std::string urlDecode(const std::string &s) {
  std::string out;
  out.reserve(s.size());
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '%' && i + 2 < s.size()) {
      unsigned int byte = 0;
      std::string hex = s.substr(i + 1, 2);
      std::istringstream iss(hex);
      iss >> std::hex >> byte;
      if (!iss.fail()) {
        out.push_back(static_cast<char>(byte));
        i += 2;
        continue;
      }
    } else if (s[i] == '+') {
      out.push_back(' ');
      continue;
    }
    out.push_back(s[i]);
  }
  return out;
}

bool parseUserInfoFromUrl(const std::string &url, std::string &username, std::string &password) {
  username.clear();
  password.clear();
  auto schemePos = url.find("://");
  if (schemePos == std::string::npos)
    return false;
  size_t start = schemePos + 3;
  size_t at = url.find('@', start);
  if (at == std::string::npos)
    return false;
  size_t slash = url.find('/', start);
  if (slash != std::string::npos && at > slash)
    return false;
  std::string userinfo = url.substr(start, at - start);
  auto colon = userinfo.find(':');
  if (colon == std::string::npos) {
    username = urlDecode(userinfo);
    return !username.empty();
  }
  username = urlDecode(userinfo.substr(0, colon));
  password = urlDecode(userinfo.substr(colon + 1));
  return !username.empty();
}

} // namespace espp::rtsp
