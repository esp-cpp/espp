#pragma once

#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace espp::rtsp {

enum class AuthMode { None, Basic, Digest, Both };

struct CryptoCallbacks {
  std::function<std::string(const std::string &)> base64_encode;
  std::function<std::string(const std::string &)> base64_decode;
  std::function<std::string(const std::string &)> md5_hex;
  std::function<std::string(size_t)> random_hex;
};

struct AuthConfig {
  AuthMode mode{AuthMode::None};
  std::string realm{"espp-rtsp"};
  std::chrono::seconds nonceTtl{std::chrono::seconds(300)};
  std::function<bool(const std::string &, const std::string &, const std::string &)>
      validateBasic{};
  std::function<std::optional<std::string>(const std::string &, const std::string &)>
      lookupPassword{};
  CryptoCallbacks crypto{};
};

struct AuthChallenge {
  std::string scheme;
  std::string realm;
  std::string nonce;
  std::string qop;
  std::string algorithm;
  std::string opaque;
  bool stale{false};
};

struct DigestParams {
  std::string username;
  std::string realm;
  std::string nonce;
  std::string uri;
  std::string qop;
  std::string nc;
  std::string cnonce;
  std::string response;
  std::string algorithm;
  std::string opaque;
};

std::string generateNonce(const CryptoCallbacks &crypto, size_t num_bytes = 16);

bool parseWwwAuthenticate(const std::vector<std::string> &wwwAuthHeaders,
                          std::vector<AuthChallenge> &out);

std::string buildBasicAuthorization(const std::string &username, const std::string &password,
                                    const CryptoCallbacks &crypto);

bool parseBasicAuthorization(const std::string &header, std::string &username,
                             std::string &password, const CryptoCallbacks &crypto);

std::string computeDigestResponse(const std::string &username, const std::string &password,
                                  const std::string &realm, const std::string &method,
                                  const std::string &uri, const std::string &nonce,
                                  const std::string &nc, const std::string &cnonce,
                                  const std::string &qop, const CryptoCallbacks &crypto);

std::string buildDigestAuthorization(const std::string &username, const std::string &password,
                                     const AuthChallenge &ch, const std::string &method,
                                     const std::string &uri, const CryptoCallbacks &crypto,
                                     const std::string &nc = "00000001",
                                     const std::string &cnonce = std::string{});

bool parseDigestAuthorization(const std::string &header, DigestParams &out);

bool parseUserInfoFromUrl(const std::string &url, std::string &username, std::string &password);

} // namespace espp::rtsp
