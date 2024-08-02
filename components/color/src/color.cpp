#include "color.hpp"

namespace espp {

Rgb::Rgb(const float &_r, const float &_g, const float &_b)
    : r(_r)
    , g(_g)
    , b(_b) {
  if (r > 1.0 || g > 1.0 || b > 1.0) {
    r /= 255.;
    g /= 255.;
    b /= 255.;
  }
  r = std::clamp(r, 0.0f, 1.0f);
  g = std::clamp(g, 0.0f, 1.0f);
  b = std::clamp(b, 0.0f, 1.0f);
}

Rgb::Rgb(const Rgb &rgb)
    : r(rgb.r)
    , g(rgb.g)
    , b(rgb.b) {}

Rgb::Rgb(const Hsv &hsv)
    : Rgb(hsv.rgb()) {}

Rgb::Rgb(const uint32_t &hex) {
  r = ((hex >> 16) & 0xFF) / 255.0f;
  g = ((hex >> 8) & 0xFF) / 255.0f;
  b = (hex & 0xFF) / 255.0f;
}

Rgb &Rgb::operator=(const Hsv &hsv) {
  *this = hsv.rgb();
  return *this;
}

Rgb Rgb::operator+(const Rgb &rhs) const {
  // divide by number of elements that went into it (blending) instead of just
  // addition (which would be more like the light model) so that we are better
  // able to create gradients.
  return Rgb((r + rhs.r) / 2.0f, (g + rhs.g) / 2.0f, (b + rhs.b) / 2.0f);
}

Rgb &Rgb::operator+=(const Rgb &rhs) {
  r += rhs.r;
  g += rhs.g;
  b += rhs.b;
  // divide by number of elements that went into it (blending) instead of just
  // addition (which would be more like the light model) so that we are better
  // able to create gradients.
  r /= 2.0f;
  g /= 2.0f;
  b /= 2.0f;
  return *this;
}

Hsv Rgb::hsv() const {
  Hsv HSV;

  float max = std::max(r, std::max(g, b)), min = std::min(r, std::min(g, b));

  HSV.v = max;
  if (max == 0.0f) {
    HSV.s = 0.0;
  } else {
    HSV.s = (max - min) / max;
  }
  if (HSV.s > 0.0f) {
    float d = max - min;
    if (r == max) {
      // color is between yellow and magenta
      HSV.h = (g - b) / d;
    } else if (g == max) {
      // color is between cyan and yellow
      HSV.h = 2.0f + (b - r) / d;
    } else if (b == max) {
      // color is between magenta and cyan
      HSV.h = 4.0f + (r - g) / d;
    }
    // convert to degrees
    HSV.h *= 60.0f;
    // prevent negative values
    if (HSV.h < 0) {
      HSV.h += 360.0f;
    }
  }

  return HSV;
}

uint32_t Rgb::hex() const {
  uint32_t hex = 0;
  hex |= static_cast<uint32_t>(r * 255) << 16;
  hex |= static_cast<uint32_t>(g * 255) << 8;
  hex |= static_cast<uint32_t>(b * 255);
  return hex;
}

Hsv::Hsv(const float &_h, const float &_s, const float &_v)
    : h(_h)
    , s(_s)
    , v(_v) {
  h = std::clamp(h, 0.0f, 360.0f);
  s = std::clamp(s, 0.0f, 1.0f);
  v = std::clamp(v, 0.0f, 1.0f);
}

Hsv::Hsv(const Hsv &hsv)
    : h(hsv.h)
    , s(hsv.s)
    , v(hsv.v) {}

Hsv::Hsv(const Rgb &rgb)
    : Hsv(rgb.hsv()) {}

Hsv &Hsv::operator=(const Rgb &rgb) {
  *this = rgb.hsv();
  return *this;
}

Rgb Hsv::rgb() const {
  Rgb RGB;

  float H = h, S = s, V = v, P, Q, T, fract;

  (H == 360.) ? (H = 0.) : (H /= 60.);
  fract = H - floor(H);

  P = V * (1. - S);
  Q = V * (1. - S * fract);
  T = V * (1. - S * (1. - fract));

  if (0. <= H && H < 1.)
    RGB = Rgb(V, T, P);
  else if (1. <= H && H < 2.)
    RGB = Rgb(Q, V, P);
  else if (2. <= H && H < 3.)
    RGB = Rgb(P, V, T);
  else if (3. <= H && H < 4.)
    RGB = Rgb(P, Q, V);
  else if (4. <= H && H < 5.)
    RGB = Rgb(T, P, V);
  else if (5. <= H && H < 6.)
    RGB = Rgb(V, P, Q);
  else
    RGB = Rgb(0., 0., 0.);

  return RGB;
}
} // namespace espp
