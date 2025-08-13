#pragma once

#include <sdkconfig.h>

#include "format.hpp"

// for libfmt formtating of wifi_phy_rate_t
template <> struct fmt::formatter<wifi_phy_rate_t> : fmt::formatter<std::string> {
  template <typename FormatContext>
  auto format(const wifi_phy_rate_t &value, FormatContext &ctx) const -> decltype(ctx.out()) {
    switch (value) {
      // base
    case WIFI_PHY_RATE_1M_L:
      return fmt::format_to(ctx.out(), "1 Mbps with long preamble");
    case WIFI_PHY_RATE_2M_L:
      return fmt::format_to(ctx.out(), "2 Mbps with long preamble");
    case WIFI_PHY_RATE_5M_L:
      return fmt::format_to(ctx.out(), "5.5 Mbps with long preamble");
    case WIFI_PHY_RATE_11M_L:
      return fmt::format_to(ctx.out(), "11 Mbps with long preamble");
    case WIFI_PHY_RATE_2M_S:
      return fmt::format_to(ctx.out(), "2 Mbps with short preamble");
    case WIFI_PHY_RATE_5M_S:
      return fmt::format_to(ctx.out(), "5.5 Mbps with short preamble");
    case WIFI_PHY_RATE_11M_S:
      return fmt::format_to(ctx.out(), "11 Mbps with short preamble");

      // HT
    case WIFI_PHY_RATE_48M:
      return fmt::format_to(ctx.out(), "48 Mbps");
    case WIFI_PHY_RATE_24M:
      return fmt::format_to(ctx.out(), "24 Mbps");
    case WIFI_PHY_RATE_12M:
      return fmt::format_to(ctx.out(), "12 Mbps");
    case WIFI_PHY_RATE_6M:
      return fmt::format_to(ctx.out(), "6 Mbps");
    case WIFI_PHY_RATE_54M:
      return fmt::format_to(ctx.out(), "54 Mbps");
    case WIFI_PHY_RATE_36M:
      return fmt::format_to(ctx.out(), "36 Mbps");
    case WIFI_PHY_RATE_18M:
      return fmt::format_to(ctx.out(), "18 Mbps");
    case WIFI_PHY_RATE_9M:
      return fmt::format_to(ctx.out(), "9 Mbps");

      // Long GI
    case WIFI_PHY_RATE_MCS0_LGI:
      return fmt::format_to(ctx.out(), "MCS0_LGI (6.5-13.5 Mbps)");
    case WIFI_PHY_RATE_MCS1_LGI:
      return fmt::format_to(ctx.out(), "MCS1_LGI (13-27 Mbps)");
    case WIFI_PHY_RATE_MCS2_LGI:
      return fmt::format_to(ctx.out(), "MCS2_LGI (19.5-40.5 Mbps)");
    case WIFI_PHY_RATE_MCS3_LGI:
      return fmt::format_to(ctx.out(), "MCS3_LGI (26-54 Mbps)");
    case WIFI_PHY_RATE_MCS4_LGI:
      return fmt::format_to(ctx.out(), "MCS4_LGI (39-81 Mbps)");
    case WIFI_PHY_RATE_MCS5_LGI:
      return fmt::format_to(ctx.out(), "MCS5_LGI (52-108 Mbps)");
    case WIFI_PHY_RATE_MCS6_LGI:
      return fmt::format_to(ctx.out(), "MCS6_LGI (58.5-121.5 Mbps)");
    case WIFI_PHY_RATE_MCS7_LGI:
      return fmt::format_to(ctx.out(), "MCS7_LGI (65-135 Mbps)");
#if CONFIG_SOC_WIFI_HE_SUPPORT || !CONFIG_SOC_WIFI_SUPPORTED
    case WIFI_PHY_RATE_MCS8_LGI:
      return fmt::format_to(ctx.out(), "MCS8_LGI (97.5 Mbps)");
    case WIFI_PHY_RATE_MCS9_LGI:
      return fmt::format_to(ctx.out(), "MCS9_LGI (108.3 Mbps)");
#endif

      // Short GI
    case WIFI_PHY_RATE_MCS0_SGI:
      return fmt::format_to(ctx.out(), "MCS0_SGI (7.2-15 Mbps)");
    case WIFI_PHY_RATE_MCS1_SGI:
      return fmt::format_to(ctx.out(), "MCS1_SGI (14.4-30 Mbps)");
    case WIFI_PHY_RATE_MCS2_SGI:
      return fmt::format_to(ctx.out(), "MCS2_SGI (21.7-45 Mbps)");
    case WIFI_PHY_RATE_MCS3_SGI:
      return fmt::format_to(ctx.out(), "MCS3_SGI (28.9-60 Mbps)");
    case WIFI_PHY_RATE_MCS4_SGI:
      return fmt::format_to(ctx.out(), "MCS4_SGI (43.3-90 Mbps)");
    case WIFI_PHY_RATE_MCS5_SGI:
      return fmt::format_to(ctx.out(), "MCS5_SGI (57.8-120 Mbps)");
    case WIFI_PHY_RATE_MCS6_SGI:
      return fmt::format_to(ctx.out(), "MCS6_SGI (65.0-135 Mbps)");
    case WIFI_PHY_RATE_MCS7_SGI:
      return fmt::format_to(ctx.out(), "MCS7_SGI (72.2-150 Mbps)");
#if CONFIG_SOC_WIFI_HE_SUPPORT || !CONFIG_SOC_WIFI_SUPPORTED
    case WIFI_PHY_RATE_MCS8_SGI:
      return fmt::format_to(ctx.out(), "MCS8_SGI (103.2 Mbps)");
    case WIFI_PHY_RATE_MCS9_SGI:
      return fmt::format_to(ctx.out(), "MCS9_SGI (114.7 Mbps)");
#endif

      // LORA
    case WIFI_PHY_RATE_LORA_250K:
      return fmt::format_to(ctx.out(), "LoRa 250Kbps");
    case WIFI_PHY_RATE_LORA_500K:
      return fmt::format_to(ctx.out(), "LoRa 500Kbps");
    default:
      return fmt::format_to(ctx.out(), "Unknown PHY rate: {}", static_cast<int>(value));
    }
  }
};
