#include "vmu-pro.hpp"

using namespace espp;

VmuPro::VmuPro()
    : BaseComponent("VmuPro") {}

espp::Interrupt &VmuPro::interrupts() { return interrupts_; }
