#pragma once

#include "deep_history_state.hpp"
#include "shallow_history_state.hpp"
#include "state_base.hpp"
#define MAGIC_ENUM_NO_CHECK_SUPPORT 1
#include "magic_enum.hpp"

namespace espp {

/**
 * @brief State machine convenience wrapper for espp. Including
 *        `state_machine.hpp` provides access to all the base classes that the
 *        generated code relies on as well as what you would need to subclass
 *        yourself for a manually written hfsm. Please see
 *        https://github.com/finger563/webgme-hfsm for more information about
 *        modeling, generating, and developing HFSMs.
 *
 * @note This class does not really exist or do anything, but it's the only
 *       way I could figure out how to get this documentation built into the
 *       system :(
 *
 * \section hfsm_ex1 State Machine / HFSM Example with the generated Complex example hfsm
 * \snippet hfsm_example.cpp hfsm example
 * \section hfsm_ex2 Running the HFSM Test Bench on a Real Device:
 * \snippet hfsm_example.cpp hfsm test bench example
 */
class __state_machine_documentation__ {};

} // namespace espp
