// litegrip/teardown.hpp — the one step wrapper every teardown path uses.
//
// A destructor is implicitly noexcept and has no caller left to handle an
// exception, so a teardown step that can throw — a disable frame on a dead bus
// raises CommError — has to be contained or the process calls std::terminate.
// That is the wrong way round: a caller whose CAN link just died is exactly the
// caller that most needs a clean shutdown and an exit code (issue #14).
//
// Public for the same reason probe.hpp is: the guarantee has to be checkable
// without a bus. See test/test_gripper.cpp.

#pragma once

#include <functional>

namespace litegrip {

/// Run `step` and swallow any exception, so a noexcept caller cannot terminate.
/// Returns true when the step completed normally.
///
/// On failure it prints one line to stderr —
/// `[litegrip] <what> failed during teardown: <error>` — and returns false. The
/// failure is contained and reported, never propagated and never silent: a
/// teardown that could not disable the motor should say so, because that is the
/// one thing the operator may have to deal with by hand.
bool teardown_step_quietly(const char* what,
                           const std::function<void()>& step) noexcept;

}  // namespace litegrip
