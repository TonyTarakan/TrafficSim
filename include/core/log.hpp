#pragma once

#include <quill/Logger.h>

// Thin wrapper around Quill, following Quill's own recommended integration
// pattern: keep Backend/Frontend setup and sink/logger creation in one
// small translation unit, so the rest of the app only needs the
// lightweight headers to actually log:
//
//   #include "core/log.hpp"
//   #include <quill/LogMacros.h>
//
//   LOG_INFO(ts::log::get(), "vehicle {} spawned on lane {}", id, lane_id);
//
// A single global logger is enough for a project this size; if subsystems
// ever need their own levels/sinks, split get() into named getters.

namespace ts::log {

// Starts the Quill backend thread and wires up sinks (console + file).
// Call once at startup, before the first LOG_* call.
void init(const std::string& log_file = "trafficsim.log");

// The shared application logger. Lazily created with a minimal
// console-only sink on first use if init() hasn't run yet, so library
// code (including code exercised directly by unit tests) can log without
// every caller having to bootstrap Quill first. Call init() explicitly at
// app startup to get the real file sink + pattern instead of the fallback.
[[nodiscard]] quill::Logger* get() noexcept;

}  // namespace ts::log
