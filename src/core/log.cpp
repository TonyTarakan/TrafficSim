#include "core/log.hpp"

#include <quill/Backend.h>
#include <quill/Frontend.h>
#include <quill/sinks/ConsoleSink.h>
#include <quill/sinks/FileSink.h>

#include <mutex>
#include <utility>

namespace ts::log {

namespace {

quill::Logger* g_logger = nullptr;
std::once_flag g_fallback_once;

// Minimal console-only setup used when nobody called init() explicitly.
// Keeps library code (including paths hit directly by unit tests) safe to
// log from without every caller bootstrapping Quill first.
// NOTE: not safe to race with a concurrent explicit init() from another
// thread -- fine for this project's single-threaded startup, revisit if
// that changes.
void init_fallback()
{
    quill::Backend::start();
    auto console_sink = quill::Frontend::create_or_get_sink<quill::ConsoleSink>("console");
    g_logger = quill::Frontend::create_or_get_logger("root", std::move(console_sink));
}

}  // namespace

void init(const std::string& log_file)
{
    quill::Backend::start();

    auto console_sink = quill::Frontend::create_or_get_sink<quill::ConsoleSink>("console");

    quill::FileSinkConfig file_sink_cfg;
    file_sink_cfg.set_open_mode('w');

    auto file_sink =
        quill::Frontend::create_or_get_sink<quill::FileSink>(log_file, file_sink_cfg, quill::FileEventNotifier{});

    g_logger = quill::Frontend::create_or_get_logger(
        "root", {std::move(console_sink), std::move(file_sink)},
        quill::PatternFormatterOptions{
            "%(time) [%(thread_id)] %(short_source_location:<28) LOG_%(log_level:<9) %(message)", "%H:%M:%S.%Qns",
            quill::Timezone::LocalTime});
}

quill::Logger* get() noexcept
{
    std::call_once(g_fallback_once, [] {
        if (!g_logger) {
            init_fallback();
        }
    });
    return g_logger;
}

}  // namespace ts::log
