#include "Log.hpp"

#include <cstdio>

namespace hxc {

    namespace {
        eLogLevel g_level = eLogLevel::INFO;

        const char* prefixFor(eLogLevel level) {
            switch (level) {
                case eLogLevel::ERR: return "error";
                case eLogLevel::WARN: return "warn ";
                case eLogLevel::INFO: return "info ";
                case eLogLevel::DEBUG: return "debug";
            }
            return "?????";
        }
    }

    void setLogLevel(eLogLevel level) {
        g_level = level;
    }

    bool logEnabled(eLogLevel level) {
        return static_cast<int>(level) <= static_cast<int>(g_level);
    }

    // Everything goes to stderr so stdout stays available for machine-readable
    // output (`validate --json`, and rawvideo when a future flag wants a pipe).
    void logRaw(eLogLevel level, std::string_view line) {
        std::fprintf(stderr, "[%s] %.*s\n", prefixFor(level), static_cast<int>(line.size()), line.data());
    }

}
