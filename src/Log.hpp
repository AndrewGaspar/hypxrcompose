#pragma once

#include <format>
#include <string>
#include <string_view>

namespace hxc {

    enum class eLogLevel {
        ERR = 0,
        WARN,
        INFO,
        DEBUG,
    };

    void setLogLevel(eLogLevel level);
    bool logEnabled(eLogLevel level);
    void logRaw(eLogLevel level, std::string_view line);

    template <typename... Args>
    void logf(eLogLevel level, std::format_string<Args...> fmt, Args&&... args) {
        if (!logEnabled(level))
            return;
        logRaw(level, std::format(fmt, std::forward<Args>(args)...));
    }

#define HXC_ERR(...)   ::hxc::logf(::hxc::eLogLevel::ERR, __VA_ARGS__)
#define HXC_WARN(...)  ::hxc::logf(::hxc::eLogLevel::WARN, __VA_ARGS__)
#define HXC_INFO(...)  ::hxc::logf(::hxc::eLogLevel::INFO, __VA_ARGS__)
#define HXC_DEBUG(...) ::hxc::logf(::hxc::eLogLevel::DEBUG, __VA_ARGS__)

}
