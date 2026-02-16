#pragma once

#include <chrono>
#include <cstdint>

namespace TimeUtils {

inline uint64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

}
