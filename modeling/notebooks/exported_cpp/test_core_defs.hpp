#ifndef CDS_TEST_CORE_DEFS_HPP
#define CDS_TEST_CORE_DEFS_HPP

#include <array>

namespace CDS {
    using Vec3 = std::array<double, 3>;

    struct Reference_t {
        Vec3 pos;    // position
        Vec3 vel;    // velocity
        Vec3 acc;    // acceleration
        Vec3 jerk;   // d^3 / dt^3
        Vec3 snap;   // d^4 / dt^4
    };
}

#endif // CDS_CORE_DEFS_HPP
