#ifndef CDS_TEST_CORE_DEFS_HPP
#define CDS_TEST_CORE_DEFS_HPP
#include <array>
namespace CDS {
    using Vec3 = std::array<double, 3>;
    struct Reference_t {
        Vec3 pos, vel, acc, jerk, snap;
        double yaw, yawRate, yawAcc;
    };
}
#endif
