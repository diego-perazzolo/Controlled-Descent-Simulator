#if 0
#pragma once
#include <array>
namespace CDS {
    using Vec3 = std::array<double, 3>;
    using RefVec = Vec3;
    using UserForces = Vec3;

    typedef struct{
        Vec3 pos;
        Vec3 vel;
        Vec3 acc;
        Vec3 jerk;
        Vec3 snap;
    } Reference_t;
}

#endif