#pragma once
#include <array>
typedef double core_coord_t;
typedef std::array<core_coord_t,3> Vec3;
typedef struct { double m,Ix,Iy,Iz,g,c,cz,kT,kQ,L,Irot,T_max,T_min; } core_quadRotorParams_t;
typedef struct { Vec3 pos,vel,acc,jerk,snap; core_coord_t yaw,yawRate,yawAcc; } Reference_t;
typedef struct { core_coord_t timestep,user_fX,user_fY,user_fZ; } core_stepParams_t;
