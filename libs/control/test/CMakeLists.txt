# =============================================================================
# Controlled Descent Simulator — generic control-solver tests
# Self-contained acid tests of libs/control (no core, no quaternions, no scipy):
#   ilqr_test — control-limited iLQR on a plain double-integrator plant
#   lqr_test  — continuous-time LQR synthesis (known-answer gains + stability)
# Build and run:
#   cmake -S libs/control/test -B build-ilqr-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-ilqr-test
#   ./build-ilqr-test/ilqr_test   # loose-box reach + tight-box feasibility
#   ./build-ilqr-test/lqr_test    # double integrator / scalar / unstable plant
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_control_test CXX)

add_executable(ilqr_test ilqr_test.cpp)
target_compile_features(ilqr_test PRIVATE cxx_std_20)
target_compile_options(ilqr_test PRIVATE -Wall -Wextra)
target_include_directories(ilqr_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/control (ilqr.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../integrate # libs/integrate (rk4.hpp)
)

add_executable(lqr_test lqr_test.cpp)
target_compile_features(lqr_test PRIVATE cxx_std_20)
target_compile_options(lqr_test PRIVATE -Wall -Wextra)
target_include_directories(lqr_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/control (lqr.hpp)
)
