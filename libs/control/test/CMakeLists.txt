# =============================================================================
# Controlled Descent Simulator — generic iLQR solver test
# Self-contained acid test of libs/control/ilqr.hpp on a plain double-integrator
# plant (no core, no quaternions). Build and run:
#   cmake -S libs/control/test -B build-ilqr-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-ilqr-test
#   ./build-ilqr-test/ilqr_test   # loose-box reach + tight-box feasibility
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_ilqr_test CXX)

add_executable(ilqr_test ilqr_test.cpp)
target_compile_features(ilqr_test PRIVATE cxx_std_20)
target_compile_options(ilqr_test PRIVATE -Wall -Wextra)
target_include_directories(ilqr_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/control (ilqr.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../integrate # libs/integrate (rk4.hpp)
)
