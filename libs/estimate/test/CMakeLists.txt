# =============================================================================
# Controlled Descent Simulator — generic estimator tests
# Self-contained acid test of libs/estimate (no core, no quaternions, no scipy):
#   observer_test — continuous-time observer synthesis (dual of LQR: known-answer
#                   gains + error-dynamics stability) plus a Step() convergence run
# Build and run:
#   cmake -S libs/estimate/test -B build-observer-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-observer-test
#   ./build-observer-test/observer_test   # scalar / double-integrator / unstable / dynamic
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_estimate_test CXX)

add_executable(observer_test observer_test.cpp)
target_compile_features(observer_test PRIVATE cxx_std_20)
target_compile_options(observer_test PRIVATE -Wall -Wextra)
target_include_directories(observer_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/estimate (observer.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../control    # libs/control  (lqr.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../integrate  # libs/integrate (rk4.hpp)
)

# reusable translational disturbance observer (offset-free wiring pattern)
add_executable(tdo_test tdo_test.cpp)
target_compile_features(tdo_test PRIVATE cxx_std_20)
target_compile_options(tdo_test PRIVATE -Wall -Wextra)
target_include_directories(tdo_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/estimate (trans_disturbance_observer.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../control    # libs/control  (lqr.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../integrate  # libs/integrate (rk4.hpp)
)
