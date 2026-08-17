# =============================================================================
# Controlled Descent Simulator — Python bindings for the generic control solvers
# Builds small C-ABI shared libraries that expose the hand-written solvers in
# libs/control so the conformance scripts (ctypes, no numpy) can certify the C++
# result against an independent Python oracle:
#   ilqr_bench — iLQR on a fixed synthetic benchmark (KKT-residual certificate)
#   lqr_bench  — LQR pass-through (Lyapunov-stationarity certificate)
# Build and run:
#   cmake -S libs/control/bind -B build-ilqr-bind -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-ilqr-bind
#   python3 libs/control/bind/ilqr_conformance.py build-ilqr-bind
#   python3 libs/control/bind/lqr_conformance.py  build-ilqr-bind
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_control_bench CXX)

add_library(ilqr_bench SHARED ilqr_bench.cpp)
target_compile_features(ilqr_bench PRIVATE cxx_std_20)
target_compile_options(ilqr_bench PRIVATE -O2 -Wall -Wextra)
target_include_directories(ilqr_bench PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/control (ilqr.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../integrate # libs/integrate (rk4.hpp)
)

add_library(lqr_bench SHARED lqr_bench.cpp)
target_compile_features(lqr_bench PRIVATE cxx_std_20)
target_compile_options(lqr_bench PRIVATE -O2 -Wall -Wextra)
target_include_directories(lqr_bench PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/control (lqr.hpp)
)
