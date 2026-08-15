# =============================================================================
# Controlled Descent Simulator — Python binding for the generic iLQR solver
# Builds a small C-ABI shared library that exposes libs/control/ilqr.hpp on a
# fixed synthetic benchmark, so ilqr_conformance.py (ctypes, no numpy) can
# certify the C++ solution is a constrained optimum. Build and run:
#   cmake -S libs/control/bind -B build-ilqr-bind -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-ilqr-bind
#   python3 libs/control/bind/ilqr_conformance.py build-ilqr-bind
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_ilqr_bench CXX)

add_library(ilqr_bench SHARED ilqr_bench.cpp)
target_compile_features(ilqr_bench PRIVATE cxx_std_20)
target_compile_options(ilqr_bench PRIVATE -O2 -Wall -Wextra)
target_include_directories(ilqr_bench PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/control (ilqr.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../integrate # libs/integrate (rk4.hpp)
)
