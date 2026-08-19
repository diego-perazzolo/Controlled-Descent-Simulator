# =============================================================================
# Controlled Descent Simulator — Python bindings for the generic estimator
# Builds a small C-ABI shared library that exposes the hand-written observer
# synthesis in libs/estimate so the conformance script (ctypes, no numpy) can
# certify the C++ result against an independent Python oracle:
#   observer_bench — observer pass-through (Lyapunov-stationarity certificate,
#                    the dual of the LQR one)
# Build and run:
#   cmake -S libs/estimate/bind -B build-observer-bind -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-observer-bind
#   python3 libs/estimate/bind/observer_conformance.py build-observer-bind
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_estimate_bench CXX)

add_library(observer_bench SHARED observer_bench.cpp)
target_compile_features(observer_bench PRIVATE cxx_std_20)
target_compile_options(observer_bench PRIVATE -O2 -Wall -Wextra)
target_include_directories(observer_bench PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/estimate (observer.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../control    # libs/control  (lqr.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../integrate  # libs/integrate (rk4.hpp)
)
