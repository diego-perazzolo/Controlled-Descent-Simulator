# =============================================================================
# Controlled Descent Simulator — diagnostics micro-benchmark
# Sizes the per-call cost of the logger / profiler / recorder macros (ON vs OFF
# at runtime, drain cost) and, via the _off target, the compile-out residual.
# Not a CI correctness gate — timing-dependent. Build and run:
#   cmake -S bench -B build-bench -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-bench
#   ./build-bench/perf_bench       # features compiled in
#   ./build-bench/perf_bench_off   # features compiled out (macros stripped)
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_perf_bench CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(BENCH_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/log     # log.hpp, LogSinks.hpp, LogRing.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/profile # profile.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/record  # Recorder.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../libs/sync    # TripleBuffer.hpp
)

find_package(Threads REQUIRED)

# features compiled in (runtime ON/OFF measurements)
add_executable(perf_bench perf_bench.cpp)
target_compile_features(perf_bench PRIVATE cxx_std_20)
target_compile_options(perf_bench PRIVATE -Wall -Wextra -O2)
target_include_directories(perf_bench PRIVATE ${BENCH_INCLUDES})
target_link_libraries(perf_bench PRIVATE Threads::Threads)

# same source, macros stripped: shows the compile-out residual (~0)
add_executable(perf_bench_off perf_bench.cpp)
target_compile_features(perf_bench_off PRIVATE cxx_std_20)
target_compile_options(perf_bench_off PRIVATE -Wall -Wextra -O2)
target_compile_definitions(perf_bench_off PRIVATE
    CDS_LOG_COMPILE_LEVEL=5   # strip Trace..Error (all normal logs)
    CDS_PROFILE_ENABLED=0
    CDS_RECORD_ENABLED=0
)
target_include_directories(perf_bench_off PRIVATE ${BENCH_INCLUDES})
target_link_libraries(perf_bench_off PRIVATE Threads::Threads)

# per-model integration benchmark — links the physics core (models + trajectory)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core core)
add_executable(model_bench model_bench.cpp)
target_compile_features(model_bench PRIVATE cxx_std_20)
target_compile_options(model_bench PRIVATE -Wall -Wextra -O2)
target_link_libraries(model_bench PRIVATE cds_core Threads::Threads)
