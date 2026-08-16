# =============================================================================
# Controlled Descent Simulator — deferred-formatting logger test
# Self-contained acid test of libs/log (log.hpp + LogRing.hpp + LogSinks.hpp).
# No core, no protocol. Build and run:
#   cmake -S libs/log/test -B build-log-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-log-test
#   ./build-log-test/log_test   # deferred format + level filter + drop counting
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_log_test CXX)

add_executable(log_test log_test.cpp)
target_compile_features(log_test PRIVATE cxx_std_20)
target_compile_options(log_test PRIVATE -Wall -Wextra)
target_include_directories(log_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..   # libs/log (log.hpp, LogRing.hpp, LogSinks.hpp)
)

find_package(Threads REQUIRED)
target_link_libraries(log_test PRIVATE Threads::Threads)
