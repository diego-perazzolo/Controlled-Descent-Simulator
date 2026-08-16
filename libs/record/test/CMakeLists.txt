# =============================================================================
# Controlled Descent Simulator — data recorder test
# Self-contained acid test of libs/record (Recorder.hpp), reusing the wait-free
# libs/log/LogRing.hpp for the wide-row ring. Build and run:
#   cmake -S libs/record/test -B build-record-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-record-test
#   ./build-record-test/record_test   # schema/CSV round-trip + explicit drops
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_record_test CXX)

add_executable(record_test record_test.cpp)
target_compile_features(record_test PRIVATE cxx_std_20)
target_compile_options(record_test PRIVATE -Wall -Wextra)
target_include_directories(record_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..        # libs/record (Recorder.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../log # libs/log   (LogRing.hpp, reused)
)

find_package(Threads REQUIRED)
target_link_libraries(record_test PRIVATE Threads::Threads)
