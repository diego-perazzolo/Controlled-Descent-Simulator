# =============================================================================
# Controlled Descent Simulator — scope profiler test
# Self-contained acid test of libs/profile (profile.hpp), reusing the wait-free
# libs/sync TripleBuffer for the snapshot mailbox. Build and run:
#   cmake -S libs/profile/test -B build-profile-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-profile-test
#   ./build-profile-test/profile_test   # aggregate stats + snapshot round-trip
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_profile_test CXX)

add_executable(profile_test profile_test.cpp)
target_compile_features(profile_test PRIVATE cxx_std_20)
target_compile_options(profile_test PRIVATE -Wall -Wextra)
target_include_directories(profile_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..        # libs/profile (profile.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/../../sync # libs/sync (TripleBuffer.hpp)
)

find_package(Threads REQUIRED)
target_link_libraries(profile_test PRIVATE Threads::Threads)
