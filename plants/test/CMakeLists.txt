# =============================================================================
# Controlled Descent Simulator — plants test driver
# Native integration test of the plant machinery (mailboxes, freshness,
# SystemManager attach/run/tick/stop). Build and run:
#   cmake -S plants/test -B build-plants-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-plants-test && ./build-plants-test/driver
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_plants_test CXX)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. plants)

add_executable(driver driver.cpp)
target_link_libraries(driver PRIVATE cds_plants)
