# =============================================================================
# Controlled Descent Simulator — plants test drivers
# Native integration tests of the plant machinery. Build and run:
#   cmake -S plants/test -B build-plants-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-plants-test
#   ./build-plants-test/driver        # mailboxes, freshness, lifecycle
#   ./build-plants-test/sitl_driver   # SITL plant vs in-process fake ArduCopter
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_plants_test CXX)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. plants)

add_executable(driver driver.cpp)
target_link_libraries(driver PRIVATE cds_plants)

# SITL integration test: the fake ArduCopter speaks MAVLink through the same
# pinned headers and talks to the plant over UDP loopback.
add_executable(sitl_driver sitl_driver.cpp)
target_link_libraries(sitl_driver PRIVATE cds_plants)
