# =============================================================================
# Controlled Descent Simulator — plants/
# Concrete plant implementations (threads, external links). Depends on the
# core (BasePlant interface); never the other way around.
# =============================================================================
cmake_minimum_required(VERSION 3.15)

if(NOT TARGET cds_core)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core core)
endif()

add_library(cds_plants STATIC
    loopback/LoopbackPlant.cpp
)

target_compile_features(cds_plants PUBLIC cxx_std_20)

target_include_directories(cds_plants PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}          # for the explicit "loopback/…" style
    ${CMAKE_CURRENT_SOURCE_DIR}/loopback
)

target_link_libraries(cds_plants PUBLIC cds_core)

find_package(Threads REQUIRED)
target_link_libraries(cds_plants PUBLIC Threads::Threads)
