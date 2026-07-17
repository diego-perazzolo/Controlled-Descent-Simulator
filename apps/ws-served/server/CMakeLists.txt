# =============================================================================
# ws-served/server — the native half of the ws-served app: cds_server runs
# the core and serves it over WebSocket (libs/ws) to the proxy built by
# ../client. Plain native build, no emsdk required.
#
#   cmake -S apps/ws-served/server -B build-server -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-server
#   ./build-server/cds_server        # listens on ws://0.0.0.0:9002
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_ws_served_server)

if (EMSCRIPTEN)
    message(FATAL_ERROR "ws-served/server is a native app: configure without emcmake")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../core core)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/ws libs_ws)

add_executable(cds_server
    main.cpp
    dispatch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/ext_comm.cpp
)

target_include_directories(cds_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..           # ws_protocol.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common  # ext_comm.hpp, ext_defs.hpp
)

target_link_libraries(cds_server PRIVATE cds_core cds_ws_server)
