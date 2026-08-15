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
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../plants plants)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/ws libs_ws)

# ext API generation: re-runs gen_ext.py when ext_api.py changes
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common/gen_ext.cmake)

add_executable(cds_server
    main.cpp
    ${CDS_EXT_WS_GEN_DIR}/dispatch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../common/ext_comm.cpp
)

target_include_directories(cds_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}   # dispatch.hpp
    ${CDS_EXT_WS_GEN_DIR}         # ws_protocol.hpp
    ${CDS_EXT_COMMON_GEN_DIR}     # ext_comm.hpp, ext_defs.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/log     # log.hpp, LogSinks.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/profile # profile.hpp, ProfileReport.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/sync    # TripleBuffer.hpp
)

# cds_core arrives transitively through cds_plants (PUBLIC), together with
# its include dirs: listing it again duplicates it on the link line
target_link_libraries(cds_server PRIVATE cds_plants cds_ws_server)
add_dependencies(cds_server cds_gen_ext)
