# =============================================================================
# ws-served/client — the browser half of the ws-served app: a thin WASM proxy
# exposing the same embind API as wasm-only, but marshalling every ext_* call
# into ws_protocol messages sent through the libs/ws RPC transport to
# cds_server (see ../server). The core is NOT included.
#
#   emcmake cmake -S apps/ws-served/client -B build-ws-client -DCMAKE_BUILD_TYPE=Debug
#   cmake --build build-ws-client
#
# The module always lands in <repo>/build/ (simulator.js + .wasm), same names
# as wasm-only, so the frontend is byte-identical in both modes: whichever
# browser app was built last owns build/. Switching app = building the other
# one, no reconfigure needed.
# Requires the page to be served with COOP/COEP headers (tools/serve.py).
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_ws_served_client)

if (NOT EMSCRIPTEN)
    message(FATAL_ERROR "ws-served/client is a browser app: configure with emcmake")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/ws libs_ws)

# ext API generation: re-runs gen_ext.py when ext_api.py changes
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common/gen_ext.cmake)

add_executable(simulator
    ${CDS_EXT_WS_GEN_DIR}/ext_comm_ws.cpp
    ${CDS_EXT_COMMON_GEN_DIR}/bindings.cpp
)

target_include_directories(simulator PRIVATE
    ${CDS_EXT_WS_GEN_DIR}      # ws_protocol.hpp
    ${CDS_EXT_COMMON_GEN_DIR}  # ext_comm.hpp, ext_defs.hpp
)

target_compile_features(simulator PRIVATE cxx_std_20)

# cds_ws_client also carries the transport pre-js and the exported runtime
# methods it needs, via INTERFACE link options
target_link_libraries(simulator PRIVATE cds_ws_client)
add_dependencies(simulator cds_gen_ext)

set_target_properties(simulator PROPERTIES
    OUTPUT_NAME "simulator"
    SUFFIX ".js"
)

# Deploy to <repo>/build/, the delivery point imported by the frontend.
# Always-run target (not POST_BUILD): re-deploys even when the module is
# already up to date in this build dir, so switching app is just a build.
add_custom_target(deploy_to_build ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_SOURCE_DIR}/../../../build"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE_DIR:simulator>/simulator.js"
            "$<TARGET_FILE_DIR:simulator>/simulator.wasm"
            "${CMAKE_CURRENT_SOURCE_DIR}/../../../build"
    DEPENDS simulator
    COMMENT "Deploying ws-served proxy module to build/"
)

target_link_options(simulator PRIVATE
    -lembind
    -sEXPORT_ES6=1
    -sMODULARIZE=1
    -sEXPORT_NAME=createSimulator
    -sALLOW_MEMORY_GROWTH=1
)
set_property(TARGET simulator APPEND PROPERTY LINK_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../libs/ws/ws_rpc_client_pre.js
)

# Debug: source maps + debug symbols
target_link_options(simulator PRIVATE
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:Debug>:--source-map-base>
    $<$<CONFIG:Debug>:http://localhost:8080/>
)

# Release: optimise for size
target_link_options(simulator PRIVATE
    $<$<CONFIG:Release>:-O2>
)
