# =============================================================================
# libs/ws — in-house dependency-free WebSocket RPC infrastructure.
# Protocol-agnostic byte transport with transport-owned request/response
# correlation (4-byte id on the wire, invisible to both endpoints).
#
# Provides one target depending on the toolchain:
#   cds_ws_client (emscripten) — synchronous RPC from the browser main thread
#                                (EM_JS + SharedArrayBuffer bridge worker);
#                                carries its pre-js via INTERFACE link options.
#   cds_ws_server (native)     — minimal RFC 6455 server, one client at a time.
# =============================================================================
cmake_minimum_required(VERSION 3.15)

if (EMSCRIPTEN)
    # OBJECT (not STATIC): the EM_JS code lives in a section of the object
    # file that the linker would not pull out of an archive, since nothing
    # references it as a symbol — objects must reach the link line directly.
    add_library(cds_ws_client OBJECT ws_rpc_client.cpp)
    target_include_directories(cds_ws_client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(cds_ws_client PUBLIC cxx_std_20)

    # The bridge worker must exist before the first spin-waiting RPC, and the
    # runtime helpers it uses must be exported: both travel with the target.
    target_link_options(cds_ws_client INTERFACE
        "--pre-js=${CMAKE_CURRENT_SOURCE_DIR}/ws_rpc_client_pre.js"
        -sEXPORTED_RUNTIME_METHODS=addRunDependency,removeRunDependency
    )
else()
    add_library(cds_ws_server STATIC ws_server.cpp)
    target_include_directories(cds_ws_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(cds_ws_server PUBLIC cxx_std_20)
endif()
