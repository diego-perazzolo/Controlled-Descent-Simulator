# =============================================================================
# wasm-only — the full in-browser app: core + direct ext adapter + embind
# bindings, all compiled into a single WASM module. No server involved.
#
#   emcmake cmake -S apps/wasm-only -B build-wasm-only -DCMAKE_BUILD_TYPE=Debug
#   cmake --build build-wasm-only
#
# The module always lands in <repo>/build/ (simulator.js + .wasm), which is
# just the delivery point imported by frontend/main.js: each browser app has
# its own build dir, and whichever app was built last owns build/. Switching
# app = building the other one, no reconfigure needed.
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_wasm_only)

if (NOT EMSCRIPTEN)
    message(FATAL_ERROR "wasm-only is a browser app: configure with emcmake")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core core)

# ext API generation: re-runs gen_ext.py when ext_api.py changes
include(${CMAKE_CURRENT_SOURCE_DIR}/../common/gen_ext.cmake)

add_executable(simulator
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/ext_comm.cpp
    ${CDS_EXT_COMMON_GEN_DIR}/bindings.cpp
    ${CDS_EXT_WASMONLY_DIR}/main.cpp
)

target_include_directories(simulator PRIVATE
    ${CDS_EXT_COMMON_GEN_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/log     # log.hpp, LogSinks.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/profile # profile.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libs/sync    # TripleBuffer.hpp
)

target_link_libraries(simulator PRIVATE cds_core)
add_dependencies(simulator cds_gen_ext)

set_target_properties(simulator PROPERTIES
    OUTPUT_NAME "simulator"
    SUFFIX ".js"
)

# Deploy to <repo>/build/, the delivery point imported by the frontend.
# Always-run target (not POST_BUILD): re-deploys even when the module is
# already up to date in this build dir, so switching app is just a build.
add_custom_target(deploy_to_build ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_SOURCE_DIR}/../../build"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE_DIR:simulator>/simulator.js"
            "$<TARGET_FILE_DIR:simulator>/simulator.wasm"
            "${CMAKE_CURRENT_SOURCE_DIR}/../../build"
    DEPENDS simulator
    COMMENT "Deploying wasm-only module to build/"
)

target_link_options(simulator PRIVATE
    -lembind
    -sEXPORT_ES6=1
    -sMODULARIZE=1
    -sEXPORT_NAME=createSimulator
    -sALLOW_MEMORY_GROWTH=1
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
