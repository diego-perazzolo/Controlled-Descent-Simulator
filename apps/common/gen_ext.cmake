# =============================================================================
# gen_ext.cmake — hooks the ext API generator into an app build.
# include() this from an app CMakeLists: it declares a custom command that
# re-runs gen_ext.py whenever ext_api.py (or the generator itself) changes,
# regenerating the exported_cpp/ files. Nothing runs when they are in sync.
#
# Defines:
#   CDS_EXT_COMMON_GEN_DIR  apps/common/exported_cpp   (+ include dir)
#   CDS_EXT_WS_GEN_DIR      apps/ws-served/exported_cpp (+ include dir)
#   CDS_EXT_COMMON_GEN / CDS_EXT_WS_GEN   the generated file lists
# An app must list the generated .cpp files it compiles among its target
# sources so the dependency takes effect.
# =============================================================================

find_package(Python3 COMPONENTS Interpreter REQUIRED)

get_filename_component(CDS_EXT_COMMON_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
get_filename_component(CDS_EXT_WS_DIR ${CDS_EXT_COMMON_DIR}/../ws-served ABSOLUTE)

set(CDS_EXT_COMMON_GEN_DIR ${CDS_EXT_COMMON_DIR}/exported_cpp)
set(CDS_EXT_WS_GEN_DIR ${CDS_EXT_WS_DIR}/exported_cpp)

set(CDS_EXT_COMMON_GEN
    ${CDS_EXT_COMMON_GEN_DIR}/ext_defs.hpp
    ${CDS_EXT_COMMON_GEN_DIR}/ext_comm.hpp
    ${CDS_EXT_COMMON_GEN_DIR}/bindings.cpp
)
set(CDS_EXT_WS_GEN
    ${CDS_EXT_WS_GEN_DIR}/ws_protocol.hpp
    ${CDS_EXT_WS_GEN_DIR}/ext_comm_ws.cpp
    ${CDS_EXT_WS_GEN_DIR}/dispatch.cpp
)

# Per-build-tree stamp: the generator runs when the description changes and
# rewrites only the files whose content differs, so the other build trees do
# not recompile needlessly. Apps hook in with add_dependencies(cds_gen_ext).
set(CDS_EXT_GEN_STAMP ${CMAKE_CURRENT_BINARY_DIR}/gen_ext.stamp)

add_custom_command(
    OUTPUT ${CDS_EXT_GEN_STAMP}
    BYPRODUCTS ${CDS_EXT_COMMON_GEN} ${CDS_EXT_WS_GEN}
    COMMAND ${Python3_EXECUTABLE} ${CDS_EXT_COMMON_DIR}/gen_ext.py
    COMMAND ${CMAKE_COMMAND} -E touch ${CDS_EXT_GEN_STAMP}
    DEPENDS
        ${CDS_EXT_COMMON_DIR}/ext_api.py
        ${CDS_EXT_COMMON_DIR}/gen_ext.py
    COMMENT "Regenerating ext API from apps/common/ext_api.py"
    VERBATIM
)

add_custom_target(cds_gen_ext DEPENDS ${CDS_EXT_GEN_STAMP})
