# =============================================================================
# core — the physics core as a static library (cds_core).
# Toolchain-agnostic: compiled to WASM by apps/wasm-only, natively by
# apps/ws-served/server. Never built standalone; apps add it with
# add_subdirectory and link against cds_core.
# =============================================================================
cmake_minimum_required(VERSION 3.15)

add_library(cds_core STATIC
    core.cpp
    System/SystemManager.cpp
    Models/BaseModel.cpp
    Models/Rocket.cpp
    Models/QuadRotor.cpp
    Trajectory/Trajectory.cpp
    Trajectory/TrajectoryManager.cpp
    Trajectory/Poly4.cpp
    Trajectory/Point.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../modeling/notebooks/exported_cpp/ROCKET_FF_LQR_01/dynamics_rocket_ff_lqr_01.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../modeling/notebooks/exported_cpp/QUADROTOR_FF_LQR_01/dynamics_quadrotor_ff_lqr_01.cpp
)

target_compile_features(cds_core PUBLIC cxx_std_20)

target_include_directories(cds_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/System
    ${CMAKE_CURRENT_SOURCE_DIR}/Models
    ${CMAKE_CURRENT_SOURCE_DIR}/Trajectory
    ${CMAKE_CURRENT_SOURCE_DIR}/../modeling/notebooks/exported_cpp/
    ${CMAKE_CURRENT_SOURCE_DIR}/../modeling/notebooks/exported_cpp/ROCKET_FF_LQR_01/
    ${CMAKE_CURRENT_SOURCE_DIR}/../modeling/notebooks/exported_cpp/QUADROTOR_FF_LQR_01/
)
