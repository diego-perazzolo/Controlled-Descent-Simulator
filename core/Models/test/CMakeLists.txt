# =============================================================================
# Controlled Descent Simulator — MPC model test
# Native acid test of the QuadRotorMPC runtime model (control-limited iLQR/DDP).
# Build and run:
#   cmake -S core/Models/test -B build-mpc-model-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-mpc-model-test
#   ./build-mpc-model-test/mpc_model_test   # Poly4 tracking + gust rejection
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_mpc_model_test CXX)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. core)

add_executable(mpc_model_test mpc_model_test.cpp)
target_link_libraries(mpc_model_test PRIVATE cds_core)

add_executable(rocket_mpc_model_test rocket_mpc_model_test.cpp)
target_link_libraries(rocket_mpc_model_test PRIVATE cds_core)

add_executable(lqr_model_test lqr_model_test.cpp)
target_link_libraries(lqr_model_test PRIVATE cds_core)

add_executable(offset_free_model_test offset_free_model_test.cpp)
target_link_libraries(offset_free_model_test PRIVATE cds_core)

add_executable(lqr_observer_model_test lqr_observer_model_test.cpp)
target_link_libraries(lqr_observer_model_test PRIVATE cds_core)
