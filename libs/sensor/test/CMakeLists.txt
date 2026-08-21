# =============================================================================
# Controlled Descent Simulator — generic sensor-model tests
# Self-contained acid test of libs/sensor (no core, no external deps):
#   sensor_model_test — identity passthrough, exact bias, runtime disable,
#                       injected-noise statistics, PRNG determinism
# Build and run:
#   cmake -S libs/sensor/test -B build-sensor-test -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-sensor-test
#   ./build-sensor-test/sensor_model_test
# =============================================================================
cmake_minimum_required(VERSION 3.15)
project(cds_sensor_test CXX)

add_executable(sensor_model_test sensor_model_test.cpp)
target_compile_features(sensor_model_test PRIVATE cxx_std_20)
target_compile_options(sensor_model_test PRIVATE -Wall -Wextra)
target_include_directories(sensor_model_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..              # libs/sensor (sensor_model.hpp)
)
