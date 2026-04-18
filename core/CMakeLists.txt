cmake_minimum_required(VERSION 3.15)
project(ControlledDescentSimulator)

set(CMAKE_CXX_STANDARD 20)

add_executable(simulator
    core.cpp
    Models/BaseModel.cpp
    Models/Rocket.cpp
    Trajectory.cpp
    ext/ext_comm.cpp
    ext/bindings.cpp
)

target_include_directories(simulator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ext
    ${CMAKE_CURRENT_SOURCE_DIR}/Models
)

# Emscripten-specific flags
if (EMSCRIPTEN)
    set_target_properties(simulator PROPERTIES
        OUTPUT_NAME "simulator"
        SUFFIX ".js"
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
endif()
