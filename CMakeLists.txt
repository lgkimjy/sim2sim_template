cmake_minimum_required(VERSION 3.16)
project(sim2sim_template LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DCMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

set(SIM2SIM_MUJOCO_SOURCE_DIR "" CACHE PATH "Path to mujoco source tree")
if(NOT EXISTS "${SIM2SIM_MUJOCO_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "Pass -DSIM2SIM_MUJOCO_SOURCE_DIR=/path/to/mujoco")
endif()

option(MUJOCO_BUILD_EXAMPLES OFF)
option(MUJOCO_BUILD_SIMULATE ON)
option(MUJOCO_BUILD_TESTS OFF)
add_subdirectory("${SIM2SIM_MUJOCO_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/mujoco")
list(APPEND CMAKE_MODULE_PATH "${SIM2SIM_MUJOCO_SOURCE_DIR}/sample/cmake")
include(SampleDependencies)

find_package(Eigen3 REQUIRED)
include(cmake/OnnxRuntime.cmake)
find_package(yaml-cpp CONFIG REQUIRED)

add_executable(sim2sim_deploy
  src/main.cpp
  src/RobotStates.cpp
  include/Controller/OnnxParser/OnnxPolicy.cpp
  include/StateMachine/FSM_template.cpp
  include/StateMachine/StateMachineCtrl.cpp
  include/Interface/MuJoCo/MuJoCoSimulationBridge.cpp
  include/Interface/MuJoCo/SimulationInterface.cpp
  include/Interface/MuJoCo/Simulate.cc
)
target_include_directories(sim2sim_deploy PRIVATE
  include
  "${SIM2SIM_MUJOCO_SOURCE_DIR}/simulate"
  "${SIM2SIM_MUJOCO_SOURCE_DIR}/sample"
)
target_link_libraries(sim2sim_deploy PRIVATE
  Eigen3::Eigen 
  onnxruntime::onnxruntime 
  yaml-cpp::yaml-cpp
  libsimulate 
  mujoco::mujoco 
  glfw 
  Threads::Threads 
  lodepng
)
target_compile_options(sim2sim_deploy PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
set_target_properties(sim2sim_deploy PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

