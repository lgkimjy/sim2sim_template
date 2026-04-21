cmake_minimum_required(VERSION 3.16)

project(sim2sim_template LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_definitions(CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

set(SIM2SIM_MUJOCO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/3rd-parties/mujoco")

if(NOT EXISTS "${SIM2SIM_MUJOCO_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "MuJoCo source tree not found.\n"
    "Clone https://github.com/google-deepmind/mujoco into:\n"
    "  include/3rd-parties/mujoco, 3.2.6 tested "
    "  git clone --branch 3.2.6 --depth 1 https://github.com/google-deepmind/mujoco.git include/3rd-parties/mujoco "
  )
endif()

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "Build MuJoCo examples" FORCE)
set(MUJOCO_BUILD_SIMULATE ON CACHE BOOL "Build MuJoCo simulate library" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "Build MuJoCo tests" FORCE)
add_subdirectory("${SIM2SIM_MUJOCO_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/mujoco")

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
include(cmake/OnnxRuntime.cmake)
find_package(yaml-cpp CONFIG REQUIRED)

file(GLOB FSM_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/StateMachine/FSM_*.cpp"
)

add_executable(sim2sim_deploy
  src/main.cpp
  include/Controller/OnnxParser/OnnxPolicy.cpp
  ${FSM_SOURCES}
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
  mujoco::libsimulate
  mujoco::mujoco 
  glfw 
  Threads::Threads 
  lodepng
)
target_compile_options(sim2sim_deploy PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
set_target_properties(sim2sim_deploy PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
