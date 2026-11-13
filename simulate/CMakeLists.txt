cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco_g1)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)


include_directories(
  mujoco/include
  mujoco/simulate
  src/lodepng
)
link_directories(mujoco/lib)

# MuJoCo's managed viewer omits the custom geometry list in Sync(). Generate
# the small rendering extension in the build tree, keeping upstream untouched.
set(SIMULATE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/mujoco/simulate/simulate.cc")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SIMULATE_SOURCE}")
file(READ "${SIMULATE_SOURCE}" SIMULATE_CODE)
set(SCENE_UPDATE "mjv_updateScene(m_, d_, &this->opt, &this->pert, &this->cam, mjCAT_ALL, &this->scn);")
string(FIND "${SIMULATE_CODE}" "${SCENE_UPDATE}" SCENE_UPDATE_POSITION)
if(SCENE_UPDATE_POSITION EQUAL -1)
  message(FATAL_ERROR "Unsupported MuJoCo simulate.cc: managed scene update not found")
endif()
string(REPLACE "${SCENE_UPDATE}" "${SCENE_UPDATE}
    const int count = std::min(static_cast<int>(user_scn_geoms_.size()), scn.maxgeom - scn.ngeom);
    if (count > 0) {
      std::copy_n(user_scn_geoms_.data(), count, scn.geoms + scn.ngeom);
      scn.ngeom += count;
    }" SIMULATE_CODE "${SIMULATE_CODE}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/simulate_with_user_geoms.cc" "${SIMULATE_CODE}")

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  mujoco/simulate/glfw_*.cc
  mujoco/simulate/platform_*.cc
  src/lodepng/lodepng.cpp
)
list(APPEND SIM_SRC "${CMAKE_CURRENT_BINARY_DIR}/simulate_with_user_geoms.cc")

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

include(CTest)
if(BUILD_TESTING)
  add_executable(g1_height_map_test tests/height_map_test.cc)
  target_include_directories(g1_height_map_test PRIVATE src)
  add_test(NAME g1_height_map COMMAND g1_height_map_test "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/g1/flat.xml")
  add_executable(g1_model_selection_test tests/model_selection_test.cc)
  target_include_directories(g1_model_selection_test PRIVATE src)
  add_test(NAME g1_model_selection
    COMMAND g1_model_selection_test "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/g1"
      "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/g1/urdf/g1_29dof_mode_15_unitree_model_collision.urdf")
endif()
