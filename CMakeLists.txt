cmake_minimum_required (VERSION 3.0)
project(mujoco_planner)

set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(mujoco-nogl SHARED IMPORTED)
set_target_properties(mujoco-nogl PROPERTIES
  IMPORTED_LOCATION "$ENV{HOME}/.mujoco/mujoco200/bin/libmujoco200nogl.so"
  INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco200/include" )

add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
  IMPORTED_LOCATION "$ENV{HOME}/.mujoco/mujoco200/bin/libmujoco200.so"
  INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco200/include" )

add_library(glfw3 SHARED IMPORTED)
set_target_properties(glfw3 PROPERTIES
  IMPORTED_LOCATION "$ENV{HOME}/.mujoco/mujoco200/bin/libglfw.so.3"
  INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco200/include" )

add_executable(plan plan.cpp mujoco_wrapper.h mujoco_wrapper.cpp)
target_link_libraries(plan ompl)
target_link_libraries(plan Eigen3::Eigen)
target_link_libraries(plan mujoco-nogl)
target_link_libraries(plan Threads::Threads)
target_link_libraries(plan yaml-cpp)

add_executable(render_plan render_plan.cpp mujoco_wrapper.h mujoco_wrapper.cpp)
target_link_libraries(render_plan ompl)
target_link_libraries(render_plan Eigen3::Eigen)
target_link_libraries(render_plan mujoco)
target_link_libraries(render_plan Threads::Threads)
target_link_libraries(render_plan glfw3)
target_link_libraries(render_plan OpenGL::GL)
target_link_libraries(render_plan GLEW::GLEW)
