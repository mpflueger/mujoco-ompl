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
find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options)

add_library(mujoco-nogl SHARED IMPORTED)
add_library(mujoco SHARED IMPORTED)
add_library(glfw3 SHARED IMPORTED)

if(APPLE)
	set_target_properties(mujoco-nogl PROPERTIES
	  IMPORTED_LOCATION "$ENV{HOME}/.mujoco/mujoco200/bin/libmujoco200nogl.dylib"
	  INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco200/include" )
	
	set_target_properties(mujoco PROPERTIES
	  IMPORTED_LOCATION "$ENV{HOME}/.mujoco/mujoco200/bin/libmujoco200.dylib"
	  INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco200/include" )
	
	set_target_properties(glfw3 PROPERTIES
	  IMPORTED_LOCATION "$ENV{HOME}/.mujoco/mujoco200/bin/libglfw.3.dylib"
	  INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco200/include" )	
elseif(UNIX)
	set_target_properties(mujoco-nogl PROPERTIES
	  IMPORTED_LOCATION "$ENV{HOME}/.mujoco/mujoco200/bin/libmujoco200nogl.so"
	  INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco200/include" )

	set_target_properties(mujoco PROPERTIES
	  IMPORTED_LOCATION "$ENV{HOME}/.mujoco/mujoco200/bin/libmujoco200.so"
	  INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco200/include" )

	set_target_properties(glfw3 PROPERTIES
	  IMPORTED_LOCATION "$ENV{HOME}/.mujoco/mujoco200/bin/libglfw.so.3"
	  INTERFACE_INCLUDE_DIRECTORIES "$ENV{HOME}/.mujoco/mujoco200/include" )
endif()

include_directories(${OMPL_INCLUDE_DIRS}, ${Boost_INCLUDE_DIR})
include_directories(3rd_party/include)

add_executable(plan
    plan.cpp
    mujoco_wrapper.h
    mujoco_wrapper.cpp
    mujoco_ompl_interface.h
    mujoco_ompl_interface.cpp
    compound_state_projector.h)
target_link_libraries(plan ${OMPL_LIBRARIES})
target_link_libraries(plan Eigen3::Eigen)
target_link_libraries(plan mujoco-nogl)
target_link_libraries(plan Threads::Threads)
target_link_libraries(plan yaml-cpp)

add_executable(render_plan
    render_plan.cpp
    mujoco_wrapper.h
    mujoco_wrapper.cpp
    mujoco_ompl_interface.h
    mujoco_ompl_interface.cpp
    compound_state_projector.h)
target_link_libraries(render_plan ${OMPL_LIBRARIES})
target_link_libraries(render_plan Eigen3::Eigen)
target_link_libraries(render_plan mujoco)
target_link_libraries(render_plan Threads::Threads)
target_link_libraries(render_plan glfw3)
target_link_libraries(render_plan OpenGL::GL)
target_link_libraries(render_plan GLEW::GLEW)

add_executable(smooth_plan
    smooth_plan.cpp
    mujoco_wrapper.h
    mujoco_wrapper.cpp
    mujoco_ompl_interface.h
    mujoco_ompl_interface.cpp
    compound_state_projector.h)
target_link_libraries(smooth_plan ${OMPL_LIBRARIES})
target_link_libraries(smooth_plan Eigen3::Eigen)
target_link_libraries(smooth_plan mujoco-nogl)
target_link_libraries(smooth_plan Threads::Threads)
target_link_libraries(smooth_plan yaml-cpp)

