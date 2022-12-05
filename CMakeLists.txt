cmake_minimum_required (VERSION 3.0)
project(mujoco_planner)

set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
if(NOT OMPL_INCLUDE_DIRS)
  find_package(ompl REQUIRED)
endif()

find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options)

# Debugging info
message(STATUS "-------- DEBUGGING ------------")
get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
  message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
message(STATUS "-------- END DEBUGGING --------")


# linking mujoco
include_directories(${CMAKE_SOURCE_DIR}/mujoco-2.3.0/include)
link_directories(${CMAKE_SOURCE_DIR}/mujoco-2.3.0/lib)


include_directories(${OMPL_INCLUDE_DIRS}, ${Boost_INCLUDE_DIR})
include_directories(3rd_party/include)
include_directories(include)

add_executable(plan plan.cpp
		src/mujoco_wrapper.cpp
		src/mujoco_ompl_interface.cpp
		)
target_link_libraries(plan ${OMPL_LIBRARIES})
target_link_libraries(plan Eigen3::Eigen)
target_link_libraries(plan mujoco)
target_link_libraries(plan Threads::Threads)
target_link_libraries(plan yaml-cpp)
target_link_libraries(plan Boost::filesystem)

add_executable(render_plan render_plan.cpp
		src/mujoco_wrapper.cpp
		src/mujoco_ompl_interface.cpp
		)
target_link_libraries(render_plan ${OMPL_LIBRARIES})
target_link_libraries(render_plan Eigen3::Eigen)
target_link_libraries(render_plan mujoco)
target_link_libraries(render_plan Threads::Threads)
target_link_libraries(render_plan glfw)

add_executable(plan_kinematic plan_kinematic.cpp
		src/mujoco_wrapper.cpp
		src/mujoco_ompl_interface.cpp
		)
target_link_libraries(plan_kinematic ${OMPL_LIBRARIES})
target_link_libraries(plan_kinematic Eigen3::Eigen)
target_link_libraries(plan_kinematic mujoco)
target_link_libraries(plan_kinematic Threads::Threads)
target_link_libraries(plan_kinematic yaml-cpp)

add_executable(render_plan_kinematic render_plan_kinematic.cpp
		src/mujoco_wrapper.cpp
		src/mujoco_ompl_interface.cpp
		)
target_link_libraries(render_plan_kinematic ${OMPL_LIBRARIES})
target_link_libraries(render_plan_kinematic Eigen3::Eigen)
target_link_libraries(render_plan_kinematic mujoco)
target_link_libraries(render_plan_kinematic Threads::Threads)
target_link_libraries(render_plan_kinematic glfw)

add_executable(smooth_plan smooth_plan.cpp
		src/mujoco_wrapper.cpp
		src/mujoco_ompl_interface.cpp
		)
target_link_libraries(smooth_plan ${OMPL_LIBRARIES})
target_link_libraries(smooth_plan Eigen3::Eigen)
target_link_libraries(smooth_plan mujoco)
target_link_libraries(smooth_plan Threads::Threads)
target_link_libraries(smooth_plan yaml-cpp)

add_library(mujoco_ompl_nogl SHARED
		src/mujoco_wrapper.cpp
		src/mujoco_ompl_interface.cpp)
target_include_directories(mujoco_ompl_nogl PUBLIC include)
target_link_libraries(mujoco_ompl_nogl ${OMPL_LIBRARIES})
target_link_libraries(mujoco_ompl_nogl Eigen3::Eigen)
target_link_libraries(mujoco_ompl_nogl mujoco)
target_link_libraries(mujoco_ompl_nogl Threads::Threads)
target_link_libraries(mujoco_ompl_nogl yaml-cpp)

add_library(mujoco_ompl SHARED
		src/mujoco_wrapper.cpp
		src/mujoco_ompl_interface.cpp)
target_include_directories(mujoco_ompl PUBLIC include)
target_link_libraries(mujoco_ompl ${OMPL_LIBRARIES})
target_link_libraries(mujoco_ompl Eigen3::Eigen)
target_link_libraries(mujoco_ompl mujoco)
target_link_libraries(mujoco_ompl Threads::Threads)
target_link_libraries(mujoco_ompl yaml-cpp)
