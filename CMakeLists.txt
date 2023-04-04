project(placo)
cmake_minimum_required(VERSION 3.10)

find_package(Threads)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(eigenpy REQUIRED)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package (Eigen3 3 REQUIRED NO_MODULE)

add_library(libplaco
    src/placo/footsteps/footsteps_planner.cpp
    src/placo/footsteps/footsteps_planner_naive.cpp
    src/placo/footsteps/footsteps_planner_repetitive.cpp
    src/placo/planning/jerk_planner.cpp
    src/placo/planning/solver_task_holder.cpp
    src/placo/planning/walk_pattern_generator.cpp
    src/placo/planning/swing_foot.cpp
    src/placo/planning/swing_foot_quintic.cpp
    src/placo/model/robot_wrapper.cpp
    src/placo/model/humanoid_robot.cpp
    src/placo/model/humanoid_parameters.cpp
    src/placo/control/kinematics_solver.cpp
    src/placo/control/task.cpp
    src/placo/control/position_task.cpp
    src/placo/control/orientation_task.cpp
    src/placo/control/centroidal_momentum_task.cpp
    src/placo/control/frame_task.cpp
    src/placo/control/pose_task.cpp
    src/placo/control/relative_position_task.cpp
    src/placo/control/relative_orientation_task.cpp
    src/placo/control/relative_frame_task.cpp
    src/placo/control/relative_pose_task.cpp
    src/placo/control/com_task.cpp
    src/placo/control/axis_align_task.cpp
    src/placo/control/axis_plane_task.cpp
    src/placo/control/distance_task.cpp
    src/placo/control/joint_task.cpp
    src/placo/control/joints_task.cpp
    src/placo/control/regularization_task.cpp
    src/placo/problem/problem.cpp
    src/placo/problem/variable.cpp
    src/placo/problem/expression.cpp
    src/placo/problem/integrator.cpp
    src/placo/problem/constraint.cpp
    src/placo/utils.cpp
)

target_include_directories(libplaco PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src> 
  ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(libplaco PUBLIC
    CLI11::CLI11
    Eigen3::Eigen
    Threads::Threads
    pinocchio::pinocchio
    eigenpy::eigenpy
    rhoban_utils
    libzmq
    cppzmq
    eiquadprog::eiquadprog
  ${PROTOBUF_LIBRARIES}
)
target_compile_definitions(libplaco PUBLIC -DMOTORS_CONFIGURATION_${MOTORS_CONFIGURATION}
  -DPINOCCHIO_WITH_HPP_FCL)

set(CMAKE_SHARED_MODULE_PREFIX "")

find_package(PythonLibs REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)

add_library(placo MODULE 
    bindings/expose-eigen.cpp
    bindings/expose-utils.cpp
    bindings/expose-footsteps.cpp
    bindings/expose-jerk-planner.cpp
    bindings/expose-robot-wrapper.cpp
    bindings/expose-parameters.cpp
    bindings/expose-kinematics.cpp
    bindings/expose-walk-pattern-generator.cpp
    bindings/expose-problem.cpp
    bindings/module.cpp
)
target_link_libraries(placo ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} libplaco)
target_include_directories(placo PRIVATE ${PYTHON_INCLUDE_DIRS})

add_custom_command(TARGET placo POST_BUILD
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/stubs.py > placo.pyi
    WORKING_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    COMMENT "Generating stubs..."
)
