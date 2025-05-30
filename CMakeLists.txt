cmake_minimum_required(VERSION 3.10)
project(placo)

find_package(Threads)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(eigenpy REQUIRED)
find_package(jsoncpp REQUIRED)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Maybe this should not be done globally but only for placo's targets
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

if (NOT PYTHON_SITELIB)
set(PYTHON_SITELIB "lib")
endif()

if ("${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" STREQUAL "")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
endif()


find_package (Eigen3 3 REQUIRED NO_MODULE)

add_library(libplaco SHARED
    # Wrappers
    src/placo/model/robot_wrapper.cpp

    # Humanoid
    src/placo/humanoid/humanoid_robot.cpp
    src/placo/humanoid/humanoid_parameters.cpp
    src/placo/humanoid/foot_trajectory.cpp
    src/placo/humanoid/swing_foot.cpp
    src/placo/humanoid/swing_foot_quintic.cpp
    src/placo/humanoid/swing_foot_cubic.cpp
    src/placo/humanoid/kick.cpp
    src/placo/humanoid/footsteps_planner.cpp
    src/placo/humanoid/footsteps_planner_naive.cpp
    src/placo/humanoid/footsteps_planner_repetitive.cpp
    src/placo/humanoid/walk_pattern_generator.cpp
    src/placo/humanoid/walk_tasks.cpp
    src/placo/humanoid/lipm.cpp

    # Tools
    src/placo/tools/axises_mask.cpp
    src/placo/tools/utils.cpp
    src/placo/tools/prioritized.cpp
    src/placo/tools/cubic_spline.cpp
    src/placo/tools/cubic_spline_3d.cpp
    src/placo/tools/directions.cpp
    src/placo/tools/polynom.cpp
    src/placo/tools/segment.cpp

    # Problem formulation
    src/placo/problem/problem.cpp
    src/placo/problem/qp_error.cpp
    src/placo/problem/variable.cpp
    src/placo/problem/expression.cpp
    src/placo/problem/integrator.cpp
    src/placo/problem/problem_polynom.cpp
    src/placo/problem/constraint.cpp
    src/placo/problem/polygon_constraint.cpp
    src/placo/problem/sparsity.cpp

    # Kinematics QP solver
    src/placo/kinematics/kinematics_solver.cpp
    src/placo/kinematics/task.cpp
    src/placo/kinematics/position_task.cpp
    src/placo/kinematics/orientation_task.cpp
    src/placo/kinematics/centroidal_momentum_task.cpp
    src/placo/kinematics/frame_task.cpp
    src/placo/kinematics/relative_position_task.cpp
    src/placo/kinematics/relative_orientation_task.cpp
    src/placo/kinematics/relative_frame_task.cpp
    src/placo/kinematics/com_task.cpp
    src/placo/kinematics/distance_task.cpp
    src/placo/kinematics/joints_task.cpp
    src/placo/kinematics/gear_task.cpp
    src/placo/kinematics/wheel_task.cpp
    src/placo/kinematics/regularization_task.cpp
    src/placo/kinematics/manipulability_task.cpp
    src/placo/kinematics/kinetic_energy_regularization_task.cpp
    src/placo/kinematics/constraint.cpp
    src/placo/kinematics/avoid_self_collisions_constraint.cpp
    src/placo/kinematics/com_polygon_constraint.cpp
    src/placo/kinematics/joint_space_half_spaces_constraint.cpp
    src/placo/kinematics/cone_constraint.cpp
    src/placo/kinematics/yaw_constraint.cpp
    src/placo/kinematics/distance_constraint.cpp
    src/placo/kinematics/axis_align_task.cpp

    # Dynamics QP solver
    src/placo/dynamics/task.cpp
    src/placo/dynamics/position_task.cpp
    src/placo/dynamics/orientation_task.cpp
    src/placo/dynamics/frame_task.cpp
    src/placo/dynamics/relative_position_task.cpp
    src/placo/dynamics/relative_orientation_task.cpp
    src/placo/dynamics/relative_frame_task.cpp
    src/placo/dynamics/joints_task.cpp
    src/placo/dynamics/torque_task.cpp
    src/placo/dynamics/gear_task.cpp
    src/placo/dynamics/com_task.cpp
    src/placo/dynamics/contacts.cpp
    src/placo/dynamics/dynamics_solver.cpp
    src/placo/dynamics/constraint.cpp
    src/placo/dynamics/avoid_self_collisions_constraint.cpp

    ${PROTO_SRCS}
    ${PROTO_HDRS}  
)

target_include_directories(libplaco PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src> 
  ${PROTOBUF_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(libplaco PUBLIC
    Eigen3::Eigen
    Threads::Threads
    pinocchio::pinocchio
    eigenpy::eigenpy
    eiquadprog::eiquadprog
    jsoncpp_lib
    ${PROTOBUF_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)
target_compile_definitions(libplaco PUBLIC)

if(TARGET rhoban_utils)
  message("Placo: Rhoban utils is present, enabling it")
  target_compile_definitions(libplaco PUBLIC -DHAVE_RHOBAN_UTILS)
  target_link_libraries(libplaco PUBLIC rhoban_utils)
endif ()

set(CMAKE_SHARED_MODULE_PREFIX "")

find_package(Python3 REQUIRED)
find_package(Boost COMPONENTS python REQUIRED)
find_package(Python REQUIRED COMPONENTS Interpreter)

add_library(placo MODULE 
    bindings/expose-eigen.cpp
    bindings/expose-tools.cpp
    bindings/expose-problem.cpp
    bindings/expose-footsteps.cpp
    bindings/expose-robot-wrapper.cpp
    bindings/expose-parameters.cpp
    bindings/expose-kinematics.cpp
    bindings/expose-walk-pattern-generator.cpp
    bindings/expose-dynamics.cpp
    bindings/module.cpp
)
set_target_properties(placo PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PYTHON_SITELIB})
target_compile_definitions(placo PUBLIC -DBOOST_DISABLE_PRAGMA_MESSAGE)

target_link_libraries(placo ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} libplaco)
target_include_directories(placo PRIVATE ${PYTHON_INCLUDE_DIRS})

add_custom_command(
    TARGET placo POST_BUILD
    COMMAND doxystub
        --module placo
        --doxygen_directory "${CMAKE_CURRENT_SOURCE_DIR}"
        --output "${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}/placo.pyi"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}"
    COMMENT "Generating stubs..."
)

ADD_CUSTOM_TARGET(placo_utils_files ALL
                  COMMAND ${CMAKE_COMMAND} -E copy_directory
                  ${CMAKE_CURRENT_SOURCE_DIR}/python/placo_utils
                  ${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}/placo_utils
                  )

if (APPLE)
set_target_properties(placo PROPERTIES INSTALL_RPATH "@loader_path/../..")
set_target_properties(libplaco PROPERTIES INSTALL_RPATH "@loader_path")
else()
set_target_properties(placo PROPERTIES INSTALL_RPATH "\$ORIGIN/../..")
set_target_properties(libplaco PROPERTIES INSTALL_RPATH "\$ORIGIN")
endif()

install(TARGETS libplaco DESTINATION lib)
install(TARGETS placo DESTINATION ${PYTHON_SITELIB})
install(FILES ${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}/placo.pyi DESTINATION ${PYTHON_SITELIB})
install(DIRECTORY ${CMAKE_BINARY_DIR}/${PYTHON_SITELIB}/placo_utils DESTINATION ${PYTHON_SITELIB})
