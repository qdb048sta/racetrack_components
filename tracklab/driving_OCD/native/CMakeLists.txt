cmake_minimum_required(VERSION 3.18)
project(tracklab_ocd LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT OCD_SOURCE_DIR)
  message(FATAL_ERROR "Set OCD_SOURCE_DIR to the pinned Open-Car-Dynamics checkout")
endif()
set(COMMON "${OCD_SOURCE_DIR}/submodules/tam__common")
set(PARAM "${COMMON}/submodules/parameter_management/param_management_cpp")
file(GLOB OCD_PACKAGES "${OCD_SOURCE_DIR}/lib/*")
add_library(tracklab_ocd SHARED bridge.cpp
  "${OCD_SOURCE_DIR}/lib/ocd_aerodynamics_models_cpp/src/default.cpp"
  "${OCD_SOURCE_DIR}/lib/ocd_tire_models_cpp/src/linear.cpp"
  "${OCD_SOURCE_DIR}/lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_model.cpp"
  "${OCD_SOURCE_DIR}/lib/ocd_drivetrain_wheel_torque_cpp/src/drivetrain_direct_torque_eqns.cpp"
  "${OCD_SOURCE_DIR}/lib/ocd_steering_actuator_pt1_cpp/src/steering_actuator_pt1_model.cpp"
  "${PARAM}/src/param_manager_composer.cpp"
  "${PARAM}/src/init_backend.cpp"
  "${PARAM}/src/initialization.cpp"
  "${PARAM}/src/param_reference_manager.cpp"
  "${PARAM}/src/param_value_manager.cpp")
foreach(PACKAGE ${OCD_PACKAGES})
  if(IS_DIRECTORY "${PACKAGE}/include")
    target_include_directories(tracklab_ocd PRIVATE "${PACKAGE}/include")
  endif()
endforeach()
target_include_directories(tracklab_ocd PRIVATE
  "${PARAM}/include"
  "${COMMON}/submodules/time_series_logging/tsl_logger_cpp/include"
  "${COMMON}/tum_helpers_cpp/include"
  "${COMMON}/tum_types_cpp/include")
if(OCD_EIGEN_INCLUDE_DIR)
  target_include_directories(tracklab_ocd PRIVATE "${OCD_EIGEN_INCLUDE_DIR}")
endif()
if(MINGW)
  target_link_options(tracklab_ocd PRIVATE -static-libgcc -static-libstdc++ -static)
elseif(UNIX AND NOT APPLE)
  target_link_options(tracklab_ocd PRIVATE -Wl,--no-undefined)
endif()
set_target_properties(tracklab_ocd PROPERTIES PREFIX "" WINDOWS_EXPORT_ALL_SYMBOLS OFF)
