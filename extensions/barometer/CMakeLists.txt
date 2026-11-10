find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_add_library(glim_barometer_extension SHARED
  src/barometer_extension.cpp)
target_include_directories(glim_barometer_extension PRIVATE include ${GTSAM_INCLUDE_DIRS})
target_link_libraries(glim_barometer_extension
  glim::glim
  gtsam
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  ${GTSAM_LIBRARIES})

if(BUILD_TESTING)
  add_executable(test_barometer_model test/test_barometer_model.cpp)
  target_compile_options(test_barometer_model PRIVATE -UNDEBUG)
  target_include_directories(test_barometer_model PRIVATE include ${GTSAM_INCLUDE_DIRS})
  target_link_libraries(test_barometer_model PRIVATE glim::glim Eigen3::Eigen gtsam ${GTSAM_LIBRARIES})
  add_test(NAME barometer_model COMMAND test_barometer_model)
endif()

install(TARGETS glim_barometer_extension LIBRARY DESTINATION lib)
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME}/extensions/barometer)
