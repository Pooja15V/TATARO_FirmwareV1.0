cmake_minimum_required(VERSION 3.8)
project(tataro_packages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

# Install Python scripts
install(PROGRAMS
        tataro_touch_marker_centre
        tataro_calibration_diagnostic.py
        tataro_charuco_touch.py
        tataro_aruco7x7_tf_publisher.py
        tataro_table_crop_node.py
        tataro_marker40_touch.py
        tataro_marker40_touch_loop.py
        tataro_marker40_touch_loopV1.0.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
