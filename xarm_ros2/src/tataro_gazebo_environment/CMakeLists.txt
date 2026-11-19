cmake_minimum_required(VERSION 3.8)
project(tataro_gazebo_environment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install world files
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# Install URDF/Xacro files
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install RViz config
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# Install scripts
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                   GROUP_READ GROUP_EXECUTE
                   WORLD_READ WORLD_EXECUTE)

ament_package()
