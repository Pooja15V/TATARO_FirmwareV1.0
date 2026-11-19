cmake_minimum_required(VERSION 3.8)
project(tatro_handeye_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ── Python executables ────────────────────────────────────────────────────
install(
  PROGRAMS
    scripts/tatro_calibration_poses.py
  RENAME tatro_calibration_poses
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_aruco_texture.py
  RENAME generate_aruco_texture
  DESTINATION lib/${PROJECT_NAME}
)

# ── Data directories (launch, worlds, models, rviz, scripts) ─────────────
install(
  DIRECTORY
    launch
    worlds
    models
    rviz
    scripts
  DESTINATION share/${PROJECT_NAME}
)

# ── ament resource index marker ───────────────────────────────────────────
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_package()
