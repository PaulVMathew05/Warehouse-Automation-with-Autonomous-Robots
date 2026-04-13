cmake_minimum_required(VERSION 3.8)
project(warehouse_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# ── Install Python scripts ───────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  warehouse_simulation/standalone_sim.py
  warehouse_simulation/gazebo_sim.py
  warehouse_simulation/warehouse_manager.py
  warehouse_simulation/robot_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install resource directories ─────────────────────────────────
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# ── Register package (needed for get_package_share_directory) ────
ament_package()
