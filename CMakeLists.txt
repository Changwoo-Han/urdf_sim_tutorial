cmake_minimum_required(VERSION 3.5)
project(urdf_sim_tutorial)

find_package(ament_cmake REQUIRED)

# 기존 install 디렉토리 설치
install(DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# ✅ Python 노드 설치
install(PROGRAMS
  urdf_sim_tutorial/twist_to_stamped_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

