cmake_minimum_required(VERSION 3.8)
project(arm_urdf)

find_package(ament_cmake REQUIRED)

# URDF + meshes + rviz config + launch files -> share/arm_urdf
install(DIRECTORY meshes urdf rviz launch config
  DESTINATION share/${PROJECT_NAME}
)

# joint slider GUI node -> lib/arm_urdf/joint_slider.py
install(PROGRAMS scripts/joint_slider.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
