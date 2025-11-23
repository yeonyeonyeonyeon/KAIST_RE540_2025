cmake_minimum_required(VERSION 3.0.2)
project(final_map)

find_package(catkin REQUIRED)

catkin_package()

# Install scripts
install(PROGRAMS
  scripts/fix_world_paths.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install materials
install(DIRECTORY materials/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/materials
  PATTERN ".svn" EXCLUDE
)

# Install worlds
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE
)

# Install models
install(FILES
  model.config
  model.sdf
  final_map.dae
  Final_map.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

# Install figure directory
install(DIRECTORY figure/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/figure
  PATTERN ".svn" EXCLUDE
)

