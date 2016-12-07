cmake_minimum_required(VERSION 2.8.3)
project(jaco_description)

find_package(catkin REQUIRED)

catkin_package()

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(FILES
#    urdf/jaco.xacro
#    urdf/jaco_arm.xacro
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
#  )
# install(DIRECTORY meshes
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    FILES_MATCHING PATTERN "*.dae"
#  )
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
