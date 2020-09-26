install(PROGRAMS
  scripts/move_group_python_interface_tutorial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(PROGRAMS
  scripts/3dof_vertical.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(PROGRAMS
#  scripts/rectangle.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


install(PROGRAMS
  scripts/6dof_horizontal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
