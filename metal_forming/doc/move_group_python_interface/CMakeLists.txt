install(PROGRAMS
  scripts/move_group_python_interface_tutorial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(PROGRAMS
  scripts/python_exp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(PROGRAMS
#  scripts/rectangle.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


install(PROGRAMS
  scripts/whatwa.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
