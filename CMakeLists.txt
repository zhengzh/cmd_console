cmake_minimum_required(VERSION 2.8.3)
project(cmd_console)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
# add_executable(we_console_cin we_console_cin.cpp)

catkin_package(
  LIBRARIES we_console
  #  CATKIN_DEPENDS roscpp
  DEPENDS libedit-dev
  )

include_directories(${catkin_INCLUDE_DIRS})
add_executable(we_console we_console.cpp)
target_link_libraries(we_console
  ${catkin_LIBRARIES}
  edit
)
# target_link_libraries(we_console_cin
#   ${catkin_LIBRARIES}
# )


install(TARGETS we_console
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    PROGRAMS
      scripts/console
      scripts/console.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})