cmake_minimum_required(VERSION 3.5)
project(ati_wireless_ft)


find_package(ament_cmake REQUIRED)
  # eigen
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
# find_package(tf2_conversions REQUIRED)
find_package(tf2_bullet REQUIRED)
find_package(Eigen3 REQUIRED)

set(deps 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_bullet
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# include_directories(
#   include
#   $(Boost_INCLUDE_DIRS)
#   $(Eigen_INCLUDE_DIRS)
# )

## Declare a C++ library
# add_library(tams_wireless_ft
#   src/${PROJECT_NAME}/tams_wireless_ft.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(tams_wireless_ft ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(tams_wireless_ft src/tams_wireless_ft.cpp)

#add_executable(tams_wireless_ft src/PolhemusTest1.cpp src/PiTracker.cpp )
#target_link_libraries(tams_wireless_ft
#  ${catkin_LIBRARIES} usb-1.0 pthread
#)

add_executable(${PROJECT_NAME} src/wireless_ft.cpp)

# target_include_directories(${PROJECT_NAME}
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

ament_target_dependencies(${PROJECT_NAME} 
  ${deps} Eigen3 pthread
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(tams_wireless_ft ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(tams_wireless_ft
#   ${catkin_LIBRARIES}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tams_wireless_ft tams_wireless_ft
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tams_wireless_ft.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_package()
