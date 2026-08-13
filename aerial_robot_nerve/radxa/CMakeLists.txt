cmake_minimum_required(VERSION 3.0.2)
project(radxa)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  spinal
  std_msgs
  std_srvs
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PERIPHERY REQUIRED libperiphery)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES radxa_hw
  CATKIN_DEPENDS roscpp spinal std_msgs std_srvs
  DEPENDS PERIPHERY
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PERIPHERY_INCLUDE_DIRS}
)
link_directories(${PERIPHERY_LIBRARY_DIRS})

add_library(radxa_hw
  src/ads1015.cpp
  src/dshot_driver.cpp
  src/i2c_driver.cpp
  src/icm20948.cpp
  src/pwm_driver.cpp
  src/radxa_board_io.cpp
)
target_link_libraries(radxa_hw ${PERIPHERY_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(radxa_hw ${catkin_EXPORTED_TARGETS})

add_executable(radxa_spinal_node src/radxa_spinal_node.cpp)
target_compile_definitions(radxa_spinal_node PRIVATE SIMULATION)
target_link_libraries(radxa_spinal_node
  radxa_hw
  ${catkin_LIBRARIES}
)
add_dependencies(radxa_spinal_node ${catkin_EXPORTED_TARGETS})

add_executable(imu_test src/imu_test.cpp)
target_link_libraries(imu_test radxa_hw ${catkin_LIBRARIES})

add_executable(adc_test src/adc_test.cpp)
target_link_libraries(adc_test radxa_hw ${catkin_LIBRARIES})

add_executable(dshot_test src/dshot_test.cpp)
target_link_libraries(dshot_test radxa_hw ${catkin_LIBRARIES})

add_executable(dshot_beacon_test src/dshot_beacon_test.cpp)
target_link_libraries(dshot_beacon_test radxa_hw ${catkin_LIBRARIES})

add_executable(dshot_arm_test src/dshot_arm_test.cpp)
target_link_libraries(dshot_arm_test radxa_hw ${catkin_LIBRARIES})

add_executable(pwm_test src/pwm_test.cpp)
target_link_libraries(pwm_test radxa_hw ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(radxa_driver_test test/driver_test.cpp)
  if(TARGET radxa_driver_test)
    target_link_libraries(radxa_driver_test radxa_hw ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS radxa_hw radxa_spinal_node imu_test adc_test dshot_test
  dshot_beacon_test dshot_arm_test pwm_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
