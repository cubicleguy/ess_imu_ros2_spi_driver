cmake_minimum_required(VERSION 3.5)
project(epson_imu_spi_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify DEBUG macro to enable any debug code by adding "-DDEBUG" in add_definitions()
#add_definitions(-DDEBUG)

# Refer to the README_src.md inside the src folder for more details about IMU macro
# Uncomment the imu_model to build for
# NOTE: Change IMU models require rebuilding binaries with colcon
#set(imu_model "G325PDF0")
#set(imu_model "G325PDF1")
#set(imu_model "G365PDF0")
set(imu_model "G365PDF1")
#set(imu_model "G365PDC0")
#set(imu_model "G365PDC1")
#set(imu_model "G370PDF0")
#set(imu_model "G370PDF1")
#set(imu_model "G354")
#set(imu_model "G364PDCA")
#set(imu_model "G364PDC0")
#set(imu_model "G320")
#set(imu_model "V340")
add_definitions(-D${imu_model})
message([STATUS] "---- Building for IMU Model: ${imu_model}")

# support RaspberryPi only
set(platform "RPI")
add_definitions(-D${platform})
message([STATUS] "---- Building for platform: ${platform}")

# support SPI only
set(interface "SPI")
add_definitions(-D${interface})
message([STATUS] "---- Building for interface: ${interface}")


# Setting macro NATIVE_QUAT enables hardware quaternion output function in G325/G365 only 
# and has no effect for any other IMU models
# If not defined then quaternion output is emulated by converting from G325/G365 Euler output using ROS tf2 library
# NATIVE_QUAT is recommended over software conversion from euler output
add_definitions(-DNATIVE_QUAT)


# Create filelist macro of C sources for library
set(lib_sources
  src/hcl_rpi.c
  src/hcl_gpio_rpi.c
  src/hcl_spi_rpi.c
  src/sensor_epsonCommon.c
  src/sensor_epsonSpi.c
)

# Add IMU model specific C source to filelist macro
if (imu_model STREQUAL "G365PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G365PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G365PDC0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G365PDC1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG365.c)
elseif (imu_model STREQUAL "G370PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (imu_model STREQUAL "G370PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG370.c)
elseif (imu_model STREQUAL "G325PDF0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG325.c)
elseif (imu_model STREQUAL "G325PDF1")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG325.c)
elseif (imu_model STREQUAL "G354")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG354.c)
elseif (imu_model STREQUAL "G364PDCA")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (imu_model STREQUAL "G364PDC0")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG364.c)
elseif (imu_model STREQUAL "G320")
  set(lib_sources ${lib_sources}
    src/sensor_epsonG320.c)
elseif (imu_model STREQUAL "V340")
  set(lib_sources ${lib_sources}
    src/sensor_epsonV340.c)
else()
  message([FATAL_ERROR] "**** Invalid IMU Model")
endif()

# Declare static library for Epson IMU functions
add_library(epson_imu_spi_driver_lib STATIC
  ${lib_sources}
)

# Determine location of wiringPi library on the host system
# Needed if building on Raspberry Pi platform
find_library(wiringPi_LIB NAMES wiringPi)

# Declare C++ executable
add_executable(imu_node src/epson_imu_node.cpp)

# Link Epson IMU C Library to the ROS node
target_link_libraries(imu_node
  epson_imu_spi_driver_lib
  ${wiringPi_LIB}
  crypt
  rt
  pthread
)

# Link C++ Library to the ROS node
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Mark executables and/or libraries for installation
install(
   TARGETS imu_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Mark directories for installation
install(
   DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)

#ament_export_include_directories(include)
#ament_export_dependencies(std_msgs)
ament_package()
