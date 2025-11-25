cmake_minimum_required(VERSION 3.10)
project(KinovaExample)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(KORTEX_API_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../examples/kortex_api")

# Set DURATION variable with default value of 20
if(NOT DEFINED DURATION)
    set(DURATION 20)
endif()

# Include directories for Kortex API
include_directories(
  ${KORTEX_API_DIR}/include
  ${KORTEX_API_DIR}/include/client
  ${KORTEX_API_DIR}/include/client_stubs
  ${KORTEX_API_DIR}/include/common
  ${KORTEX_API_DIR}/include/google
  ${KORTEX_API_DIR}/include/messages
  include
)

# Include your package headers
include_directories(
  include
)

# Add the executable
add_executable(kinova_example
  api_setup.cpp
  leader.cpp
  arm_core_functions.cpp
)

# Define compile flags
target_compile_definitions(kinova_example PRIVATE _OS_UNIX DURATION=${DURATION})

# Link against Kortex library
target_link_libraries(kinova_example
  ${KORTEX_API_DIR}/lib/release/libKortexApiCpp.a
  pthread
)

# Export executables
ament_target_dependencies(kinova_example rclcpp)

install(TARGETS
  kinova_example
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
