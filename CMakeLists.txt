cmake_minimum_required(VERSION 3.8)
project(thermal_camera_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(monitor_interfaces REQUIRED)

set(HIK_SDK_ROOT_ARM "${CMAKE_CURRENT_SOURCE_DIR}/third_party/hik_sdk/arm64")
set(HIK_SDK_ROOT_X64 "${CMAKE_CURRENT_SOURCE_DIR}/third_party/hik_sdk/x64")

add_executable(hik_alarm_node src/hik_alarm_node.cpp)
ament_target_dependencies(hik_alarm_node rclcpp std_msgs monitor_interfaces)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(HIK_SDK_INC_DIR "${HIK_SDK_ROOT_ARM}/include")
  set(HIK_SDK_LIB_DIR "${HIK_SDK_ROOT_ARM}/lib")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
  set(HIK_SDK_INC_DIR "${HIK_SDK_ROOT_X64}/include")
  set(HIK_SDK_LIB_DIR "${HIK_SDK_ROOT_X64}/lib")
endif()

if(HIK_SDK_LIB_DIR)
  target_compile_definitions(hik_alarm_node PRIVATE USE_HIK_SDK=1)
  target_compile_definitions(hik_alarm_node PRIVATE
    HIK_SDK_LIB_DIR="${HIK_SDK_LIB_DIR}"
    HIK_SDK_COM_DIR="${HIK_SDK_LIB_DIR}/HCNetSDKCom"
    HIK_SDK_SSL_PATH="${HIK_SDK_LIB_DIR}/libssl.so.3"
    HIK_SDK_CRYPTO_PATH="${HIK_SDK_LIB_DIR}/libcrypto.so.3"
  )
  target_include_directories(hik_alarm_node SYSTEM PRIVATE
    ${HIK_SDK_INC_DIR}
  )
  target_compile_options(hik_alarm_node PRIVATE -Wno-comment -Wno-pedantic)
  target_link_directories(hik_alarm_node PRIVATE
    ${HIK_SDK_LIB_DIR}
  )
  target_link_libraries(hik_alarm_node hcnetsdk)
  set_target_properties(hik_alarm_node PROPERTIES
    BUILD_RPATH "${HIK_SDK_LIB_DIR};${HIK_SDK_LIB_DIR}/HCNetSDKCom"
    INSTALL_RPATH "${HIK_SDK_LIB_DIR};${HIK_SDK_LIB_DIR}/HCNetSDKCom"
  )
endif()

install(TARGETS hik_alarm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY third_party
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
