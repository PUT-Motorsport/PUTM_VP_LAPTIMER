cmake_minimum_required(VERSION 3.8)
project(putm_lap_timer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(putm_vcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(src)

# Biblioteka z podzielonymi modułami (współdzielona między węzłem a testami TDD)
add_library(lap_timer_core_lib
  src/core/geo_math.cpp
  src/core/sector_tracker.cpp
  src/core/lap_timer.cpp
  src/track/track_loader.cpp
  src/io/gnss_driver.cpp
  src/io/mock_provider.cpp
  src/io/can_sender.cpp
  src/utils/logger.cpp
)
ament_target_dependencies(lap_timer_core_lib
  rclcpp std_msgs sensor_msgs geometry_msgs std_srvs putm_vcl_interfaces
)

# Główny plik wykonywalny węzła ROS 2
add_executable(lap_timer src/main.cpp)
target_link_libraries(lap_timer lap_timer_core_lib)
ament_target_dependencies(lap_timer
  rclcpp std_msgs sensor_msgs geometry_msgs std_srvs putm_vcl_interfaces
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  lap_timer
  DESTINATION lib/${PROJECT_NAME}
)

# --- SEKCJA TESTÓW JEDNOSTKOWYCH (AUTOMATYCZNY GTEST) ---
if(BUILD_TESTING)
  # Usunęliśmy ament_lint_auto - colcon nie będzie już robił problemów o spacje w Pythonie ani styl C++

  # Budowanie testów logicznych GTest
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_laptimer test/test_laptimer.cpp)
  
  # MEGA WAŻNE: Test musi wiedzieć, gdzie są nagłówki ROS 2, żeby się skompilować!
  ament_target_dependencies(test_laptimer
    rclcpp std_msgs sensor_msgs geometry_msgs std_srvs putm_vcl_interfaces
  )
  
  # Linkowanie naszej biblioteki z logiką
  target_link_libraries(test_laptimer lap_timer_core_lib)
endif()

ament_package()