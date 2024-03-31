cmake_minimum_required(VERSION 3.5)
project(ros2_turtle_gui)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(/usr/include/x86_64-linux-gnu/qt5)

# Configuration Qts
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES src/main.cpp src/qt/MainWindow.cpp)
set(UI_FILES src/qt/MainWindow.ui)

qt5_wrap_ui(UI_HEADERS ${UI_FILES})

add_executable(turtlesim_gui ${SOURCES} ${UI_HEADERS})

target_link_libraries(turtlesim_gui Qt5::Widgets ${rclcpp_LIBS})

ament_target_dependencies(turtlesim_gui rclcpp turtlesim geometry_msgs)

install(TARGETS
  turtlesim_gui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()