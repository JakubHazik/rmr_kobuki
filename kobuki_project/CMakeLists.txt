cmake_minimum_required(VERSION 3.10)
project(kobuki_project)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 REQUIRED COMPONENTS Gui Core Widgets)
find_package(OpenCV REQUIRED)
find_package(Boost 1.58.0 REQUIRED COMPONENTS signals)

#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wall)
#add_compile_options(-O2)

# set source files
set(INCLUDE_FILES
        include/ui.h
        include/robot_interface.h
        include/regulator.h
        include/global_planner.h
        include/robot_map.h
        include/lidar_interface.h
        include/lidar.h
        include/kobuki.h
        include/visualizer.h
        include/local_planner.h)
set(SOURCE_FILES
        src/ui.cpp
        src/robot_interface.cpp
        src/regulator.cpp
        src/robot_map.cpp
        src/global_planner.cpp
        src/lidar_interface.cpp
        src/lidar.cpp
        src/kobuki.cpp
        src/visualizer.cpp
        src/local_planner.cpp)
set(UI_SOURCES
        ui/mainwindow.ui)

qt5_wrap_ui(UI_GENERATED_HEADERS ${UI_SOURCES})

add_executable(kobuki
        src/main.cpp
        ${INCLUDE_FILES}
        ${SOURCE_FILES}
        ${UI_GENERATED_HEADERS})

target_link_libraries(kobuki
        pthread
        Qt5::Core
        Qt5::Widgets
        ${OpenCV_LIBS}
        Boost::signals)