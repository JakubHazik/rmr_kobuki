cmake_minimum_required(VERSION 3.10)
project(kobuki_project)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 REQUIRED COMPONENTS Gui Core Widgets)

#set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -Wall)
#add_compile_options(-O2)

# set source files
set(INCLUDE_FILES
        include/ui.h
        include/robot_interface.h
        include/regulator.h
        include/own_typedefs.h
        include/lidar_interface.h)
set(SOURCE_FILES
        src/ui.cpp
        src/robot_interface.cpp
        src/lidar_interface.cpp
        src/regulator.cpp)
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
        Qt5::Widgets)
