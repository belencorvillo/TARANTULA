cmake_minimum_required(VERSION 3.16)

project(TARANTULA_UI VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS Widgets REQUIRED)

add_executable(TARANTULA_UI
    main.cpp
    MainWindow.cpp
    MainWindow.h
    MainWindow.ui
    SpiderWidget.cpp
    SpiderWidget.h
    MotorController.cpp
    MotorController.h
    Leg.cpp
    Leg.h
    Tarantula.cpp
    Tarantula.h
    SimpleSerial.cpp
    SimpleSerial.h
    SteadyWinMotor.cpp
    SteadyWinMotor.h
    WaveshareInterface.cpp
    WaveshareInterface.h
    Config.h
)

target_link_libraries(TARANTULA_UI PRIVATE Qt6::Widgets)
target_include_directories(TARANTULA_UI PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Set WIN32 definition since it was present in the VS project
target_compile_definitions(TARANTULA_UI PRIVATE WIN32 _CONSOLE)
