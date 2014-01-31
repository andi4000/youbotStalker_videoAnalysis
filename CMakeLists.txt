cmake_minimum_required(VERSION 2.8)
project(youbotStalker_videoAnalysis)
set(EXECUTABLE_OUTPUT_PATH ../bin)
set(LIBRARY_OUTPUT_PATH ../lib)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(extract src/main.cpp)
target_link_libraries(extract ${OpenCV_LIBS})

