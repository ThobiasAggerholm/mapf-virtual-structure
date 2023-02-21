cmake_minimum_required(VERSION 3.1.0)
project(mapf-virtual-structure VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)

include(CTest)
enable_testing()

find_package( OpenCV REQUIRED )

include_directories("inc" ${OpenCV_INCLUDE_DIRS})
file(GLOB SOURCES "src/*.cpp")

add_executable(mapf-virtual-structure main.cpp ${SOURCES})

target_link_libraries( mapf-virtual-structure  ${OpenCV_LIBS} )


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
