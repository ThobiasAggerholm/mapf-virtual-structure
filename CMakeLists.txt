cmake_minimum_required(VERSION 3.1.0)
project(mapf-virtual-structure VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)

include(CTest)
enable_testing()

find_package( OpenCV REQUIRED )
find_package(TBB REQUIRED)


include_directories("inc" ${OpenCV_INCLUDE_DIRS})
file(GLOB SOURCES "src/*.cpp"  "test_files/src/*.cpp")

add_executable(mapf-virtual-structure main.cpp ${SOURCES})

target_link_libraries( mapf-virtual-structure PUBLIC ${OpenCV_LIBS})
target_link_libraries( mapf-virtual-structure PRIVATE TBB::tbb)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
