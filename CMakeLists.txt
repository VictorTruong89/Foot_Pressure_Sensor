cmake_minimum_required(VERSION 3.17)

project(TestFootSensor)

set(CMAKE_STANDARD_CXX 11)

# Add relevant code of Serial Comm & Matrix Calculation & RealTimeSaveFile
set (SRC_DIR ${PROJECT_SOURCE_DIR}/src)
file (GLOB SOURCES ${SRC_DIR}/*.cpp)
file (GLOB HEADERS ${SRC_DIR}/*.hpp)

# Add Eigen library
include_directories("D:/Libraries/eigen-3.3.7")


add_executable( ${PROJECT_NAME} main.cpp ${SOURCES} ${HEADERS} )

target_include_directories( ${PROJECT_NAME} PUBLIC ${PROJECT_BINARY_DIR} ${SRC_DIR} )


