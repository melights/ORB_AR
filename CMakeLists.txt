cmake_minimum_required(VERSION 2.8)
project(BOB_AR)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH /home/long/ORB_AR/cmake_modules)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/temp)

find_package( OpenCV REQUIRED )
find_package( OpenGL REQUIRED )
include_directories(${OPENGL_INCLUDE_DIR})

find_package( GLUT REQUIRED )
include_directories(${GLUT_INCLUDE_DIR})

find_package( GLEW REQUIRED )
include_directories(${GLEW_INCLUDE_DIR})

find_package( GLM REQUIRED )
include_directories(${GLM_INCLUDE_DIR})

find_package( PNG REQUIRED )
include_directories(${PNG_INCLUDE_DIR})

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

find_package(X11)

LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})


find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( ADDITIONAL_LIBS 
     -L/usr/lib/x86_64-linux-gnu
     -lGLEW
     -lGLU 
     -lglfw3
     -pthread
     -lrt
     -lXinerama
     -lXrandr
     -lXxf86vm
     -lXi
     -lXcursor
)



set(SOURCE_FILES
        src/main.cpp
        src/shader.cpp
        src/controls.cpp
        src/texture.cpp
        src/objloader.cpp
        src/orbslam.cpp
        #temp/track_chessboard/track_chessboard.cpp
        #temp/track_chessboard/track_chessboard.hpp
        src/planar.cpp)
add_executable(BOB_AR ${SOURCE_FILES})
target_link_libraries(BOB_AR
        ${X11_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${OpenCV_LIBS}
        ${GLEW_LIBRARIES}
        ${GLFW_LIBRARIES}
        ${PNG_LIBRARY}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/lib/libORB_SLAM2.so
        ${ADDITIONAL_LIBS})
