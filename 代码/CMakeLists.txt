cmake_minimum_required(VERSION 2.8)

project(orbbec)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# orbbec openni
set(Include ${CMAKE_SOURCE_DIR}/Device/Include)
set(Redist ${CMAKE_SOURCE_DIR}/Device/Redist)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${Include})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Opencv
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# OpenMP
find_package( OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# cpu_tsdf
include_directories(Thirdparty/cpu_tsdf/include)

set(THIRD_PARTY_LIBS ${OpenCV_LIBS} ${PCL_LIBRARIES}
${GLUT_LIBRARY} ${OPENGL_LIBRARY}
${Redist}/libOpenNI2.so  ${Redist}/OpenNI2/Drivers/libOniFile.so ${Redist}/OpenNI2/Drivers/liborbbec.so ${Redist}/libDepthUtils.a ${Redist}/libSonixCamera.a
${PROJECT_SOURCE_DIR}/Thirdparty/cpu_tsdf/build/libcpu_tsdf.so
-lpthread
)

############ source and examples ###############
include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(src)
add_subdirectory(examples)