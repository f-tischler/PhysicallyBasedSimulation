cmake_minimum_required(VERSION 3.5.1)
project(Assignment3 LANGUAGES CXX)

#set(CMAKE_BUILD_TYPE Debug)

if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest /wd4141")
	set(CMAKE_CXX_FLAGS_RELEASE "/Ox /Ot /DNDEBUG")
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

set(SOURCE_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/Exercise.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Fluid2D.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/FluidSim.cpp
)

add_executable(FluidSim ${SOURCE_FILES})

target_include_directories(FluidSim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(WIN32)
	find_path(FREEGLUT_INCLUDE_DIR gl/freeglut.h)
	find_library(FREEGLUT_LIBRARY freeglut)

	find_path(OPENGL_INCLUDE_DIR gl/gl.h)
	find_library(OPENGL_LIBRARY opengl32)

	target_include_directories(FluidSim PRIVATE ${FREEGLUT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
	target_link_libraries(FluidSim ${FREEGLUT_LIBRARY} ${OPENGL_LIBRARY})
else()
	find_package(OpenGL REQUIRED)
	find_package(GLUT REQUIRED)

	target_link_libraries(FluidSim ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
endif()

FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()