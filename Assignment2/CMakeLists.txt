cmake_minimum_required(VERSION 3.5.1)
project(Assignment3 LANGUAGES CXX)

if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++latest /wd4141")
	set(CMAKE_CXX_FLAGS_RELEASE "/Ox /Ot /DNDEBUG")
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")
	set(CMAKE_CXX_FLAGS_DEBUG "-O0")
	set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

set(SOURCE_FILES 
	${CMAKE_CURRENT_SOURCE_DIR}/LinTriElement.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/FEM.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/FEModel.cpp
)

add_executable(FEM ${SOURCE_FILES})

target_include_directories(FEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/utils)

if(WIN32)
	find_path(FREEGLUT_INCLUDE_DIR gl/freeglut.h)
	find_library(FREEGLUT_LIBRARY freeglut)

	find_path(OPENGL_INCLUDE_DIR gl/gl.h)
	find_library(OPENGL_LIBRARY opengl32)

	target_include_directories(FEM PRIVATE ${FREEGLUT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
	target_link_libraries(FEM ${FREEGLUT_LIBRARY} ${OPENGL_LIBRARY})
else()
	find_package(OpenGL REQUIRED)
	find_package(GLUT REQUIRED)

	target_link_libraries(FEM ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
endif()
