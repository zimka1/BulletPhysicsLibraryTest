cmake_minimum_required(VERSION 3.22)
project(projectile_2d_ballistics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(BP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../BulletPhysics")
file(GLOB_RECURSE BP_SOURCES CONFIGURE_DEPENDS "${BP_ROOT}/src/*.cpp")

add_library(bp_for_zadanie2 STATIC ${BP_SOURCES})
target_include_directories(bp_for_zadanie2 PUBLIC "${BP_ROOT}/src")
if(APPLE OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(bp_for_zadanie2 PRIVATE
        "-include" "${CMAKE_CURRENT_SOURCE_DIR}/bp_compile_fixup.h")
endif()

add_executable(projectile_2d projectile_2d.cpp)
target_link_libraries(projectile_2d PRIVATE bp_for_zadanie2)

if(APPLE)
    find_library(MACOS_OPENGL_FRAMEWORK OpenGL REQUIRED)
    find_library(MACOS_GLUT_FRAMEWORK GLUT REQUIRED)
    target_link_libraries(projectile_2d PRIVATE ${MACOS_OPENGL_FRAMEWORK} ${MACOS_GLUT_FRAMEWORK})
endif()
