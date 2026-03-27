# Demo only: builds BulletPhysics sources as a static library without modifying
# the BulletPhysics/ tree or running its tests (no GTest required here).

cmake_minimum_required(VERSION 3.22)
project(projectile_glut_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(BP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../BulletPhysics")
file(GLOB_RECURSE BP_SOURCES CONFIGURE_DEPENDS "${BP_ROOT}/src/*.cpp")

add_library(bp_for_demo STATIC ${BP_SOURCES})
target_include_directories(bp_for_demo PUBLIC "${BP_ROOT}/src")
# Apple libc++: compile library sources without editing BulletPhysics/*.cpp
if(APPLE OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(bp_for_demo PRIVATE
        "-include" "${CMAKE_CURRENT_SOURCE_DIR}/bp_compile_fixup.h")
endif()

add_executable(projectile_glut_demo projectile_glut_bullet.cpp)
target_link_libraries(projectile_glut_demo PRIVATE bp_for_demo)

if(APPLE)
    find_library(MACOS_OPENGL_FRAMEWORK OpenGL REQUIRED)
    find_library(MACOS_GLUT_FRAMEWORK GLUT REQUIRED)
    target_link_libraries(projectile_glut_demo PRIVATE ${MACOS_OPENGL_FRAMEWORK} ${MACOS_GLUT_FRAMEWORK})
endif()
