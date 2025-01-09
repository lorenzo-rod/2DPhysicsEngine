cmake_minimum_required(VERSION 3.16.0)
project(2DPhysicsEngine)

add_executable(Main 
    src/main.cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.5 COMPONENTS system window graphics REQUIRED)
find_package(OpenGL REQUIRED)

add_subdirectory(src/math)
add_subdirectory(src/rigid_body)
add_subdirectory(src/physics_world)
add_subdirectory(src/rendering)

target_link_libraries(Main 
    PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
        OpenGL::GL
        pthread
        dl
        X11
        freetype
        jpeg
        udev
        FlatMath
        RigidBody
        PhysicsWorld
        Rendering
)
