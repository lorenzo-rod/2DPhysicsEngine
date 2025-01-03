
add_library(RigidBody
    rigid_body.cpp
    circle_body.cpp
    static_circle_body.cpp
    rectangle_body.cpp
    static_rectangle_body.cpp)

target_link_libraries(RigidBody
    FlatMath)
