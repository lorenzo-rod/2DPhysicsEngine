
add_library(RigidBody
    rigid_body.cpp
    circle_body.cpp
    rectangle_body.cpp)

target_link_libraries(RigidBody
    FlatMath)
