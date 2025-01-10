#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <iostream>
#include "math/vector2.h"
#include "math/vector2.h"
#include "rigid_body/circle_body.h"
#include "rigid_body/rectangle_body.h"
#include "rigid_body/static_circle_body.h"
#include "rigid_body/static_rectangle_body.h"
#include <vector>
#include <random>
#include "physics_world/physics_world.h"
#include <cmath>
#include "rendering/half_color_circle.h"

inline float to_radians(float x) { return x * M_PI / 180.f; };

float generateRandomFloat(float min, float max);

int main()
{
    int x_len = 1536;
    int y_len = 864;
    sf::RenderWindow window(sf::VideoMode(x_len, y_len), "Physics Engine Demo!");
    sf::Clock clock;
    sf::Clock step_time_clock;
    PhysicsWorld physics_world;

    sf::Font font;
    if (!font.loadFromFile("../../../src/Roboto-Regular.ttf"))
    {
        std::cerr << "Error loading font\n";
        return -1;
    }

    sf::Text text;
    text.setFont(font);
    text.setCharacterSize(24);
    text.setFillColor(sf::Color::White);
    text.setPosition(30.f, 30.f);

    float rotation = 0.f;
    float restitution{0.35f};
    flatmath::Vector2 position1{0, 0};
    flatmath::Vector2 position2{x_len, 0};
    flatmath::Vector2 position3{0, y_len};
    flatmath::Vector2 position4{x_len, y_len};
    flatmath::Vector2 force{0.0f, 0.f};
    float length = 2 * x_len;
    float height = 0.5f * 100;
    StaticRectangleBody static_rectangle{rotation,
                                         restitution, position1,
                                         length, height};
    physics_world.addRigidBody(static_rectangle);

    length = 2 * x_len;
    height = 0.5f * 100;
    rotation = 0.f;
    StaticRectangleBody static_rectangle2{rotation,
                                          restitution, position3,
                                          length, height};
    physics_world.addRigidBody(static_rectangle2);

    StaticRectangleBody static_rectangle3{rotation + to_radians(90),
                                          restitution, position3,
                                          length, height};
    physics_world.addRigidBody(static_rectangle3);

    StaticRectangleBody static_rectangle4{rotation + to_radians(90),
                                          restitution, position3 + flatmath::Vector2{x_len, 0.f},
                                          length, height};
    physics_world.addRigidBody(static_rectangle4);

    StaticRectangleBody static_rectangle5{rotation + to_radians(15),
                                          restitution, flatmath::Vector2{500, 250.f},
                                          600, height - 10};
    physics_world.addRigidBody(static_rectangle5);

    StaticRectangleBody static_rectangle6{rotation + to_radians(-20),
                                          restitution, flatmath::Vector2{1000, 450.f},
                                          600, height - 10};
    physics_world.addRigidBody(static_rectangle6);

    float inv_mass = 1.f;
    float rotational_velocity = 0.f;
    flatmath::Vector2 velocity{};
    length = 1.f * 100;
    height = 0.5f * 100;
    flatmath::Vector2 position;
    float x_pos;
    float y_pos;
    float radius = 0.5f * 100;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::MouseButtonPressed)
            {
                x_pos = event.mouseButton.x;
                y_pos = event.mouseButton.y;
                position = flatmath::Vector2{x_pos, y_pos};
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    RectangleBody rectangle{inv_mass, rotation, rotational_velocity,
                                            restitution, position, velocity,
                                            force, 0.f, length, height};
                    physics_world.addRigidBody(rectangle);
                }
                else
                {
                    CircleBody circle{inv_mass, rotation, rotational_velocity,
                                      restitution, position, velocity,
                                      force, 0.f, radius};
                    physics_world.addRigidBody(circle);
                }
            }
        }
        window.clear();
        step_time_clock.restart();
        physics_world.step(clock.restart().asSeconds());
        text.setString("Step Time: " + std::to_string(step_time_clock.restart().asMicroseconds()) + " us" +
                       "\n" + "Number of bodies: " + std::to_string(physics_world.getNumberOfBodies()));
        physics_world.draw(window);
        window.draw(text);
        window.display();
    }

    return 0;
}

float generateRandomFloat(float min, float max)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float> dis(min, max);

    return dis(gen);
}
