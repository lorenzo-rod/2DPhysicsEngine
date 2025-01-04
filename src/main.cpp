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

#define TO_DEGREES(X) ((X) * (180 / M_PI))

void drawSegment(const flatmath::Vector2 &p1, const flatmath::Vector2 &p2, sf::RenderWindow &window);
void drawRectangleFromPoints(const flatmath::Vector2 &p1, const flatmath::Vector2 &p2, sf::RenderWindow &window, float width);
float generateRandomFloat(float min, float max);

int main()
{
    int x_len = 1536;
    int y_len = 864;
    sf::RenderWindow window(sf::VideoMode(x_len, y_len), "SFML works!");
    sf::Clock clock;
    sf::Time delta = sf::seconds(0.01);
    // window.setFramerateLimit(60);
    // window.setFramerateLimit(60);
    // sf::RectangleShape shape{sf::Vector2f{100.f, 1.f}};
    // sf::RectangleShape shape2{sf::Vector2f{1.f, 100.f}};
    // sf::CircleShape shape3{100.f};
    // shape2.setOrigin(0, -y_len);
    // shape.setFillColor(sf::Color::Green);
    // sf::Transform t;
    // sf::Transform t2;
    // sf::Transform t3;
    // t.translate(300.f, 100.f);
    // t.rotate(-90);
    // t2.translate(0.f, -100.f);
    // t3.rotate(-90);
    // t3.rotate(90);
    // shape3.setOrigin(100.f, 100.f);
    PhysicsWorld physics_world;
    for (int i = 0; i < 5; i++)
    {
        float inv_mass = 1.f;
        float rotation = 0.f;
        float rotational_velocity = 0.f;
        float restitution{1.f};
        flatmath::Vector2 position{generateRandomFloat(0, x_len), generateRandomFloat(0, y_len)};
        flatmath::Vector2 velocity{};
        flatmath::Vector2 force{0.0f, 0.f};
        float radius = 0.5f * 100;
        CircleBody circle{inv_mass, rotation, rotational_velocity,
                          restitution, position, velocity,
                          force, radius};
        physics_world.addRigidBody(circle);
    }

    for (int i = 0; i < 5; i++)
    {
        float inv_mass = 1.f;
        float rotation = generateRandomFloat(0, 360);
        float rotational_velocity = 0.f;
        float restitution{1.f};
        flatmath::Vector2 position{generateRandomFloat(0, x_len), generateRandomFloat(0, y_len)};
        flatmath::Vector2 velocity{};
        flatmath::Vector2 force{0.0f, 0.f};
        float length = 1.f * 100;
        float height = 0.5f * 100;
        RectangleBody rectangle{inv_mass, rotation, rotational_velocity,
                                restitution, position, velocity,
                                force, length, height};
        physics_world.addRigidBody(rectangle);
    }
    float rotation = 0.f;
    float restitution{1.f};
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
    StaticRectangleBody static_rectangle3{rotation + 90,
                                          restitution, position3,
                                          length, height};
    physics_world.addRigidBody(static_rectangle3);
    StaticRectangleBody static_rectangle4{rotation + 90,
                                          restitution, position3 + flatmath::Vector2{x_len, 0.f},
                                          length, height};
    physics_world.addRigidBody(static_rectangle4);
    RigidBody *controllable_rb = physics_world.getRigidBody(0);
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            else if (event.type == sf::Event::MouseButtonPressed)
            {
                flatmath::Vector2 vec{event.mouseButton.x, event.mouseButton.y};
                std::cout << vec << std::endl;
            }
            if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::W)
                {
                    controllable_rb->addForce({0.0f, -100.f});
                    // physics_world.moveRigidBody(0, {0.f, -5.f});
                }
                if (event.key.code == sf::Keyboard::A)
                {
                    controllable_rb->addForce({-100.f, 0.0f});
                    // physics_world.moveRigidBody(0, {-5.f, 0.f});
                }
                if (event.key.code == sf::Keyboard::S)
                {
                    controllable_rb->addForce({0.0f, 100.f});
                    // physics_world.moveRigidBody(0, {0.f, 5.f});
                }
                if (event.key.code == sf::Keyboard::D)
                {
                    controllable_rb->addForce({100.f, 0.0f});
                    // physics_world.moveRigidBody(0, {5.f, 0.f});
                }
                if (event.key.code == sf::Keyboard::F)
                {
                    controllable_rb->setVelocity({0.f, 0.f});
                    controllable_rb->addForce({0.f, 0.f});
                    // physics_world.moveRigidBody(0, {5.f, 0.f});
                }
            }
        }
        std::cout << controllable_rb->getVelocity() << std::endl;
        window.clear();
        if (clock.getElapsedTime() > delta)
        {
            sf::Time dt = clock.getElapsedTime();
            // std::cout << dt.asSeconds() << std::endl;
            physics_world.step(delta.asSeconds());
            physics_world.resolveCollisions();
            clock.restart();
        }
        physics_world.draw(window);
        window.display();
    }

    return 0;
}

void drawSegment(const flatmath::Vector2 &p1, const flatmath::Vector2 &p2, sf::RenderWindow &window)
{
    drawRectangleFromPoints(p1, p2, window, 1.f);
}

void drawRectangleFromPoints(const flatmath::Vector2 &p1, const flatmath::Vector2 &p2, sf::RenderWindow &window, float width)
{
    float length = (p1 - p2).modulus();
    float angle = atan2((p2.y - p1.y), (p2.x - p1.x));
    sf::RectangleShape segment{sf::Vector2f{length, width}};
    sf::Transform segment_transform;

    segment.setOrigin(length / 2, width / 2);
    segment_transform.translate(p1.x + length / 2 * cos(angle), p1.y + length / 2 * sin(angle));
    segment_transform.rotate(TO_DEGREES(angle));
    window.draw(segment, segment_transform);
}

float generateRandomFloat(float min, float max)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float> dis(min, max);

    return dis(gen);
}
