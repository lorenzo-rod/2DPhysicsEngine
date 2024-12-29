#include <SFML/Graphics.hpp>
#include <iostream>
#include "math/vector2.h"
#include "math/point2.h"
#include "rigid_body/circle_body.h"
#include "rigid_body/rectangle_body.h"
#include <vector>
#include <random>
#include "physics_world/physics_world.h"

#define TO_DEGREES(X) ((X) * (180 / M_PI))

void drawSegment(const flatmath::Point2 &p1, const flatmath::Point2 &p2, sf::RenderWindow &window);
void drawRectangleFromPoints(const flatmath::Point2 &p1, const flatmath::Point2 &p2, sf::RenderWindow &window, float width);
float generateRandomFloat(float min, float max);

int main()
{
    int x_len = 1536;
    int y_len = 864;
    sf::RenderWindow window(sf::VideoMode(x_len, y_len), "SFML works!");
    window.setFramerateLimit(60);
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
        float mass = 1.f;
        float rotation = 0.f;
        flatmath::Vector2 position{generateRandomFloat(0, x_len), generateRandomFloat(0, y_len)};
        flatmath::Vector2 velocity{};
        flatmath::Vector2 force{};
        float radius = 1.f;
        CircleBody circle{mass, rotation, position, velocity, force, radius, 100};
        physics_world.addRigidBody(circle);
    }

    // for (int i = 0; i < 2; i++)
    // {
    //     float mass = 1.f;
    //     float rotation = 30.f;
    //     flatmath::Vector2 position{generateRandomFloat(0, x_len), generateRandomFloat(0, y_len)};
    //     flatmath::Vector2 velocity{};
    //     flatmath::Vector2 force{};
    //     float length = 1.f;
    //     float height = 1.f;
    //     RectangleBody rectangle{mass, rotation, position, velocity, force, length, height, 100};
    //     physics_world.addRigidBody(rectangle);
    // }

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            // else if (event.type == sf::Event::MouseButtonPressed)
            // {
            //     flatmath::Vector2 vec{event.mouseButton.x, event.mouseButton.y};
            //     std::cout << vec << std::endl;
            // }
            if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::W)
                {
                    physics_world.moveRigidBody(0, {0.f, -5.f});
                }
                if (event.key.code == sf::Keyboard::A)
                {
                    physics_world.moveRigidBody(0, {-5.f, 0.f});
                }
                if (event.key.code == sf::Keyboard::S)
                {
                    physics_world.moveRigidBody(0, {0.f, 5.f});
                }
                if (event.key.code == sf::Keyboard::D)
                {
                    physics_world.moveRigidBody(0, {5.f, 0.f});
                }
            }
        }

        flatmath::Point2 p1{343.0f, 128.0f};
        flatmath::Point2 p2{834.0f, 100.0f};
        window.clear();
        physics_world.resolveCollisions();
        for (const auto &rigid_body_ptr : physics_world)
        {
            rigid_body_ptr->draw(window);
        }
        window.display();
    }

    return 0;
}

void drawSegment(const flatmath::Point2 &p1, const flatmath::Point2 &p2, sf::RenderWindow &window)
{
    drawRectangleFromPoints(p1, p2, window, 1.f);
}

void drawRectangleFromPoints(const flatmath::Point2 &p1, const flatmath::Point2 &p2, sf::RenderWindow &window, float width)
{
    float length = flatmath::distance(p1, p2);
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
