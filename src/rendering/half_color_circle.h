#pragma once
#include <SFML/Graphics.hpp>
#include <cmath>

class HalfColorCircle : public sf::Shape
{
public:
    HalfColorCircle(float radius = 0, std::size_t pointCount = 100);

    sf::Vector2f getPoint(std::size_t index) const override;
    std::size_t getPointCount() const override;

private:
    float m_radius;
    std::size_t m_pointCount;
    sf::Color m_color1;
    sf::Color m_color2;
    sf::VertexArray m_vertices;

    void updateVertices();
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const override;
};
