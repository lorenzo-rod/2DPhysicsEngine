#include "half_color_circle.h"

HalfColorCircle::HalfColorCircle(float radius, std::size_t pointCount)
    : m_radius(radius), m_pointCount(pointCount), m_color1(sf::Color::White), m_color2(sf::Color::Cyan)
{
    updateVertices();
}

sf::Vector2f HalfColorCircle::getPoint(std::size_t index) const
{
    return (index < m_pointCount)
               ? sf::Vector2f{m_vertices[index].position.x, m_vertices[index].position.y}
               : sf::Vector2f{m_vertices[0].position.x, m_vertices[0].position.y};
}

std::size_t HalfColorCircle::getPointCount() const
{
    return m_pointCount;
}

void HalfColorCircle::updateVertices()
{
    m_vertices = sf::VertexArray(sf::TriangleFan, m_pointCount + 2);

    m_vertices[0].position = {0.f, 0.f};
    m_vertices[0].color = m_color1;

    for (std::size_t i = 0; i <= m_pointCount; ++i)
    {
        float angle = static_cast<float>(i) / m_pointCount * 2 * M_PI;
        float x = std::cos(angle) * m_radius;
        float y = std::sin(angle) * m_radius;

        m_vertices[i + 1].position = {x, y};
        m_vertices[i + 1].color = (angle <= M_PI) ? m_color1 : m_color2;
    }
}

void HalfColorCircle::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    states.transform *= getTransform();
    target.draw(m_vertices, states);
}
