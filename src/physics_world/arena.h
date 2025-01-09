#pragma once
#include <memory>

class Arena
{
    std::unique_ptr<std::byte[]> m_pool;
    size_t m_size;
    size_t m_offset;

public:
    Arena(size_t size);

    void *allocate(size_t size);
    void release();
};
