#include "arena.h"

Arena::Arena(size_t size) : m_pool(std::make_unique<std::byte[]>(size)),
                            m_size(size), m_offset(0)
{
}

void *Arena::allocate(size_t size)
{
    if (m_offset + size < (m_size - 1))
    {
        throw std::bad_alloc();
    }
    else
    {
        void *ptr = m_pool.get() + m_offset;
        m_offset += size;
        return static_cast<void *>(ptr);
    }
}

void Arena::release()
{
    m_offset = 0;
}