/*
    Scan Tailor Universal - Interactive post-processing tool for scanned pages.
*/

#include "ImageCache.h"
#include <QMutexLocker>

ImageCache::ImageCache(int const capacity)
    : m_capacity(capacity)
{
}

QImage
ImageCache::get(ImageId const& id)
{
    QMutexLocker const locker(&m_mutex);

    CacheMap::iterator it = m_map.find(id);
    if (it == m_map.end()) {
        return QImage();
    }

    // Promote to most-recently-used.
    m_lru.splice(m_lru.end(), m_lru, it->second.second);
    return it->second.first;
}

void
ImageCache::put(ImageId const& id, QImage const& image)
{
    if (image.isNull() || m_capacity <= 0) {
        return;
    }

    QMutexLocker const locker(&m_mutex);

    CacheMap::iterator it = m_map.find(id);
    if (it != m_map.end()) {
        // Already present — update image and promote.
        it->second.first = image;
        m_lru.splice(m_lru.end(), m_lru, it->second.second);
        return;
    }

    // Evict LRU entry if at capacity.
    while (static_cast<int>(m_map.size()) >= m_capacity) {
        ImageId const& victim = m_lru.front();
        m_map.erase(victim);
        m_lru.pop_front();
    }

    // Insert new entry at MRU position.
    LruList::iterator lru_it = m_lru.insert(m_lru.end(), id);
    m_map[id] = std::make_pair(image, lru_it);
}

void
ImageCache::clear()
{
    QMutexLocker const locker(&m_mutex);
    m_map.clear();
    m_lru.clear();
}

ImageCache&
ImageCache::instance()
{
    static ImageCache cache(3);
    return cache;
}
