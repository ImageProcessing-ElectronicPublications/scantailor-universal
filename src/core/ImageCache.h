/*
    Scan Tailor Universal - Interactive post-processing tool for scanned pages.

    A small thread-safe LRU cache for decoded QImages, keyed by ImageId.
    Avoids redundant disk I/O when the same source image is loaded
    multiple times across filter pipeline stages.
*/

#ifndef IMAGE_CACHE_H_
#define IMAGE_CACHE_H_

#include "ImageId.h"
#include <QImage>
#include <QMutex>
#include <list>
#include <map>

class ImageCache
{
public:
    /**
     * \param capacity Maximum number of images to cache.
     *        A value of 0 disables caching entirely.
     */
    explicit ImageCache(int capacity = 2);

    /**
     * Look up a cached image.  Returns a null QImage on miss.
     * On hit the entry is promoted to most-recently-used.
     */
    QImage get(ImageId const& id);

    /**
     * Insert an image into the cache.  If the cache is full,
     * the least-recently-used entry is evicted.
     * Null images are never cached.
     */
    void put(ImageId const& id, QImage const& image);

    /**
     * Remove all cached entries.
     */
    void clear();

    /**
     * Returns the process-wide singleton instance.
     */
    static ImageCache& instance();

private:
    typedef std::list<ImageId> LruList;
    typedef std::map<ImageId, std::pair<QImage, LruList::iterator>> CacheMap;

    mutable QMutex m_mutex;
    LruList m_lru;
    CacheMap m_map;
    int m_capacity;
};

#endif
