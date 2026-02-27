/*
    Scan Tailor - Interactive post-processing tool for scanned pages.
    Copyright (C)  Joseph Artsimovich <joseph.artsimovigh@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef IMAGELOADER_H_
#define IMAGELOADER_H_

class ImageId;
class QImage;
class QString;
class QIODevice;
class QSize;

class ImageLoader
{
public:
    static QImage load(QString const& file_path, int page_num = 0);

    static QImage load(ImageId const& image_id);

    static QImage load(QIODevice& io_dev, int page_num);

    /**
     * Load an image at reduced resolution when the format supports it.
     * For JPEG, this uses libjpeg's built-in DCT-domain downscaling.
     * For other formats, loads the full image then downscales.
     * \p max_size The maximum thumbnail dimensions desired.
     */
    static QImage loadScaled(ImageId const& image_id, QSize const& max_size);
};

#endif
