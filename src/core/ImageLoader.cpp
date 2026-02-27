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

#include "config.h"
#include "ImageLoader.h"
#include "TiffReader.h"
#ifdef ENABLE_MUPDF
#include "PdfReader.h"
#endif
#ifdef ENABLE_OPENJPEG
#include "Jp2Reader.h"
#endif
#include "ImageId.h"
#include <QImageReader>
#include <QImage>
#include <QString>
#include <QIODevice>
#include <QFile>
#include <QSize>

QImage
ImageLoader::load(ImageId const& image_id)
{
    return load(image_id.filePath(), image_id.zeroBasedPage());
}

QImage
ImageLoader::load(QString const& file_path, int const page_num)
{
    QFile file(file_path);
    if (!file.open(QIODevice::ReadOnly)) {
        return QImage();
    }

    if (file_path.startsWith(":")) {
        // internally empty pages are represented as multipage image although they're just links to the same single page image in app resources
        return load(file, 0);
    }

    return load(file, page_num);
}

QImage
ImageLoader::load(QIODevice& io_dev, int const page_num)
{
#ifdef ENABLE_MUPDF
    if (PdfReader::canRead(io_dev)) {
        return PdfReader::readImage(io_dev, page_num);
    }
#endif

    if (TiffReader::canRead(io_dev)) {
        return TiffReader::readImage(io_dev, page_num);
    }

    if (page_num != 0) {
        // Qt can only load the first page of multi-page images.
        return QImage();
    }

#ifdef ENABLE_OPENJPEG
    if (Jp2Reader::canRead(io_dev)) {
        return Jp2Reader::readImage(io_dev);
    }
#endif

    QImage image;
    QImageReader(&io_dev).read(&image);
    return image;
}

QImage
ImageLoader::loadScaled(ImageId const& image_id, QSize const& max_size)
{
    QString const& file_path = image_id.filePath();
    int const page_num = image_id.zeroBasedPage();

    QFile file(file_path);
    if (!file.open(QIODevice::ReadOnly)) {
        return QImage();
    }

    // For TIFF, PDF, and JP2 we must use the specialized readers
    // which don't support reduced-resolution loading, so fall back
    // to the full load path.
#ifdef ENABLE_MUPDF
    if (PdfReader::canRead(file)) {
        QImage image = PdfReader::readImage(file, page_num);
        if (!image.isNull() && (image.width() > max_size.width()
                || image.height() > max_size.height())) {
            image = image.scaled(max_size, Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
        }
        return image;
    }
#endif
    if (TiffReader::canRead(file)) {
        QImage image = TiffReader::readImage(file, page_num);
        if (!image.isNull() && (image.width() > max_size.width()
                || image.height() > max_size.height())) {
            image = image.scaled(max_size, Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
        }
        return image;
    }
#ifdef ENABLE_OPENJPEG
    if (Jp2Reader::canRead(file)) {
        QImage image = Jp2Reader::readImage(file);
        if (!image.isNull() && (image.width() > max_size.width()
                || image.height() > max_size.height())) {
            image = image.scaled(max_size, Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation);
        }
        return image;
    }
#endif

    if (page_num != 0) {
        return QImage();
    }

    // For formats handled by QImageReader (JPEG, PNG, BMP, etc.),
    // use setScaledSize() which enables libjpeg's built-in
    // DCT-domain downscaling (1/2, 1/4, 1/8) for JPEG images.
    // This avoids decoding the full-resolution image entirely.
    QImageReader reader(&file);
    QSize const full_size = reader.size();
    if (full_size.isValid() && (full_size.width() > max_size.width()
            || full_size.height() > max_size.height())) {
        QSize scaled = full_size;
        scaled.scale(max_size, Qt::KeepAspectRatio);
        reader.setScaledSize(scaled);
    }

    QImage image;
    reader.read(&image);
    return image;
}
