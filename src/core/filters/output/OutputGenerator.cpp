/*
    Scan Tailor - Interactive post-processing tool for scanned pages.
    Copyright (C)  Joseph Artsimovich <joseph.artsimovich@gmail.com>

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

#include "CommandLine.h"
#include "OutputGenerator.h"
#include "ImageTransformation.h"
#include "FilterData.h"
#include "TaskStatus.h"
#include "Utils.h"
#include "DebugImages.h"
#include "EstimateBackground.h"
#include "Despeckle.h"
#include "RenderParams.h"
#include "dewarping/DistortionModel.h"
#include "Dpi.h"
#include "Dpm.h"
#include "Zone.h"
#include "ZoneSet.h"
#include "PictureLayerProperty.h"
#include "FillColorProperty.h"
#include "dewarping/CylindricalSurfaceDewarper.h"
#include "dewarping/TextLineTracer.h"
#include "dewarping/TopBottomEdgeTracer.h"
#include "dewarping/DistortionModelBuilder.h"
#include "dewarping/DewarpingPointMapper.h"
#include "dewarping/RasterDewarper.h"
#include "imageproc/GrayImage.h"
#include "imageproc/BinaryImage.h"
#include "imageproc/BinaryThreshold.h"
#include "imageproc/Binarize.h"
#include "imageproc/BWColor.h"
#include "imageproc/Transform.h"
#include "imageproc/Scale.h"
#include "imageproc/Morphology.h"
#include "imageproc/Connectivity.h"
#include "imageproc/ConnCompEraser.h"
#include "imageproc/ConnCompEraserExt.h"
#include "imageproc/SeedFill.h"
#include "imageproc/Constants.h"
#include "imageproc/Grayscale.h"
#include "imageproc/GaussBlur.h"
#include "imageproc/RasterOp.h"
#include "imageproc/GrayRasterOp.h"
#include "imageproc/PolynomialSurface.h"
#include "imageproc/SavGolFilter.h"
#include "imageproc/DrawOver.h"
#include "imageproc/AdjustBrightness.h"
#include "imageproc/PolygonRasterizer.h"
#include "imageproc/ConnectivityMap.h"
#include "imageproc/InfluenceMap.h"
#include "config.h"
#include "settings/globalstaticsettings.h"
#ifdef HAVE_EXIV2
#include "ImageMetadataCopier.h"
#endif
#ifndef Q_MOC_RUN
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#endif
#include <QImage>
#include <QSize>
#include <QPoint>
#include <QRect>
#include <QRectF>
#include <QPointF>
#include <QPolygonF>
#include <QPainter>
#include <QPainterPath>
#include <QColor>
#include <QPen>
#include <QBrush>
#include <QtGlobal>
#include <QDebug>
#include <Qt>
#include <vector>
#include <memory>
#include <new>
#include <algorithm>
#include <assert.h>
#include <string.h>
#include <stdint.h>
//begin of modified by monday2000
//Marginal_Dewarping
#include "imageproc/OrthogonalRotation.h"
//end of modified by monday2000

using namespace imageproc;
using namespace dewarping;

namespace output
{

namespace
{

struct RaiseAboveBackground {
    static uint8_t transform(uint8_t src, uint8_t dst)
    {
        // src: orig
        // dst: background (dst >= src)
        if (dst - src < 1) {
            return 0xff;
        }
        unsigned const orig = src;
        unsigned const background = dst;
        return static_cast<uint8_t>((orig * 255 + background / 2) / background);
    }
};

struct CombineInverted {
    static uint8_t transform(uint8_t src, uint8_t dst)
    {
        unsigned const dilated = dst;
        unsigned const eroded = src;
        unsigned const res = 255 - (255 - dilated) * eroded / 255;
        return static_cast<uint8_t>(res);
    }
};

/**
 * In picture areas we make sure we don't use pure black and pure white colors.
 * These are reserved for text areas.  This behaviour makes it possible to
 * detect those picture areas later and treat them differently, for example
 * encoding them as a background layer in DjVu format.
 */
template<typename PixelType>
PixelType reserveBlackAndWhite(PixelType color);

template<>
uint32_t reserveBlackAndWhite(uint32_t color)
{
    // We handle both RGB32 and ARGB32 here.
    switch (color & 0x00FFFFFF) {
    case 0x00000000:
        return 0xFF010101;
    case 0x00FFFFFF:
        return 0xFFFEFEFE;
    default:
        return color;
    }
}

template<>
uint8_t reserveBlackAndWhite(uint8_t color)
{
    switch (color) {
    case 0x00:
        return 0x01;
    case 0xFF:
        return 0xFE;
    default:
        return color;
    }
}

template<typename PixelType>
void reserveBlackAndWhite(QSize size, int stride, PixelType* data)
{
    int const width = size.width();
    int const height = size.height();

    PixelType* line = data;
    for (int y = 0; y < height; ++y, line += stride) {
        for (int x = 0; x < width; ++x) {
            line[x] = reserveBlackAndWhite<PixelType>(line[x]);
        }
    }
}

void reserveBlackAndWhite(QImage& img)
{
    assert(img.depth() == 8 || img.depth() == 24 || img.depth() == 32);
    switch (img.format()) {
    case QImage::Format_Indexed8:
        reserveBlackAndWhite(img.size(), img.bytesPerLine(), img.bits());
        break;
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32:
        reserveBlackAndWhite(img.size(), img.bytesPerLine() / 4, (uint32_t*)img.bits());
        break;
    default:; // Should not happen.
    }
}

/**
 * Fills areas of \p mixed with pixels from \p bw_content in
 * areas where \p bw_mask is black.  Supported \p mixed image formats
 * are Indexed8 grayscale, RGB32 and ARGB32.
 * The \p MixedPixel type is uint8_t for Indexed8 grayscale and uint32_t
 * for RGB32 and ARGB32.
 */
template<typename MixedPixel>
void combineMixed(
    QImage& mixed, BinaryImage const& bw_content,
    BinaryImage const& bw_mask)
{
    MixedPixel* mixed_line = reinterpret_cast<MixedPixel*>(mixed.bits());
    int const mixed_stride = mixed.bytesPerLine() / sizeof(MixedPixel);
    uint32_t const* bw_content_line = bw_content.data();
    int const bw_content_stride = bw_content.wordsPerLine();
    uint32_t const* bw_mask_line = bw_mask.data();
    int const bw_mask_stride = bw_mask.wordsPerLine();
    int const width = mixed.width();
    int const height = mixed.height();
    uint32_t const msb = uint32_t(1) << 31;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (bw_mask_line[x >> 5] & (msb >> (x & 31))) {
                // B/W content.

                uint32_t tmp = bw_content_line[x >> 5];
                tmp >>= (31 - (x & 31));
                tmp &= uint32_t(1);
                // Now it's 0 for white and 1 for black.

                --tmp; // 0 becomes 0xffffffff and 1 becomes 0.

                tmp |= 0xff000000; // Force opacity.

                mixed_line[x] = static_cast<MixedPixel>(tmp);
            } else {
                // Non-B/W content.
                mixed_line[x] = reserveBlackAndWhite<MixedPixel>(mixed_line[x]);
            }
        }
        mixed_line += mixed_stride;
        bw_content_line += bw_content_stride;
        bw_mask_line += bw_mask_stride;
    }
}

}

// --------------- contour tracing helpers ---------------

inline bool pixBlack(BinaryImage const& img, int x, int y)
{
    if (x < 0 || x >= img.width() || y < 0 || y >= img.height()) {
        return false;
    }
    uint32_t const* line = img.data() + img.wordsPerLine() * y;
    uint32_t const msb = uint32_t(1) << 31;
    return (line[x >> 5] & (msb >> (x & 31))) != 0;
}

/**
 * Moore boundary tracing on a single-component binary image.
 * Traces the outer boundary of BLACK pixels, returning an ordered
 * polygon of pixel coordinates.
 */
QPolygonF traceMooreBoundary(BinaryImage const& ccImg)
{
    int const w = ccImg.width();
    int const h = ccImg.height();

    // 8-connected clockwise: E, SE, S, SW, W, NW, N, NE
    static int const dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
    static int const dy[] = {0, 1, 1, 1, 0, -1, -1, -1};

    // Find start: topmost row, leftmost black pixel.
    int sx = -1, sy = -1;
    for (int y = 0; y < h && sx < 0; ++y) {
        for (int x = 0; x < w; ++x) {
            if (pixBlack(ccImg, x, y)) {
                sx = x;
                sy = y;
                break;
            }
        }
    }
    if (sx < 0) {
        return QPolygonF();
    }

    // Check for isolated pixel — return a unit square.
    bool hasNeighbor = false;
    for (int d = 0; d < 8; ++d) {
        if (pixBlack(ccImg, sx + dx[d], sy + dy[d])) {
            hasNeighbor = true;
            break;
        }
    }
    if (!hasNeighbor) {
        QPolygonF p;
        p << QPointF(sx, sy) << QPointF(sx + 1, sy)
          << QPointF(sx + 1, sy + 1) << QPointF(sx, sy + 1);
        return p;
    }

    QPolygonF boundary;
    int cx = sx, cy = sy;
    // We found start by scanning L→R, so the backtrack pixel is to the west.
    int backDir = 4; // West

    int const maxIter = w * h + 1;
    for (int iter = 0; iter < maxIter; ++iter) {
        boundary.append(QPointF(cx, cy));

        bool found = false;
        for (int i = 1; i <= 8; ++i) {
            int const dir = (backDir + i) % 8;
            int const nx = cx + dx[dir];
            int const ny = cy + dy[dir];
            if (pixBlack(ccImg, nx, ny)) {
                backDir = (dir + 4) % 8;
                cx = nx;
                cy = ny;
                found = true;
                break;
            }
        }
        if (!found) {
            break;
        }
        if (cx == sx && cy == sy) {
            break; // returned to start
        }
    }

    return boundary;
}

/**
 * Squared perpendicular distance from point p to line segment a–b.
 */
double ptLineDistSq(QPointF const& p, QPointF const& a, QPointF const& b)
{
    double const abx = b.x() - a.x();
    double const aby = b.y() - a.y();
    double const len2 = abx * abx + aby * aby;
    if (len2 < 1e-12) {
        double const dx = p.x() - a.x();
        double const dy = p.y() - a.y();
        return dx * dx + dy * dy;
    }
    double t = ((p.x() - a.x()) * abx + (p.y() - a.y()) * aby) / len2;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;
    double const dx = p.x() - (a.x() + t * abx);
    double const dy = p.y() - (a.y() + t * aby);
    return dx * dx + dy * dy;
}

void dpRecurse(
    QPolygonF const& poly, int first, int last,
    double epsSq, std::vector<bool>& keep)
{
    if (last - first <= 1) {
        return;
    }
    double maxDistSq = 0;
    int maxIdx = first;
    for (int i = first + 1; i < last; ++i) {
        double const d = ptLineDistSq(poly[i], poly[first], poly[last]);
        if (d > maxDistSq) {
            maxDistSq = d;
            maxIdx = i;
        }
    }
    if (maxDistSq > epsSq) {
        keep[maxIdx] = true;
        dpRecurse(poly, first, maxIdx, epsSq, keep);
        dpRecurse(poly, maxIdx, last, epsSq, keep);
    }
}

/**
 * Douglas-Peucker polygon simplification.
 * For a closed polygon, we split at two anchor points (0 and n/2)
 * and simplify each arc independently.
 */
QPolygonF simplifyDP(QPolygonF const& poly, double epsilon)
{
    int const n = poly.size();
    if (n <= 4) {
        return poly;
    }

    double const epsSq = epsilon * epsilon;
    int const mid = n / 2;

    std::vector<bool> keep(n, false);
    keep[0] = true;
    keep[mid] = true;
    keep[n - 1] = true;

    dpRecurse(poly, 0, mid, epsSq, keep);
    dpRecurse(poly, mid, n - 1, epsSq, keep);

    QPolygonF result;
    for (int i = 0; i < n; ++i) {
        if (keep[i]) {
            result.append(poly[i]);
        }
    }
    return result;
}

/**
 * Feathered version of combineMixed.  Uses a grayscale alpha mask
 * (0 = full color/gray, 255 = full binarized) to blend between the
 * original image and the binarized content at picture zone boundaries.
 * This eliminates the hard seam visible in the binary-mask version.
 */
template<typename MixedPixel>
void combineMixedFeathered(
    QImage& mixed, BinaryImage const& bw_content,
    GrayImage const& alpha_mask)
{
    MixedPixel* mixed_line = reinterpret_cast<MixedPixel*>(mixed.bits());
    int const mixed_stride = mixed.bytesPerLine() / sizeof(MixedPixel);
    uint32_t const* bw_content_line = bw_content.data();
    int const bw_content_stride = bw_content.wordsPerLine();
    uint8_t const* alpha_line = alpha_mask.data();
    int const alpha_stride = alpha_mask.stride();
    int const width = mixed.width();
    int const height = mixed.height();
    uint32_t const msb = uint32_t(1) << 31;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int const alpha = alpha_line[x]; // 0=color, 255=binarized

            if (alpha == 0) {
                // Fully in picture zone — keep original, but reserve
                // pure black/white.
                mixed_line[x] = reserveBlackAndWhite<MixedPixel>(mixed_line[x]);
            } else if (alpha == 255) {
                // Fully in text zone — use binarized content.
                uint32_t tmp = bw_content_line[x >> 5];
                tmp >>= (31 - (x & 31));
                tmp &= uint32_t(1);
                --tmp;
                tmp |= 0xff000000;
                mixed_line[x] = static_cast<MixedPixel>(tmp);
            } else {
                // Feathered transition zone — blend.
                // Get binarized value (0x00 for black, 0xFF for white).
                uint32_t bw_bit = bw_content_line[x >> 5];
                bw_bit >>= (31 - (x & 31));
                bw_bit &= uint32_t(1);
                uint8_t const bw_val = bw_bit ? 0x00 : 0xFF;

                MixedPixel const orig = reserveBlackAndWhite<MixedPixel>(mixed_line[x]);

                if (sizeof(MixedPixel) == 1) {
                    // Grayscale: simple alpha blend.
                    int const o = static_cast<uint8_t>(orig);
                    int const blended = (o * (255 - alpha) + bw_val * alpha + 127) / 255;
                    mixed_line[x] = static_cast<MixedPixel>(blended);
                } else {
                    // RGB32/ARGB32: blend each channel.
                    uint32_t const o = static_cast<uint32_t>(orig);
                    int const or_ = (o >> 16) & 0xFF;
                    int const og  = (o >> 8)  & 0xFF;
                    int const ob  =  o        & 0xFF;
                    int const inv_a = 255 - alpha;
                    int const br = (or_ * inv_a + bw_val * alpha + 127) / 255;
                    int const bg = (og  * inv_a + bw_val * alpha + 127) / 255;
                    int const bb = (ob  * inv_a + bw_val * alpha + 127) / 255;
                    mixed_line[x] = static_cast<MixedPixel>(
                        0xFF000000u | (br << 16) | (bg << 8) | bb
                    );
                }
            }
        }
        mixed_line += mixed_stride;
        bw_content_line += bw_content_stride;
        alpha_line += alpha_stride;
    }
}

} // anonymous namespace

OutputGenerator::OutputGenerator(
    Dpi const& dpi, ColorParams const& color_params,
    DespeckleLevel const despeckle_level,
    ImageTransformation const& xform,
    QPolygonF const& content_rect_phys)
    :   m_dpi(dpi),
        m_colorParams(color_params),
        m_xform(xform),
        m_outRect(xform.resultingRect().toAlignedRect()),
        m_contentRect(xform.transform().map(content_rect_phys).boundingRect().toAlignedRect()),
        m_despeckleLevel(despeckle_level)
{
    /*
    std::cout << "m_outRect.left(): " << m_outRect.left() << " right(): " << m_outRect.right() << " top: " << m_outRect.top() << " bottom: " << m_outRect.bottom() << std::endl;
    std::cout << "m_contentRect.left(): " << m_contentRect.left() << " right(): " << m_contentRect.right() << " top: " << m_contentRect.top() << " bottom: " << m_contentRect.bottom() << std::endl;
    */

    // Sometimes `toAlignedRect()` may return rect with coordinate < 0.0
    if (m_outRect.left() < 0) {
        m_outRect.setLeft(0);
    }
    if (m_outRect.top() < 0) {
        m_outRect.setTop(0);
    }
    if (m_contentRect.left() < m_outRect.left()) {
        m_contentRect.setLeft(m_outRect.left());
    }
    if (m_contentRect.top() < m_outRect.top()) {
        m_contentRect.setTop(m_outRect.top());
    }
    if (m_contentRect.right() > m_outRect.right()) {
        m_contentRect.setRight(m_outRect.right());
    }
    if (m_contentRect.bottom() > m_outRect.bottom()) {
        m_contentRect.setBottom(m_outRect.bottom());
    }

    assert(m_outRect.topLeft() == QPoint(0, 0));

    // Note that QRect::contains(<empty rect>) always returns false, so we don't use it here.
    assert(m_outRect.contains(m_contentRect.topLeft()) && m_outRect.contains(m_contentRect.bottomRight()));
}

QImage
OutputGenerator::process(
    TaskStatus const& status, FilterData const& input,
//Quadro_Zoner
    //ZoneSet const& picture_zones, ZoneSet const& fill_zones,
    ZoneSet& picture_zones, ZoneSet const& fill_zones,
    DewarpingMode dewarping_mode,
    DistortionModel& distortion_model,
    DepthPerception const& depth_perception,
//Original_Foreground_Mixed
    bool keep_orig_fore_subscan,
    imageproc::BinaryImage* auto_layer_mask,
    imageproc::BinaryImage* speckles_image,
    DebugImages* const dbg,
    PageId* p_pageId,
    IntrusivePtr<Settings>* p_settings
) const
{
    QImage image(
        processImpl(
            status, input, picture_zones, fill_zones,
            dewarping_mode, distortion_model, depth_perception,
            keep_orig_fore_subscan,
            auto_layer_mask, speckles_image, dbg,
            p_pageId, p_settings
        )
    );
    assert(!image.isNull());

    // Set the correct DPI.
    Dpm const output_dpm(m_dpi);
    image.setDotsPerMeterX(output_dpm.horizontal());
    image.setDotsPerMeterY(output_dpm.vertical());

    return image;
}

QSize
OutputGenerator::outputImageSize() const
{
    return m_outRect.size();
}

QRect
OutputGenerator::outputContentRect() const
{
    return m_contentRect;
}

GrayImage
OutputGenerator::normalizeIlluminationGray(
    TaskStatus const& status,
    QImage const& input, QPolygonF const& area_to_consider,
    QTransform const& xform, QRect const& target_rect,
    GrayImage* background, DebugImages* const dbg)
{
    GrayImage to_be_normalized(
        transformToGray(
            input, xform, target_rect, OutsidePixels::assumeWeakNearest()
        )
    );
    if (dbg) {
        dbg->add(to_be_normalized, "to_be_normalized");
    }

    status.throwIfCancelled();

    QPolygonF transformed_consideration_area(xform.map(area_to_consider));
    transformed_consideration_area.translate(-target_rect.topLeft());

    PolynomialSurface const bg_ps(
        estimateBackground(
            to_be_normalized, transformed_consideration_area,
            status, dbg
        )
    );

    status.throwIfCancelled();

    GrayImage bg_img(bg_ps.render(to_be_normalized.size()));
    if (dbg) {
        dbg->add(bg_img, "background");
    }
    if (background) {
        *background = bg_img;
    }

    status.throwIfCancelled();

    grayRasterOp<RaiseAboveBackground>(bg_img, to_be_normalized);
    if (dbg) {
        dbg->add(bg_img, "normalized_illumination");
    }

    return bg_img;
}

imageproc::BinaryImage
OutputGenerator::estimateBinarizationMask(
    TaskStatus const& status, GrayImage const& gray_source,
    QRect const& source_rect, QRect const& source_sub_rect,
    DebugImages* const dbg) const
{
    assert(source_rect.contains(source_sub_rect));

    // If we need to strip some of the margins from a grayscale
    // image, we may actually do it without copying anything.
    // We are going to construct a QImage from existing data.
    // That image won't own that data, but gray_source is not
    // going anywhere, so it's fine.

    GrayImage trimmed_image;

    if (source_rect == source_sub_rect) {
        trimmed_image = gray_source; // Shallow copy.
    } else {
        // Sub-rectangle in input image coordinates.
        QRect relative_subrect(source_sub_rect);
        relative_subrect.moveTopLeft(
            source_sub_rect.topLeft() - source_rect.topLeft()
        );

        int const stride = gray_source.stride();
        int const offset = relative_subrect.top() * stride
                           + relative_subrect.left();

        trimmed_image = GrayImage(QImage(
                                      gray_source.data() + offset,
                                      relative_subrect.width(), relative_subrect.height(),
                                      stride, QImage::Format_Indexed8
                                  ));
    }

    status.throwIfCancelled();

    QSize const downscaled_size(to300dpi(trimmed_image.size(), m_dpi));

    // A 300dpi version of trimmed_image.
    GrayImage downscaled_input(
        scaleToGray(trimmed_image, downscaled_size)
    );
    trimmed_image = GrayImage(); // Save memory.

    status.throwIfCancelled();

    // Light areas indicate pictures.
    GrayImage picture_areas(detectPictures(downscaled_input, status, dbg));

    status.throwIfCancelled();

    // Text evidence suppression.
    //
    // The gradient-based detector can falsely classify large display
    // fonts (chapter titles, headings, drop caps) as pictures because
    // their gradient features are large enough to survive the morphological
    // opening.  ContentBoxFinder::estimateTextMask() solves this using
    // fill-factor and UEP analysis, but that result is discarded before
    // reaching the output stage.
    //
    // We perform a lightweight version here: binarize the input,
    // horizontal closing to connect characters into text-line blobs,
    // then check each connected component for text-line properties
    // (fill factor 15-70%, width > 3x height).  Qualifying regions
    // have their picture likelihood suppressed to zero.
    {
        BinaryImage bw_content(downscaled_input, BinaryThreshold::otsuThreshold(downscaled_input));

        // Horizontal closing connects characters within a text line.
        // 30px at 300 DPI ≈ 2.5mm, bridges inter-character gaps but
        // does not bridge inter-column gaps.
        BinaryImage closed(closeBrick(bw_content, QSize(30, 1)));

        int const pa_w = picture_areas.width();
        int const pa_h = picture_areas.height();

        ConnCompEraserExt eraser(closed, CONN4);
        for (;;) {
            ConnComp const cc(eraser.nextConnComp());
            if (cc.isNull()) {
                break;
            }

            QRect const& r = cc.rect();

            // Text lines are significantly wider than tall.
            if (r.width() < r.height() * 3) {
                continue;
            }

            // Skip tiny components (noise).
            if (r.width() < 20 || r.height() < 4) {
                continue;
            }

            // Compute fill factor of original content within this CC's rect.
            int black_pixels = 0;
            int total_pixels = r.width() * r.height();
            uint32_t const* bw_line = bw_content.data() + bw_content.wordsPerLine() * r.top();
            int const bw_wpl = bw_content.wordsPerLine();
            uint32_t const msb = uint32_t(1) << 31;
            for (int y = r.top(); y <= r.bottom(); ++y, bw_line += bw_wpl) {
                for (int x = r.left(); x <= r.right(); ++x) {
                    if (bw_line[x >> 5] & (msb >> (x & 31))) {
                        ++black_pixels;
                    }
                }
            }

            double const fill = (double)black_pixels / total_pixels;

            // Text lines typically have fill factor 15-70%.
            // Below 15% is mostly whitespace (not a text line).
            // Above 70% is a solid block (rule, border, filled shape).
            if (fill < 0.15 || fill > 0.70) {
                continue;
            }

            // This CC looks like a text line — suppress picture
            // likelihood in this region.
            uint8_t* pa_line = picture_areas.data() + picture_areas.stride() * r.top();
            int const pa_stride = picture_areas.stride();
            int const clamp_bottom = std::min(r.bottom(), pa_h - 1);
            int const clamp_right = std::min(r.right(), pa_w - 1);
            for (int y = r.top(); y <= clamp_bottom; ++y, pa_line += pa_stride) {
                for (int x = r.left(); x <= clamp_right; ++x) {
                    pa_line[x] = 0;
                }
            }
        }
    }

    // Halftone detection.
    //
    // Halftone photographs in older printed material (pre-1990s books,
    // newspapers, magazines) reproduce continuous-tone images using
    // regular patterns of small dots.  At 300 DPI, typical halftone
    // screens (85-150 lpi) produce dots spaced 2-4 pixels apart with
    // dot diameters of 1-2 pixels.
    //
    // The gradient-based detector often fails on halftones because the
    // gradient is distributed at dot-scale intervals, making the
    // response look text-like (many small sharp transitions rather
    // than the broad transitions of a continuous-tone photograph).
    //
    // Key insight: halftone dots are much smaller than text character
    // strokes.  A 3x3 morphological opening destroys most halftone
    // dots (they don't survive the erosion) but preserves text strokes
    // (which are wider and connected).  The fraction of black pixels
    // that disappear after opening — the "vanishing ratio" — is very
    // high for halftone (>60%) and low for text (<30%).
    //
    // Algorithm: binarize, open with 3x3, divide into tiles, compute
    // vanishing ratio per tile, boost picture likelihood in tiles where
    // the ratio indicates halftone.
    {
        BinaryImage bw_input(downscaled_input,
            BinaryThreshold::otsuThreshold(downscaled_input));
        BinaryImage opened(openBrick(bw_input, QSize(3, 3), WHITE));

        int const w = downscaled_input.width();
        int const h = downscaled_input.height();
        int const tile_size = 32;
        int const bw_wpl = bw_input.wordsPerLine();
        int const op_wpl = opened.wordsPerLine();
        uint32_t const* bw_data = bw_input.data();
        uint32_t const* op_data = opened.data();
        uint32_t const msb = uint32_t(1) << 31;

        uint8_t* pa_data = picture_areas.data();
        int const pa_stride = picture_areas.stride();

        for (int ty = 0; ty < h; ty += tile_size) {
            int const y_end = std::min(ty + tile_size, h);
            for (int tx = 0; tx < w; tx += tile_size) {
                int const x_end = std::min(tx + tile_size, w);
                int const tile_area = (y_end - ty) * (x_end - tx);

                // Count black pixels in original and in opened version.
                int orig_black = 0;
                int survived_black = 0;
                for (int y = ty; y < y_end; ++y) {
                    uint32_t const* bw_line = bw_data + bw_wpl * y;
                    uint32_t const* op_line = op_data + op_wpl * y;
                    for (int x = tx; x < x_end; ++x) {
                        uint32_t const bit = msb >> (x & 31);
                        int const word = x >> 5;
                        if (bw_line[word] & bit) {
                            ++orig_black;
                        }
                        if (op_line[word] & bit) {
                            ++survived_black;
                        }
                    }
                }

                // Skip tiles with negligible content.
                if (orig_black < tile_area / 20) {  // < 5% density
                    continue;
                }

                double const vanishing_ratio =
                    1.0 - (double)survived_black / orig_black;

                // Halftone: most dots vanish after 3x3 opening.
                // Text: most strokes survive.
                // Threshold at 0.55 — halftone typically > 0.65,
                // text typically < 0.30.  The gap is wide.
                if (vanishing_ratio < 0.55) {
                    continue;
                }

                // Also require minimum dot density (vanished pixels
                // per tile area) to reject sparse noise.
                int const vanished = orig_black - survived_black;
                if (vanished < tile_area / 30) {  // < ~3.3%
                    continue;
                }

                // This tile is likely halftone — boost picture
                // likelihood to maximum.
                for (int y = ty; y < y_end; ++y) {
                    uint8_t* pa_line = pa_data + pa_stride * y;
                    for (int x = tx; x < x_end; ++x) {
                        pa_line[x] = 0xff;
                    }
                }
            }
        }
    }

    downscaled_input = GrayImage(); // Save memory.

    status.throwIfCancelled();

    // Adaptive threshold for the picture likelihood map.
    //
    // The original hardcoded threshold of 48 worked for typical pages but
    // was too aggressive on low-contrast scans (false picture detections)
    // and too conservative on high-contrast photographic content (missed
    // pictures).  The commented-out mokjiThreshold was presumably abandoned
    // because it was unreliable on the heavily skewed distribution this
    // image produces.
    //
    // We use Otsu's method, which finds the optimal threshold to separate
    // two classes (text/background vs. pictures).  Safety bounds [30, 80]
    // prevent pathological results.  If the image has almost no bright
    // pixels (no pictures present), we use a high threshold to suppress
    // noise-driven false positives.
    BinaryThreshold threshold(48); // fallback
    {
        int const w = picture_areas.width();
        int const h = picture_areas.height();
        int const total_pixels = w * h;

        if (total_pixels > 0) {
            // Count pixels above a moderate level.  If fewer than 0.5%
            // of pixels are bright, there are likely no real pictures —
            // use a high threshold to avoid false positives from noise.
            int bright_pixels = 0;
            uint8_t const* pa_line = picture_areas.data();
            int const pa_stride = picture_areas.stride();
            for (int y = 0; y < h; ++y, pa_line += pa_stride) {
                for (int x = 0; x < w; ++x) {
                    if (pa_line[x] >= 80) {
                        ++bright_pixels;
                    }
                }
            }

            if (bright_pixels < total_pixels / 200) {
                // No significant picture content detected.
                threshold = BinaryThreshold(80);
            } else {
                // Otsu's method on the picture likelihood image.
                int otsu = BinaryThreshold::otsuThreshold(picture_areas);

                // Clamp to [30, 80] — below 30 classifies too much as
                // picture (noise), above 80 misses real pictures.
                otsu = std::max(30, std::min(80, otsu));
                threshold = BinaryThreshold(otsu);
            }
        }
    }

    // Scale back to original size.
    picture_areas = scaleToGray(
                        picture_areas, source_sub_rect.size()
                    );

    return BinaryImage(picture_areas, threshold);
}

GrayImage
OutputGenerator::featherMask(BinaryImage const& bw_mask, float sigma)
{
    // Convert the binary binarization mask into a soft (grayscale) mask
    // with feathered transitions at picture zone boundaries.
    //
    // The binary mask has sharp 0/1 transitions that produce visible
    // seams in the output where binarized text abruptly meets color/gray
    // picture content.  Gaussian blur on the mask creates a smooth
    // gradient zone ~3*sigma pixels wide on each side of the boundary.
    //
    // Convention: 0 = picture zone (keep color), 255 = text zone (binarize).
    // This matches the binary mask where BLACK (1) = text, WHITE (0) = picture,
    // but inverted to grayscale levels.

    int const w = bw_mask.width();
    int const h = bw_mask.height();

    if (w <= 0 || h <= 0) {
        return GrayImage(QSize(w, h));
    }

    // Convert: BLACK pixels (text) → 255, WHITE pixels (picture) → 0.
    GrayImage gray(QSize(w, h));
    uint8_t* gray_line = gray.data();
    int const gray_stride = gray.stride();
    uint32_t const* bw_line = bw_mask.data();
    int const bw_wpl = bw_mask.wordsPerLine();
    uint32_t const msb = uint32_t(1) << 31;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            gray_line[x] = (bw_line[x >> 5] & (msb >> (x & 31))) ? 255 : 0;
        }
        gray_line += gray_stride;
        bw_line += bw_wpl;
    }

    // Gaussian blur creates the feathered transition.
    // sigma=2.0 at 300 DPI → ~6 pixel transition zone ≈ 0.5mm,
    // which is visually smooth but doesn't noticeably blur text.
    return gaussBlur(gray, sigma, sigma);
}

void
OutputGenerator::modifyBinarizationMask(
    imageproc::BinaryImage& bw_mask,
    QRect const& mask_rect, ZoneSet const& zones, int filter) const
{
    QTransform xform(m_xform.transform());
    xform *= QTransform().translate(-mask_rect.x(), -mask_rect.y());

    typedef PictureLayerProperty PLP;

    // Pass 1: ERASER1
    if (filter & BINARIZATION_MASK_ERASER1) {
        for (Zone const& zone : zones) {
            if (zone.properties().locateOrDefault<PLP>()->layer() == PLP::ERASER1) {
                if (zone.type() == Zone::SplineType) {
                    QPolygonF const poly(zone.spline().toPolygon());
                    PolygonRasterizer::fill(bw_mask, BLACK, xform.map(poly), Qt::WindingFill);
                } else if (zone.type() == Zone::EllipseType) {
                    QPainterPath path;
                    QTransform t;
                    t.translate(zone.ellipse().center().x(), zone.ellipse().center().y());
                    t.rotate(zone.ellipse().angle());
                    t.translate(-zone.ellipse().center().x(), -zone.ellipse().center().y());
                    path.addEllipse(zone.ellipse().center(), zone.ellipse().rx(), zone.ellipse().ry());
                    path = t.map(path);
//                    path = xform.map(path);
                    PolygonRasterizer::fill(bw_mask, BLACK, xform.map(path.toFillPolygon()), Qt::WindingFill);
                }
            }
        }
    }

    // Pass 2: PAINTER2
    if (filter & BINARIZATION_MASK_PAINTER2) {
        for (Zone const& zone : zones) {
            if (zone.properties().locateOrDefault<PLP>()->layer() == PLP::PAINTER2) {
                if (zone.type() == Zone::SplineType) {
                    QPolygonF const poly(zone.spline().toPolygon());
                    PolygonRasterizer::fill(bw_mask, WHITE, xform.map(poly), Qt::WindingFill);
                } else if (zone.type() == Zone::EllipseType) {
                    QPainterPath path;
                    QTransform t;
                    t.translate(zone.ellipse().center().x(), zone.ellipse().center().y());
                    t.rotate(zone.ellipse().angle());
                    t.translate(-zone.ellipse().center().x(), -zone.ellipse().center().y());
                    path.addEllipse(zone.ellipse().center(), zone.ellipse().rx(), zone.ellipse().ry());
                    path = t.map(path);
//                    path = xform.map(path);
                    PolygonRasterizer::fill(bw_mask, WHITE, xform.map(path.toFillPolygon()), Qt::WindingFill);
                }
            }
        }
    }

    // Pass 1: ERASER3
    if (filter & BINARIZATION_MASK_ERASER3) {
        for (Zone const& zone : zones) {
            if (zone.properties().locateOrDefault<PLP>()->layer() == PLP::ERASER3) {
                if (zone.type() == Zone::SplineType) {
                    QPolygonF const poly(zone.spline().toPolygon());
                    PolygonRasterizer::fill(bw_mask, BLACK, xform.map(poly), Qt::WindingFill);
                } else if (zone.type() == Zone::EllipseType) {
                    QPainterPath path;
                    QTransform t;
                    t.translate(zone.ellipse().center().x(), zone.ellipse().center().y());
                    t.rotate(zone.ellipse().angle());
                    t.translate(-zone.ellipse().center().x(), -zone.ellipse().center().y());
                    path.addEllipse(zone.ellipse().center(), zone.ellipse().rx(), zone.ellipse().ry());
                    path = t.map(path);
//                    path = xform.map(path);
                    PolygonRasterizer::fill(bw_mask, BLACK, xform.map(path.toFillPolygon()), Qt::WindingFill);
                }
            }
        }
    }
}

QImage
OutputGenerator::processImpl(
    TaskStatus const& status, FilterData const& input,
//Quadro_Zoner
    //ZoneSet const& picture_zones, ZoneSet const& fill_zones,
    ZoneSet& picture_zones, ZoneSet const& fill_zones,
    DewarpingMode dewarping_mode,
    DistortionModel& distortion_model,
    DepthPerception const& depth_perception,
//Original_Foreground_Mixed
    bool keep_orig_fore_subscan,
    imageproc::BinaryImage* auto_layer_mask,
    imageproc::BinaryImage* speckles_image,
    DebugImages* const dbg,
    PageId* p_pageId,
    IntrusivePtr<Settings>* p_settings
) const
{
    RenderParams const render_params(m_colorParams);

//begin of modified by monday2000
//Original_Foreground_Mixed
//added:

    if (keep_orig_fore_subscan) {
        if (dewarping_mode == DewarpingMode::AUTO ||
//begin of modified by monday2000
//Marginal_Dewarping
                dewarping_mode == DewarpingMode::MARGINAL ||
//end of modified by monday2000
                (dewarping_mode == DewarpingMode::MANUAL && distortion_model.isValid())) {
            return processWithDewarping(
                       status, input, picture_zones, fill_zones,
                       dewarping_mode, distortion_model, depth_perception,
                       keep_orig_fore_subscan,
                       auto_layer_mask, speckles_image, dbg,
                       p_pageId, p_settings
                   );
        } else return processAsIs(
                              input, status, fill_zones, depth_perception, dbg
                          );
    }

    if (dewarping_mode == DewarpingMode::AUTO ||
//begin of modified by monday2000
//Marginal_Dewarping
            dewarping_mode == DewarpingMode::MARGINAL ||
//end of modified by monday2000
            (dewarping_mode == DewarpingMode::MANUAL && distortion_model.isValid())) {
        return processWithDewarping(
                   status, input, picture_zones, fill_zones,
                   dewarping_mode, distortion_model, depth_perception,
                   false,
                   auto_layer_mask, speckles_image, dbg,
                   p_pageId, p_settings
               );
    } else if (!render_params.whiteMargins()) {
        return processAsIs(
                   input, status, fill_zones, depth_perception, dbg
               );
    } else {
        return processWithoutDewarping(
                   status, input, picture_zones, fill_zones,
                   auto_layer_mask, speckles_image, dbg,
                   p_pageId, p_settings
               );
    }
}

QImage
OutputGenerator::processAsIs(
    FilterData const& input, TaskStatus const& status,
    ZoneSet const& fill_zones,
    DepthPerception const& depth_perception,
    DebugImages* const dbg) const
{
    uint8_t const dominant_gray = reserveBlackAndWhite<uint8_t>(
                                      calcDominantBackgroundGrayLevel(input.grayImage())
                                  );

    status.throwIfCancelled();

    QColor const bg_color(dominant_gray, dominant_gray, dominant_gray);

    QImage out;
    CommandLine const& cli = CommandLine::get();

    if (input.origImage().allGray() && !cli.hasTiffForceKeepColorSpace()
        #ifdef HAVE_EXIV2
            && !( GlobalStaticSettings::m_output_copy_icc_metadata &&
                  ImageMetadataCopier::iccProfileDefined(input.origImageFilename()) )
        #endif
            ) {
        if (m_outRect.isEmpty()) {
            QImage image(1, 1, QImage::Format_Indexed8);
            image.setColorTable(createGrayscalePalette());
            if (image.isNull()) {
                throw std::bad_alloc();
            }
            image.fill(dominant_gray);
            return image;
        }

        out = transformToGray(
                  input.grayImage(), m_xform.transform(), m_outRect,
                  OutsidePixels::assumeColor(bg_color)
              );
    } else {
        if (m_outRect.isEmpty()) {
            QImage image(1, 1, QImage::Format_RGB32);
            image.fill(bg_color.rgb());
            return image;
        }

        out = transform(
                  input.origImage(), m_xform.transform(), m_outRect,
                  OutsidePixels::assumeColor(bg_color)
              );
    }

    applyFillZonesInPlace(out, fill_zones);
    reserveBlackAndWhite(out);

    return out;
}

QImage
OutputGenerator::processWithoutDewarping(TaskStatus const& status, FilterData const& input,
        ZoneSet& picture_zones, ZoneSet const& fill_zones,
        imageproc::BinaryImage* auto_layer_mask,
        imageproc::BinaryImage* speckles_image,
        DebugImages* dbg, PageId* p_pageId,
        IntrusivePtr<Settings>* p_settings
                                        ) const
{
    RenderParams const render_params(m_colorParams);
    const bool suppress_smoothing = GlobalStaticSettings::m_disable_bw_smoothing &&
                                    (m_colorParams.colorMode() == ColorParams::BLACK_AND_WHITE);

    // The whole image minus the part cut off by the split line.
    QRect const big_margins_rect(
        m_xform.resultingPreCropArea().boundingRect().toRect() | m_contentRect
    );

    // For various reasons, we need some whitespace around the content
    // area.  This is the number of pixels of such whitespace.
//begin of modified by monday2000
//Marginal_Dewarping
    //int const content_margin = m_dpi.vertical() * 20 / 300;
    int const content_margin = 40;
//end of modified by monday2000

    // The content area (in output image coordinates) extended
    // with content_margin.  Note that we prevent that extension
    // from reaching the neighboring page.
    QRect const small_margins_rect(
        m_contentRect.adjusted(
            -content_margin, -content_margin,
            content_margin, content_margin
        ).intersected(big_margins_rect)
    );

    // This is the area we are going to pass to estimateBackground().
    // estimateBackground() needs some margins around content, and
    // generally smaller margins are better, except when there is
    // some garbage that connects the content to the edge of the
    // image area.
    QRect const normalize_illumination_rect(
#if 1
        small_margins_rect
#else
        big_margins_rect
#endif
    );

    QImage maybe_normalized;

    // Crop area in original image coordinates.
    QPolygonF const orig_image_crop_area(
        m_xform.transformBack().map(
            m_xform.resultingPreCropArea()
        )
    );

    // Crop area in maybe_normalized image coordinates.
    QPolygonF normalize_illumination_crop_area(m_xform.resultingPreCropArea());
    normalize_illumination_crop_area.translate(-normalize_illumination_rect.topLeft());

    if (render_params.normalizeIllumination() || render_params.mixedOutput()) {
        maybe_normalized = normalizeIlluminationGray(
                               status, input.grayImage(), orig_image_crop_area,
                               m_xform.transform(), normalize_illumination_rect, 0, dbg
                           );
    } else {
        maybe_normalized = transform(
                               input.origImage(), m_xform.transform(),
                               normalize_illumination_rect, OutsidePixels::assumeColor(Qt::white)
                           );
    }

    status.throwIfCancelled();

    QImage maybe_smoothed;

    // We only do smoothing if we are going to do binarization later.
    if (!render_params.needBinarization() || suppress_smoothing) {
        maybe_smoothed = maybe_normalized;
    } else {
        maybe_smoothed =  smoothToGrayscale(maybe_normalized, m_dpi);
        if (dbg) {
            dbg->add(maybe_smoothed, "smoothed");
        }
    }

    status.throwIfCancelled();

    if (render_params.binaryOutput() || m_outRect.isEmpty()) {
        BinaryImage dst(m_outRect.size().expandedTo(QSize(1, 1)), WHITE);

        if (!m_contentRect.isEmpty()) {
            BinaryImage bw_content(
                binarize(maybe_smoothed, normalize_illumination_crop_area)
            );
            if (dbg) {
                dbg->add(bw_content, "binarized_and_cropped");
            }

            status.throwIfCancelled();

            if (!suppress_smoothing) {
                morphologicalSmoothInPlace(bw_content, status);
                if (dbg) {
                    dbg->add(bw_content, "edges_smoothed");
                }
            }

            status.throwIfCancelled();

            QRect const src_rect(m_contentRect.translated(-normalize_illumination_rect.topLeft()));
            QRect const dst_rect(m_contentRect);
            rasterOp<RopSrc>(dst, dst_rect, bw_content, src_rect.topLeft());
            bw_content.release(); // Save memory.

            // It's important to keep despeckling the very last operation
            // affecting the binary part of the output. That's because
            // we will be reconstructing the input to this despeckling
            // operation from the final output file.
            maybeDespeckleInPlace(
                dst, m_outRect, m_outRect, m_despeckleLevel,
                speckles_image, m_dpi, status, dbg
            );
        }

        applyFillZonesInPlace(dst, fill_zones);
        return dst.toQImage();
    }

    QSize const target_size(m_outRect.size().expandedTo(QSize(1, 1)));

    BinaryImage bw_mask;
    BinaryImage bw_auto_layer_mask;
    if (render_params.mixedOutput()) {
        // This block should go before the block with
        // adjustBrightnessGrayscale(), which may convert
        // maybe_normalized from grayscale to color mode.

        if (auto_layer_mask) {
            if (auto_layer_mask->size() != target_size) {
                BinaryImage(target_size).swap(*auto_layer_mask);
            }

            auto_layer_mask->fill(BLACK);
        }

        if (render_params.anyLayer()) {
            bw_mask = estimateBinarizationMask(
                          status, GrayImage(maybe_normalized),
                          normalize_illumination_rect,
                          small_margins_rect, dbg
                      );

            // Boost picture mask with color information.
            // Regions with significant chrominance are almost certainly
            // pictures, regardless of what the gradient-based detector found.
            if (!input.origImage().allGray()) {
                QImage color_output = transform(
                    input.origImage(), m_xform.transform(),
                    small_margins_rect, OutsidePixels::assumeColor(Qt::white)
                );
                boostMaskWithChroma(bw_mask, color_output, m_dpi);
            }

            if (dbg) {
                dbg->add(bw_mask, "bw_mask");
            }

            //Picture_Shape
            if (render_params.pictureZonesLayer()) {
                if (!picture_zones.auto_zones_found()) {
                    std::vector<QPolygonF> contours;
                    contourize(bw_mask, contours, GlobalStaticSettings::m_picture_detection_sensitivity);

                    QTransform xform1(m_xform.transform());
                    xform1 *= QTransform().translate(-small_margins_rect.x(), -small_margins_rect.y());

                    QTransform inv_xform(xform1.inverted());

                    for (int i = 0; i < (int)contours.size(); i++) {
                        QPolygonF area(inv_xform.map(contours[i]));

                        Zone zone1(area);

                        picture_zones.add(zone1);
                    }

                    picture_zones.setPictureZonesSensitivity(GlobalStaticSettings::m_picture_detection_sensitivity);
                    (*p_settings)->setPictureZones(*p_pageId, picture_zones);
                }

            } else {
                picture_zones.remove_auto_zones();
                (*p_settings)->setPictureZones(*p_pageId, picture_zones);
            }

            if (render_params.foregroundLayer()) {
                bw_auto_layer_mask = bw_mask; // need it later
            }

            if (render_params.autoLayer()) {
                if (!m_contentRect.isEmpty() && !render_params.foregroundLayer()) {
                    // if foregroundLayer - will have to overwrite auto_layer_mask later
                    // so just not wasting time
                    QRect const src_rect(m_contentRect.translated(-small_margins_rect.topLeft()));
                    QRect const dst_rect(m_contentRect);
                    rasterOp<RopSrc>(*auto_layer_mask, dst_rect, bw_mask, src_rect.topLeft());
                }
            } else {
                bw_mask = BinaryImage(maybe_normalized.size(), BLACK);
            }
        } else {
            bw_mask = BinaryImage(maybe_normalized.size(), BLACK);
        }

        status.throwIfCancelled();
    }

    if ((render_params.normalizeIllumination() && !input.origImage().allGray())
            || render_params.mixedOutput()) {
        // in case of mixedOutput we normalized image for picture detection and now should
        // restoren non-normalized image if it has !normalizeIllumination()
        QImage tmp;
        if (!input.origImage().allGray()
#ifdef HAVE_EXIV2
                || ( GlobalStaticSettings::m_output_copy_icc_metadata &&
                 ImageMetadataCopier::iccProfileDefined(input.origImageFilename()) )
#endif
                ) {
            assert(maybe_normalized.format() == QImage::Format_Indexed8);
            tmp = (
                      transform(
                          input.origImage(), m_xform.transform(),
                          normalize_illumination_rect,
                          OutsidePixels::assumeColor(Qt::white)
                      )
                  );

            status.throwIfCancelled();

            if (render_params.normalizeIllumination()) {
                adjustBrightnessGrayscale(tmp, maybe_normalized);
            }
        } else {
            tmp = (
                      transform(
                          input.grayImage(), m_xform.transform(),
                          normalize_illumination_rect,
                          OutsidePixels::assumeColor(Qt::white)
                      )
                  );
            status.throwIfCancelled();
        }
        maybe_normalized = tmp;
        if (dbg) {
            dbg->add(maybe_normalized, "norm_illum_color");
        }

    }

    if (!render_params.mixedOutput()) {
        // It's "Color / Grayscale" mode, as we handle B/W above.
        reserveBlackAndWhite(maybe_normalized);
    } else {

        if (!render_params.foregroundLayer()) {
            modifyBinarizationMask(bw_mask, small_margins_rect, picture_zones);
            if (dbg) {
                dbg->add(bw_mask, "bw_mask with zones");
            }
        }

        BinaryImage bw_content(
            binarize(maybe_smoothed, normalize_illumination_crop_area, &bw_mask)
        );

        std::unique_ptr<BinaryImage> foreground_mask = nullptr;
        if (render_params.foregroundLayer() &&
                (m_colorParams.blackWhiteOptions().thresholdAdjustment()
                 != m_colorParams.blackWhiteOptions().thresholdForegroundAdjustment())) {
            const int adj = m_colorParams.blackWhiteOptions().thresholdForegroundAdjustment();
            foreground_mask.reset(new BinaryImage(binarize(maybe_smoothed, normalize_illumination_crop_area, &bw_mask, &adj)));
        }

        maybe_smoothed = QImage(); // Save memory.
        if (dbg) {
            dbg->add(bw_content, "binarized_and_cropped");
        }

        status.throwIfCancelled();

        if (!suppress_smoothing) {
            morphologicalSmoothInPlace(bw_content, status);
            if (dbg) {
                dbg->add(bw_content, "edges_smoothed");
            }
            if (foreground_mask) {
                morphologicalSmoothInPlace(*foreground_mask, status);
            }
        }

        status.throwIfCancelled();

        // We don't want speckles in non-B/W areas, as they would
        // then get visualized on the Despeckling tab.
        rasterOp<RopAnd<RopSrc, RopDst> >(bw_content, bw_mask);
        if (foreground_mask) {
            rasterOp<RopAnd<RopSrc, RopDst> >(*foreground_mask, bw_mask);
        }

        status.throwIfCancelled();

        // It's important to keep despeckling the very last operation
        // affecting the binary part of the output. That's because
        // we will be reconstructing the input to this despeckling
        // operation from the final output file.
        maybeDespeckleInPlace(
            bw_content, small_margins_rect, m_contentRect,
            m_despeckleLevel, speckles_image, m_dpi, status, dbg
        );

        if (foreground_mask) {
            maybeDespeckleInPlace(
                *foreground_mask, small_margins_rect, m_contentRect,
                m_despeckleLevel, speckles_image, m_dpi, status, nullptr
            );
        }

        status.throwIfCancelled();

        if (render_params.foregroundLayer()) {
            if (foreground_mask) {
                bw_mask = foreground_mask->release();
                foreground_mask.reset(nullptr);
            } else {
                bw_mask = bw_content;
            }
            bw_mask.invert();

            BinaryImage new_auto_layer_mask = bw_mask;
            if (render_params.autoLayer()) {
                rasterOp<RopAnd<RopSrc, RopDst> >(new_auto_layer_mask, bw_auto_layer_mask);

                modifyBinarizationMask(bw_auto_layer_mask, small_margins_rect, picture_zones, BINARIZATION_MASK_ERASER1 | BINARIZATION_MASK_PAINTER2);
                rasterOp<RopAnd<RopSrc, RopDst> >(bw_mask, bw_auto_layer_mask);
                modifyBinarizationMask(bw_mask, small_margins_rect, picture_zones, BINARIZATION_MASK_ERASER3);
                bw_auto_layer_mask.release();
            } else {
                // apply all zones directly to color layer mask as we have no autolayer.
                modifyBinarizationMask(bw_mask, small_margins_rect, picture_zones);
            }

            if (!m_contentRect.isEmpty()) {
                QRect const src_rect(m_contentRect.translated(-small_margins_rect.topLeft()));
                QRect const dst_rect(m_contentRect);
                rasterOp<RopSrc>(*auto_layer_mask, dst_rect, new_auto_layer_mask, src_rect.topLeft());
            }

//            bw_content.fill(WHITE);
        }

        if (maybe_normalized.format() == QImage::Format_Indexed8) {
            GrayImage const soft_mask(featherMask(bw_mask));
            combineMixedFeathered<uint8_t>(
                maybe_normalized, bw_content, soft_mask
            );
        } else {
            assert(maybe_normalized.format() == QImage::Format_RGB32
                   || maybe_normalized.format() == QImage::Format_ARGB32);

            GrayImage const soft_mask(featherMask(bw_mask));
            combineMixedFeathered<uint32_t>(
                maybe_normalized, bw_content, soft_mask
            );
        }
    }

    status.throwIfCancelled();

    assert(!target_size.isEmpty());
    QImage dst(target_size, maybe_normalized.format());

    if (maybe_normalized.format() == QImage::Format_Indexed8) {
        dst.setColorTable(createGrayscalePalette());
        // White.  0xff is reserved if in "Color / Grayscale" mode.
        uint8_t const color = render_params.mixedOutput() ? 0xff : 0xfe;
        dst.fill(color);
    } else {
        // White.  0x[ff]ffffff is reserved if in "Color / Grayscale" mode.
        uint32_t const color = render_params.mixedOutput() ? 0xffffffff : 0xfffefefe;
        dst.fill(color);
    }

    if (dst.isNull()) {
        // Both the constructor and setColorTable() above can leave the image null.
        throw std::bad_alloc();
    }

    if (!m_contentRect.isEmpty()) {
        QRect const src_rect(m_contentRect.translated(-small_margins_rect.topLeft()));
        QRect const dst_rect(m_contentRect);
        drawOver(dst, dst_rect, maybe_normalized, src_rect);
    }

    applyFillZonesInPlace(dst, fill_zones);
    return dst;
}

QImage
OutputGenerator::processWithDewarping(TaskStatus const& status, FilterData const& input,
                                      ZoneSet& picture_zones, ZoneSet const& fill_zones,
                                      DewarpingMode dewarping_mode,
                                      DistortionModel& distortion_model,
                                      DepthPerception const& depth_perception,
//Original_Foreground_Mixed
                                      bool keep_orig_fore_subscan,
                                      imageproc::BinaryImage* auto_layer_mask,
                                      imageproc::BinaryImage* speckles_image,
                                      DebugImages* dbg, PageId* p_pageId,
                                      IntrusivePtr<Settings>* p_settings
                                     ) const
{
    QSize const target_size(m_outRect.size().expandedTo(QSize(1, 1)));
    if (m_outRect.isEmpty()) {
        return BinaryImage(target_size, WHITE).toQImage();
    }

    RenderParams const render_params(m_colorParams);
    const bool suppress_smoothing = GlobalStaticSettings::m_disable_bw_smoothing &&
                                    (m_colorParams.colorMode() == ColorParams::BLACK_AND_WHITE);

    // The whole image minus the part cut off by the split line.
    QRect const big_margins_rect(
        m_xform.resultingPreCropArea().boundingRect().toRect() | m_contentRect
    );

    // For various reasons, we need some whitespace around the content
    // area.  This is the number of pixels of such whitespace.
//begin of modified by monday2000
//Marginal_Dewarping
    //int const content_margin = m_dpi.vertical() * 20 / 300;
    int const content_margin = 40;
//end of modified by monday2000

    // The content area (in output image coordinates) extended
    // with content_margin.  Note that we prevent that extension
    // from reaching the neighboring page.
    QRect const small_margins_rect(
        m_contentRect.adjusted(
            -content_margin, -content_margin,
            content_margin, content_margin
        ).intersected(big_margins_rect)
    );

    // This is the area we are going to pass to estimateBackground().
    // estimateBackground() needs some margins around content, and
    // generally smaller margins are better, except when there is
    // some garbage that connects the content to the edge of the
    // image area.
    QRect const normalize_illumination_rect(
#if 1
        small_margins_rect
#else
        big_margins_rect
#endif
    );

    // Crop area in original image coordinates.
    QPolygonF const orig_image_crop_area(
        m_xform.transformBack().map(m_xform.resultingPreCropArea())
    );

    // Crop area in maybe_normalized image coordinates.
    QPolygonF normalize_illumination_crop_area(m_xform.resultingPreCropArea());
    normalize_illumination_crop_area.translate(-normalize_illumination_rect.topLeft());

    bool const color_original = !input.origImage().allGray()
#ifdef HAVE_EXIV2
            || ( GlobalStaticSettings::m_output_copy_icc_metadata &&
                 ImageMetadataCopier::iccProfileDefined(input.origImageFilename()) )
#endif
            ;

    // Original image, but:
    // 1. In a format we can handle, that is grayscale, RGB32, ARGB32
    // 2. With illumination normalized over the content area, if required.
    // 3. With margins filled with white, if required.
    QImage normalized_original;

    // The output we would get if dewarping was turned off, except always grayscale.
    // Used for automatic picture detection and binarization threshold calculation.
    // This image corresponds to the area of normalize_illumination_rect above.
    GrayImage warped_gray_output;

    // Picture mask (white indicate a picture) in the same coordinates as
    // warped_gray_output.  Only built for Mixed mode.
    BinaryImage warped_bw_mask;

    BinaryThreshold bw_threshold(128);

    QTransform const norm_illum_to_original(
        QTransform().translate(
            normalize_illumination_rect.left(),
            normalize_illumination_rect.top()
        ) * m_xform.transformBack()
    );

    if (!render_params.normalizeIllumination()) {
        if (color_original) {
            normalized_original = convertToRGBorRGBA(input.origImage());
        } else {
            normalized_original = input.grayImage();
        }
        if (dewarping_mode == DewarpingMode::AUTO
                || dewarping_mode == DewarpingMode::MARGINAL
                || render_params.mixedOutput()
           ) {
            warped_gray_output = transformToGray(
                                     input.grayImage(), m_xform.transform(), normalize_illumination_rect,
                                     OutsidePixels::assumeWeakColor(Qt::white)
                                 );
        } // Otherwise we just don't need it.
    } else {
        GrayImage warped_gray_background;
        warped_gray_output = normalizeIlluminationGray(
                                 status, input.grayImage(), orig_image_crop_area,
                                 m_xform.transform(), normalize_illumination_rect,
                                 &warped_gray_background, dbg
                             );

        status.throwIfCancelled();

        // Transform warped_gray_background to original image coordinates.
        warped_gray_background = transformToGray(
                                     warped_gray_background.toQImage(), norm_illum_to_original,
                                     input.origImage().rect(), OutsidePixels::assumeWeakColor(Qt::black)
                                 );
        if (dbg) {
            dbg->add(warped_gray_background, "orig_background");
        }

        status.throwIfCancelled();

        // Turn background into a grayscale, illumination-normalized image.
        grayRasterOp<RaiseAboveBackground>(warped_gray_background, input.grayImage());
        if (dbg) {
            dbg->add(warped_gray_background, "norm_illum_gray");
        }

        status.throwIfCancelled();

        if (!color_original || render_params.binaryOutput()) {
            normalized_original = warped_gray_background;
        } else {
            normalized_original = convertToRGBorRGBA(input.origImage());
            adjustBrightnessGrayscale(normalized_original, warped_gray_background);
            if (dbg) {
                dbg->add(normalized_original, "norm_illum_color");
            }
        }
    }

    status.throwIfCancelled();

    if (render_params.binaryOutput()) {
        bw_threshold = calcBinarizationThreshold(
                           warped_gray_output, normalize_illumination_crop_area
                       );

        status.throwIfCancelled();

    } else if (render_params.anyLayer()) {

        estimateBinarizationMask(
            status, GrayImage(warped_gray_output),
            normalize_illumination_rect,
            small_margins_rect, dbg
        ).swap(warped_bw_mask);

        // Boost picture mask with color information.
        if (color_original) {
            QImage color_output = transform(
                input.origImage(), m_xform.transform(),
                small_margins_rect, OutsidePixels::assumeColor(Qt::white)
            );
            boostMaskWithChroma(warped_bw_mask, color_output, m_dpi);
        }

        if (dbg) {
            dbg->add(warped_bw_mask, "warped_bw_mask");
        }

        if (render_params.pictureZonesLayer()) {
            if (!picture_zones.auto_zones_found()) {
                std::vector<QPolygonF> contours;
                contourize(warped_bw_mask, contours, GlobalStaticSettings::m_picture_detection_sensitivity);

                QTransform xform1(m_xform.transform());
                xform1 *= QTransform().translate(-small_margins_rect.x(), -small_margins_rect.y());

                QTransform inv_xform(xform1.inverted());

                for (int i = 0; i < (int)contours.size(); i++) {
                    QPolygonF area(inv_xform.map(contours[i]));

                    Zone zone1(area);

                    picture_zones.add(zone1);
                }

                picture_zones.setPictureZonesSensitivity(GlobalStaticSettings::m_picture_detection_sensitivity);
                (*p_settings)->setPictureZones(*p_pageId, picture_zones);

            }

        } else {
            picture_zones.remove_auto_zones();
            (*p_settings)->setPictureZones(*p_pageId, picture_zones);
        }

        status.throwIfCancelled();

        if (render_params.autoLayer()) {

            if (auto_layer_mask) {
                if (auto_layer_mask->size() != target_size) {
                    BinaryImage(target_size).swap(*auto_layer_mask);
                }
                auto_layer_mask->fill(BLACK);

                if (!m_contentRect.isEmpty()) {
                    QRect const src_rect(m_contentRect.translated(-small_margins_rect.topLeft()));
                    QRect const dst_rect(m_contentRect);
                    rasterOp<RopSrc>(*auto_layer_mask, dst_rect, warped_bw_mask, src_rect.topLeft());
                }
            }

            status.throwIfCancelled();

            modifyBinarizationMask(warped_bw_mask, small_margins_rect, picture_zones);
            if (dbg) {
                dbg->add(warped_bw_mask, "warped_bw_mask with zones");
            }

            status.throwIfCancelled();

            // For Mixed output, we mask out pictures when calculating binarization threshold.
            bw_threshold = calcBinarizationThreshold(
                               warped_gray_output, normalize_illumination_crop_area, &warped_bw_mask
                           );

            status.throwIfCancelled();
        } else {
            warped_bw_mask = BinaryImage(warped_gray_output.size(), BLACK);
        }
    }

    if (dewarping_mode == DewarpingMode::AUTO) {
        DistortionModelBuilder model_builder(Vec2d(0, 1));

        QRect const content_rect(
            m_contentRect.translated(-normalize_illumination_rect.topLeft())
        );
        TextLineTracer::trace(
            warped_gray_output, m_dpi, content_rect, model_builder, status, dbg
        );
        model_builder.transform(norm_illum_to_original);

        TopBottomEdgeTracer::trace(
            input.grayImage(), model_builder.verticalBounds(),
            model_builder, status, dbg
        );

        distortion_model = model_builder.tryBuildModel(dbg, &input.grayImage().toQImage());

        if (!distortion_model.isValid()) {
            setupTrivialDistortionModel(distortion_model);
        }

//begin of modified by monday2000
//Auto_Dewarping_Vert_Half_Correction
        if (GlobalStaticSettings::m_dewarpAutoVertHalfCorrection) {
            BinaryThreshold bw_threshold(64);
            BinaryImage bw_image(input.grayImage(), bw_threshold);

            QTransform transform = m_xform.preRotation().transform(bw_image.size());
            QTransform inv_transform = transform.inverted();

            int degrees = m_xform.preRotation().toDegrees();
            bw_image = orthogonalRotation(bw_image, degrees);

            std::vector<QPointF> const& top_polyline0 = distortion_model.topCurve().polyline();
            std::vector<QPointF> const& bottom_polyline0 = distortion_model.bottomCurve().polyline();

            std::vector<QPointF> top_polyline;
            std::vector<QPointF> bottom_polyline;

            for (int i = 0; i < (int)top_polyline0.size(); i++) {
                top_polyline.push_back(transform.map(top_polyline0[i]));
            }

            for (int i = 0; i < (int)bottom_polyline0.size(); i++) {
                bottom_polyline.push_back(transform.map(bottom_polyline0[i]));
            }

            //QImage out_image(bw_image.toQImage().convertToFormat(QImage::Format_RGB32));
            //for (int i=0; i<(int)top_polyline.size(); i++) drawPoint(out_image, top_polyline[i]);
            //for (int i=0; i<(int)bottom_polyline.size(); i++) drawPoint(out_image, bottom_polyline[i]);
            //TiffWriter::writeImage("C:\\bw_dewarp.tif", out_image);

            PageId const& pageId = *p_pageId;

            QString stAngle;

            float max_angle = 2.75; // chosen empirically

            //QFile file("C:\\st_angle.txt");
            //file.open(QIODevice::WriteOnly | QIODevice::Text);
            //QTextStream out(&file);
            //out << "degrees = " << QString::number(degrees) << endl;

            if (pageId.subPage() == PageId::SINGLE_PAGE || pageId.subPage() == PageId::LEFT_PAGE) {
                float vert_skew_angle_left = vert_border_skew_angle(top_polyline.front(), bottom_polyline.front());

                stAngle.setNum(vert_skew_angle_left);

                //out << "vert_skew_angle_left = " << stAngle << endl;

                if (vert_skew_angle_left > max_angle) {
                    //out << "vert_skew_angle_left correction" << endl;

                    float top_x = top_polyline.front().x();
                    float bottom_x = bottom_polyline.front().x();

                    if (top_x < bottom_x) {
                        std::vector<QPointF> new_bottom_polyline;

                        QPointF pt(top_x, bottom_polyline.front().y());

                        new_bottom_polyline.push_back(inv_transform.map(pt));

                        for (int i = 0; i < (int)bottom_polyline.size(); i++) {
                            new_bottom_polyline.push_back(inv_transform.map(bottom_polyline[i]));
                        }

                        distortion_model.setBottomCurve(dewarping::Curve(new_bottom_polyline));
                    } else {
                        std::vector<QPointF> new_top_polyline;

                        QPointF pt(bottom_x, top_polyline.front().y());

                        new_top_polyline.push_back(inv_transform.map(pt));

                        for (int i = 0; i < (int)top_polyline.size(); i++) {
                            new_top_polyline.push_back(inv_transform.map(top_polyline[i]));
                        }

                        distortion_model.setTopCurve(dewarping::Curve(new_top_polyline));
                    }
                }
            } else {
                float vert_skew_angle_right = vert_border_skew_angle(top_polyline.back(), bottom_polyline.back());

                stAngle.setNum(vert_skew_angle_right);

                //out << "vert_skew_angle_right = " << stAngle << endl;

                if (vert_skew_angle_right > max_angle) {
                    //out << "vert_skew_angle_right correction" << endl;

                    float top_x = top_polyline.back().x();
                    float bottom_x = bottom_polyline.back().x();

                    if (top_x > bottom_x) {
                        std::vector<QPointF> new_bottom_polyline;

                        QPointF pt(top_x, bottom_polyline.back().y());

                        for (int i = 0; i < (int)bottom_polyline.size(); i++) {
                            new_bottom_polyline.push_back(inv_transform.map(bottom_polyline[i]));
                        }

                        new_bottom_polyline.push_back(inv_transform.map(pt));

                        distortion_model.setBottomCurve(dewarping::Curve(new_bottom_polyline));
                    } else {
                        std::vector<QPointF> new_top_polyline;

                        QPointF pt(bottom_x, top_polyline.back().y());

                        for (int i = 0; i < (int)top_polyline.size(); i++) {
                            new_top_polyline.push_back(inv_transform.map(top_polyline[i]));
                        }

                        new_top_polyline.push_back(inv_transform.map(pt));

                        distortion_model.setTopCurve(dewarping::Curve(new_top_polyline));
                    }
                }
            }

            //file.close();
        }
//end of modified by monday2000

//begin of modified by monday2000
//Marginal_Dewarping
    } else if (dewarping_mode == DewarpingMode::MARGINAL) {
        BinaryThreshold bw_threshold(64);
        BinaryImage bw_image(input.grayImage(), bw_threshold);

        QTransform transform = m_xform.preRotation().transform(bw_image.size());
        QTransform inv_transform = transform.inverted();

        int degrees = m_xform.preRotation().toDegrees();
        bw_image = orthogonalRotation(bw_image, degrees);

        setupTrivialDistortionModel(distortion_model);

        PageId const& pageId = *p_pageId;

        int max_red_points = 5; //the more the curling the more this value

        XSpline top_spline;

        std::vector<QPointF> const& top_polyline = distortion_model.topCurve().polyline();

        QLineF const top_line(transform.map(top_polyline.front()), transform.map(top_polyline.back()));

        top_spline.appendControlPoint(top_line.p1(), 0);

        if (pageId.subPage() == PageId::SINGLE_PAGE || pageId.subPage() == PageId::LEFT_PAGE) {
            for (int i = 29 - max_red_points; i < 29; i++) {
                top_spline.appendControlPoint(top_line.pointAt((float)i / 29.0), 1);
            }
        } else {
            for (int i = 1; i <= max_red_points; i++) {
                top_spline.appendControlPoint(top_line.pointAt((float)i / 29.0), 1);
            }
        }

        top_spline.appendControlPoint(top_line.p2(), 0);

        for (int i = 0; i <= top_spline.numSegments(); i++) {
            movePointToTopMargin(bw_image, top_spline, i);
        }

        for (int i = 0; i <= top_spline.numSegments(); i++) {
            top_spline.moveControlPoint(i, inv_transform.map(top_spline.controlPointPosition(i)));
        }

        distortion_model.setTopCurve(dewarping::Curve(top_spline));

//bottom:

        XSpline bottom_spline;

        std::vector<QPointF> const& bottom_polyline = distortion_model.bottomCurve().polyline();

        QLineF const bottom_line(transform.map(bottom_polyline.front()), transform.map(bottom_polyline.back()));

        bottom_spline.appendControlPoint(bottom_line.p1(), 0);

        if (pageId.subPage() == PageId::SINGLE_PAGE || pageId.subPage() == PageId::LEFT_PAGE) {
            for (int i = 29 - max_red_points; i < 29; i++) {
                bottom_spline.appendControlPoint(top_line.pointAt((float)i / 29.0), 1);
            }
        } else {
            for (int i = 1; i <= max_red_points; i++) {
                bottom_spline.appendControlPoint(top_line.pointAt((float)i / 29.0), 1);
            }
        }

        bottom_spline.appendControlPoint(bottom_line.p2(), 0);

        for (int i = 0; i <= bottom_spline.numSegments(); i++) {
            movePointToBottomMargin(bw_image, bottom_spline, i);
        }

        for (int i = 0; i <= bottom_spline.numSegments(); i++) {
            bottom_spline.moveControlPoint(i, inv_transform.map(bottom_spline.controlPointPosition(i)));
        }

        distortion_model.setBottomCurve(dewarping::Curve(bottom_spline));

        if (!distortion_model.isValid()) {
            setupTrivialDistortionModel(distortion_model);
        }

        if (dbg) {
            QImage out_image(bw_image.toQImage().convertToFormat(QImage::Format_RGB32));
            for (int i = 0; i <= top_spline.numSegments(); i++) {
                drawPoint(out_image, top_spline.controlPointPosition(i));
            }
            for (int i = 0; i <= bottom_spline.numSegments(); i++) {
                drawPoint(out_image, bottom_spline.controlPointPosition(i));
            }
            dbg->add(out_image, "marginal dewarping");
        }
//end of modified by monday2000
    }

    warped_gray_output = GrayImage(); // Save memory.

    if (render_params.whiteMargins()) {
        // Fill everything except the content area in normalized_original to white.
        QPolygonF const orig_content_poly(m_xform.transformBack().map(QRectF(m_contentRect)));
        fillMarginsInPlace(normalized_original, orig_content_poly, Qt::white);
        if (dbg) {
            dbg->add(normalized_original, "white margins");
        }
    }

    status.throwIfCancelled();

    QColor bg_color(Qt::white);
    if (!render_params.whiteMargins()) {
        uint8_t const dominant_gray = reserveBlackAndWhite<uint8_t>(
                                          calcDominantBackgroundGrayLevel(input.grayImage())
                                      );
        bg_color = QColor(dominant_gray, dominant_gray, dominant_gray);
    }

    QImage dewarped;
    try {
        dewarped = dewarp(
                       QTransform(), normalized_original, m_xform.transform(),
                       distortion_model, depth_perception, bg_color
                   );
    } catch (std::runtime_error const&) {
        // Probably an impossible distortion model.  Let's fall back to a trivial one.
        setupTrivialDistortionModel(distortion_model);
        dewarped = dewarp(
                       QTransform(), normalized_original, m_xform.transform(),
                       distortion_model, depth_perception, bg_color
                   );
    }
    normalized_original = QImage(); // Save memory.
    if (dbg) {
        dbg->add(dewarped, "dewarped");
    }

    status.throwIfCancelled();

    QImage dewarped_and_maybe_smoothed;
    // We only do smoothing if we are going to do binarization later.
    if (!render_params.needBinarization() || suppress_smoothing) {
        dewarped_and_maybe_smoothed = dewarped;
    } else {
        dewarped_and_maybe_smoothed = smoothToGrayscale(dewarped, m_dpi);
        if (dbg) {
            dbg->add(dewarped_and_maybe_smoothed, "smoothed");
        }
    }

    boost::shared_ptr<DewarpingPointMapper> mapper(
        new DewarpingPointMapper(
            distortion_model, depth_perception.value(),
            m_xform.transform(), m_contentRect
        )
    );
    boost::function<QPointF(QPointF const&)> const orig_to_output(
        boost::bind(&DewarpingPointMapper::mapToDewarpedSpace, mapper, _1)
    );

    if (render_params.binaryOutput()) {
        BinaryImage dewarped_bw_content(dewarped_and_maybe_smoothed, bw_threshold);
        dewarped_and_maybe_smoothed = QImage(); // Save memory.
        if (dbg) {
            dbg->add(dewarped_bw_content, "dewarped_bw_content");
        }

        status.throwIfCancelled();

        if (!suppress_smoothing) {
            morphologicalSmoothInPlace(dewarped_bw_content, status);
            if (dbg) {
                dbg->add(dewarped_bw_content, "edges_smoothed");
            }
        }

        status.throwIfCancelled();

        // It's important to keep despeckling the very last operation
        // affecting the binary part of the output. That's because
        // we will be reconstructing the input to this despeckling
        // operation from the final output file.
        maybeDespeckleInPlace(
            dewarped_bw_content, m_outRect, m_outRect, m_despeckleLevel,
            speckles_image, m_dpi, status, dbg
        );

        applyFillZonesInPlace(dewarped_bw_content, fill_zones, orig_to_output);

        if (GlobalStaticSettings::m_dewarpAutoDeskewAfterDewarp) {
            QImage tmp_image(dewarped_bw_content.toQImage());
            maybe_deskew(&tmp_image, dewarping_mode);
            return tmp_image.convertToFormat(QImage::Format_Mono);
        } else {
            return dewarped_bw_content.toQImage();
        }
//end of modified by monday2000
    }

    if (!render_params.mixedOutput()) {
        // It's "Color / Grayscale" mode, as we handle B/W above.
        reserveBlackAndWhite(dewarped);
    } else {
        status.throwIfCancelled();

        // Dewarp the B/W mask.
        QTransform const orig_to_small_margins(
            m_xform.transform() * QTransform().translate(
                -small_margins_rect.left(),
                -small_margins_rect.top()
            )
        );
        QTransform small_margins_to_output;
        small_margins_to_output.translate(
            small_margins_rect.left(), small_margins_rect.top()
        );
        BinaryImage dewarped_bw_mask(
            dewarp(
                orig_to_small_margins, warped_bw_mask.toQImage(),
                small_margins_to_output, distortion_model,
                depth_perception, Qt::black
            )
        );
        if (dbg) {
            dbg->add(dewarped_bw_mask, "dewarped_bw_mask");
        }

        status.throwIfCancelled();

        BinaryImage dewarped_bw_content(dewarped_and_maybe_smoothed, bw_threshold);
        dewarped_and_maybe_smoothed = QImage(); // Save memory.
        if (dbg) {
            dbg->add(dewarped_bw_content, "dewarped_bw_content");
        }

        status.throwIfCancelled();

        if (!suppress_smoothing) {
            morphologicalSmoothInPlace(dewarped_bw_content, status);
            if (dbg) {
                dbg->add(dewarped_bw_content, "edges_smoothed");
            }
        }

        status.throwIfCancelled();

        // We don't want speckles in non-B/W areas, as they would
        // then get visualized on the Despeckling tab.
        rasterOp<RopAnd<RopSrc, RopDst> >(dewarped_bw_content, dewarped_bw_mask);

        status.throwIfCancelled();

        // It's important to keep despeckling the very last operation
        // affecting the binary part of the output. That's because
        // we will be reconstructing the input to this despeckling
        // operation from the final output file.
        maybeDespeckleInPlace(
            dewarped_bw_content, m_outRect, m_contentRect,
            m_despeckleLevel, speckles_image, m_dpi, status, dbg
        );

        status.throwIfCancelled();

        if (GlobalStaticSettings::m_dewarpAutoDeskewAfterDewarp) {
            double const angle = maybe_deskew(&dewarped, dewarping_mode);
            if (angle != 0.) {
                // we deskew img and mask before merging together
                // to prevent appearence of gray pixels around b/w text if it's roteted
                QImage tmp_img = dewarped_bw_content.toQImage();
                do_deskew(&tmp_img, angle);
                dewarped_bw_content = BinaryImage(tmp_img);
                tmp_img = dewarped_bw_mask.toQImage();
                do_deskew(&tmp_img, angle);
                dewarped_bw_mask = BinaryImage(tmp_img);
            }
        }

        if (dewarped.format() == QImage::Format_Indexed8) {
            GrayImage const soft_mask(featherMask(dewarped_bw_mask));
            combineMixedFeathered<uint8_t>(
                dewarped, dewarped_bw_content, soft_mask
            );
        } else {
            assert(dewarped.format() == QImage::Format_RGB32
                   || dewarped.format() == QImage::Format_ARGB32);

            GrayImage const soft_mask(featherMask(dewarped_bw_mask));
            combineMixedFeathered<uint32_t>(
                dewarped, dewarped_bw_content, soft_mask
            );
        }
    }

    applyFillZonesInPlace(dewarped, fill_zones, orig_to_output);
    return dewarped;
}

/**
 * Set up a distortion model corresponding to the content rect,
 * which will result in no distortion correction.
 */
void
OutputGenerator::setupTrivialDistortionModel(DistortionModel& distortion_model) const
{
    QPolygonF poly;
    if (!m_contentRect.isEmpty()) {
        poly = QRectF(m_contentRect);
    } else {
        poly << m_contentRect.topLeft() + QPointF(-0.5, -0.5);
        poly << m_contentRect.topLeft() + QPointF(0.5, -0.5);
        poly << m_contentRect.topLeft() + QPointF(0.5, 0.5);
        poly << m_contentRect.topLeft() + QPointF(-0.5, 0.5);
    }
    poly = m_xform.transformBack().map(poly);

    std::vector<QPointF> top_polyline, bottom_polyline;
    top_polyline.push_back(poly[0]); // top-left
    top_polyline.push_back(poly[1]); // top-right
    bottom_polyline.push_back(poly[3]); // bottom-left
    bottom_polyline.push_back(poly[2]); // bottom-right
    distortion_model.setTopCurve(Curve(top_polyline));
    distortion_model.setBottomCurve(Curve(bottom_polyline));
}

CylindricalSurfaceDewarper
OutputGenerator::createDewarper(
    DistortionModel const& distortion_model,
    QTransform const& distortion_model_to_target, double depth_perception)
{
    if (distortion_model_to_target.isIdentity()) {
        return CylindricalSurfaceDewarper(
                   distortion_model.topCurve().polyline(),
                   distortion_model.bottomCurve().polyline(), depth_perception
               );
    }

    std::vector<QPointF> top_polyline(distortion_model.topCurve().polyline());
    std::vector<QPointF> bottom_polyline(distortion_model.bottomCurve().polyline());
    for (QPointF& pt : top_polyline) {
        pt = distortion_model_to_target.map(pt);
    }
    for (QPointF& pt : bottom_polyline) {
        pt = distortion_model_to_target.map(pt);
    }
    return CylindricalSurfaceDewarper(
               top_polyline, bottom_polyline, depth_perception
           );
}

/**
 * \param orig_to_src Transformation from the original image coordinates
 *                    to the coordinate system of \p src image.
 * \param src_to_output Transformation from the \p src image coordinates
 *                      to output image coordinates.
 * \param distortion_model Distortion model.
 * \param depth_perception Depth perception.
 * \param bg_color The color to use for areas outsize of \p src.
 * \param modified_content_rect A vertically shrunk version of outputContentRect().
 *                              See function definition for more details.
 */
QImage
OutputGenerator::dewarp(
    QTransform const& orig_to_src, QImage const& src,
    QTransform const& src_to_output, DistortionModel const& distortion_model,
    DepthPerception const& depth_perception, QColor const& bg_color) const
{
    CylindricalSurfaceDewarper const dewarper(
        createDewarper(distortion_model, orig_to_src, depth_perception.value())
    );

    // Model domain is a rectangle in output image coordinates that
    // will be mapped to our curved quadrilateral.
    QRect const model_domain(
        distortion_model.modelDomain(
            dewarper, orig_to_src * src_to_output, outputContentRect()
        ).toRect()
    );
    if (model_domain.isEmpty()) {
        GrayImage out(src.size());
        out.fill(0xff); // white
        return out;
    }

    return RasterDewarper::dewarp(
               src, m_outRect.size(), dewarper, model_domain, bg_color
           );
}

QSize
OutputGenerator::from300dpi(QSize const& size, Dpi const& target_dpi)
{
    double const hscale = target_dpi.horizontal() / 300.0;
    double const vscale = target_dpi.vertical() / 300.0;
    int const width = qRound(size.width() * hscale);
    int const height = qRound(size.height() * vscale);
    return QSize(std::max(1, width), std::max(1, height));
}

QSize
OutputGenerator::to300dpi(QSize const& size, Dpi const& source_dpi)
{
    double const hscale = 300.0 / source_dpi.horizontal();
    double const vscale = 300.0 / source_dpi.vertical();
    int const width = qRound(size.width() * hscale);
    int const height = qRound(size.height() * vscale);
    return QSize(std::max(1, width), std::max(1, height));
}

QImage
OutputGenerator::convertToRGBorRGBA(QImage const& src)
{
    QImage::Format const fmt = src.hasAlphaChannel()
                               ? QImage::Format_ARGB32 : QImage::Format_RGB32;

    return src.convertToFormat(fmt);
}

void
OutputGenerator::fillMarginsInPlace(
    QImage& image, QPolygonF const& content_poly, QColor const& color)
{
    if (image.format() == QImage::Format_Indexed8 && image.isGrayscale()) {
        PolygonRasterizer::grayFillExcept(
            image, qGray(color.rgb()), content_poly, Qt::WindingFill
        );
        return;
    }

    assert(image.format() == QImage::Format_RGB32 || image.format() == QImage::Format_ARGB32);

    if (image.format() == QImage::Format_ARGB32) {
        image = image.convertToFormat(QImage::Format_ARGB32_Premultiplied);
    }

    {
        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setBrush(color);
        painter.setPen(Qt::NoPen);

        QPainterPath outer_path;
        outer_path.addRect(image.rect());
        QPainterPath inner_path;
        inner_path.addPolygon(content_poly);

        painter.drawPath(outer_path.subtracted(inner_path));
    }

    if (image.format() == QImage::Format_ARGB32_Premultiplied) {
        image = image.convertToFormat(QImage::Format_ARGB32);
    }
}

void
OutputGenerator::boostMaskWithChroma(
    BinaryImage& mask, QImage const& color_source,
    Dpi const& dpi)
{
    if (color_source.isNull() || color_source.allGray()) {
        return;
    }

    QSize const mask_size(mask.size());
    if (mask_size.isEmpty()) {
        return;
    }

    // Downscale the color source to 300 DPI for analysis,
    // matching the resolution used by detectPictures().
    QSize const downscaled_size(to300dpi(mask_size, dpi));
    if (downscaled_size.isEmpty()) {
        return;
    }

    QImage small_color = color_source.scaled(
        downscaled_size, Qt::IgnoreAspectRatio, Qt::SmoothTransformation
    ).convertToFormat(QImage::Format_RGB32);

    int const w = small_color.width();
    int const h = small_color.height();

    // Build a binary chroma mask: WHITE = significant color present.
    // Any pixel with substantial chrominance is almost certainly part
    // of a picture -- text is black/gray on white/cream background.
    //
    // Chroma metric: max(|R-G|, |R-B|, |G-B|).
    // Threshold of 25 catches visible color while ignoring the slight
    // warm/cool tint of aged paper or scanner white balance drift.
    BinaryImage chroma_mask(QSize(w, h), BLACK);
    int const chroma_thresh = 25;

    for (int y = 0; y < h; ++y) {
        QRgb const* src_line = reinterpret_cast<QRgb const*>(
            small_color.constScanLine(y)
        );
        for (int x = 0; x < w; ++x) {
            int const r = qRed(src_line[x]);
            int const g = qGreen(src_line[x]);
            int const b = qBlue(src_line[x]);
            int const rg = abs(r - g);
            int const rb = abs(r - b);
            int const gb = abs(g - b);
            int const chroma = std::max(rg, std::max(rb, gb));
            if (chroma > chroma_thresh) {
                chroma_mask.setPixel(x, y, WHITE);
            }
        }
    }

    small_color = QImage(); // Save memory.

    // Morphological close to bridge small gaps within colored regions
    // (halftone dots, dithering, JPEG artifacts in color areas).
    chroma_mask = closeBrick(chroma_mask, QSize(5, 5), WHITE);

    // Remove isolated small specks of color (stains, noise).
    // Opening removes tiny white regions that don't form coherent areas.
    chroma_mask = openBrick(chroma_mask, QSize(7, 7), WHITE);

    // Scale back to mask dimensions.
    GrayImage chroma_gray = scaleToGray(
        GrayImage(chroma_mask.toQImage()), mask_size
    );
    chroma_mask = BinaryImage(); // Save memory.

    BinaryImage chroma_upscaled(chroma_gray, BinaryThreshold(128));
    chroma_gray = GrayImage();

    // OR the chroma detections into the existing gradient-based mask.
    rasterOp<RopOr<RopSrc, RopDst> >(mask, chroma_upscaled);
}

void
OutputGenerator::contourize(
    BinaryImage const& mask, std::vector<QPolygonF>& contours,
    int sensitivity)
{
    // Convert the picture mask to contour polygons that tightly follow
    // the actual picture boundaries, replacing the axis-aligned rectangles
    // produced by rectangularize().
    //
    // Algorithm:
    //   1. Morphological close to bridge small gaps between fragments
    //      of the same picture (similar to rectangularize's 16px merge).
    //   2. Small dilation to add a safety margin around picture edges.
    //   3. Connected component iteration.
    //   4. Moore boundary tracing per component.
    //   5. Douglas-Peucker simplification to reduce vertex count.

    // Invert: WHITE (picture) → BLACK (foreground for CC iteration).
    BinaryImage inv(mask.inverted());

    // Close with 15×15 to bridge gaps up to ~14px (comparable to
    // rectangularize's 16px horizontal merge distance).
    inv = closeBrick(inv, QSize(15, 15));

    // Dilate by 3×3 to add a 1-pixel margin so the polygon doesn't
    // cut into the picture edge.
    inv = dilateBrick(inv, QSize(3, 3));

    // Minimum component area.  Higher sensitivity keeps smaller zones.
    // At default sensitivity (100): min_area = 500 pixels (~1.4mm²).
    int const minArea = std::max(100, 500 * (200 - sensitivity) / 100);

    ConnCompEraserExt eraser(inv, CONN8);
    for (;;) {
        ConnComp const cc(eraser.nextConnComp());
        if (cc.isNull()) {
            break;
        }

        if (cc.pixCount() < minArea) {
            continue;
        }

        BinaryImage ccImg(eraser.computeConnCompImage());
        QPolygonF boundary(traceMooreBoundary(ccImg));

        if (boundary.size() < 3) {
            continue;
        }

        // Douglas-Peucker simplification.  Epsilon of 2 pixels
        // at 300 DPI ≈ 0.17mm — imperceptible deviation, but
        // reduces a raw boundary of ~thousands of points to
        // typically 10-30 vertices.
        boundary = simplifyDP(boundary, 2.0);

        if (boundary.size() < 3) {
            continue;
        }

        // Offset from component-local to mask coordinates.
        QPointF const offset(cc.rect().topLeft());
        for (int i = 0; i < boundary.size(); ++i) {
            boundary[i] += offset;
        }

        contours.push_back(boundary);
    }
}

GrayImage
OutputGenerator::detectPictures(
    GrayImage const& input_300dpi, TaskStatus const& status,
    DebugImages* const dbg)
{
    // We stretch the range of gray levels to cover the whole
    // range of [0, 255].  We do it because we want text
    // and background to be equally far from the center
    // of the whole range.  Otherwise text printed with a big
    // font will be considered a picture.
    GrayImage stretched(stretchGrayRange(input_300dpi, 0.01, 0.01));
    if (dbg) {
        dbg->add(stretched, "stretched");
    }

    status.throwIfCancelled();

    // Fine-scale gradient (3x3): captures sharp edges.
    GrayImage eroded(erodeGray(stretched, QSize(3, 3), 0x00));
    GrayImage dilated(dilateGray(stretched, QSize(3, 3), 0xff));

    status.throwIfCancelled();

    grayRasterOp<CombineInverted>(dilated, eroded);
    GrayImage gray_gradient(dilated);
    dilated = GrayImage();
    eroded = GrayImage();

    // Multi-scale gradient analysis.
    //
    // The original single-scale 3x3 gradient captures sharp edges well
    // but responds weakly to gradual intensity transitions found in
    // photographs with soft focus, watercolors, and smooth gradients.
    // A coarser 9x9 gradient responds more strongly to these broad
    // transitions while still capturing text edges.
    //
    // We compute the coarse gradient, then take the pixel-wise maximum
    // with the fine gradient.  This boosts picture regions with gradual
    // transitions (where the 9x9 gradient is significantly stronger
    // than the 3x3) while leaving text edges unchanged (both scales
    // produce similar magnitudes for sharp step edges).
    //
    // The combined gradient then goes through the single opening-by-
    // reconstruction pipeline, which removes text-scale features and
    // preserves picture-scale features as before.
    {
        GrayImage coarse_eroded(erodeGray(stretched, QSize(9, 9), 0x00));
        GrayImage coarse_dilated(dilateGray(stretched, QSize(9, 9), 0xff));

        status.throwIfCancelled();

        grayRasterOp<CombineInverted>(coarse_dilated, coarse_eroded);
        // coarse_dilated now holds the coarse gradient.
        coarse_eroded = GrayImage();

        // Pixel-wise maximum: boost gray_gradient where the coarse
        // gradient is stronger.
        int const w = gray_gradient.width();
        int const h = gray_gradient.height();
        uint8_t* fine_line = gray_gradient.data();
        int const fine_stride = gray_gradient.stride();
        uint8_t const* coarse_line = coarse_dilated.data();
        int const coarse_stride = coarse_dilated.stride();
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (coarse_line[x] > fine_line[x]) {
                    fine_line[x] = coarse_line[x];
                }
            }
            fine_line += fine_stride;
            coarse_line += coarse_stride;
        }
    }

    stretched = GrayImage(); // Save memory.

    if (dbg) {
        dbg->add(gray_gradient, "gray_gradient");
    }

    status.throwIfCancelled();

    // Adaptive structuring element size for the opening-by-reconstruction.
    //
    // The original hardcoded 35x35 was tuned for ~12pt text at 300 DPI
    // (line spacing ~33px).  Large display fonts (chapter titles, headings)
    // have gradient features that survive a 35x35 erosion and get falsely
    // classified as pictures.  Small dense text (footnotes, CJK) has the
    // opposite problem -- the SE is too large relative to the features.
    //
    // We estimate the dominant text line spacing from the horizontal
    // projection profile of the gradient image, then set the SE size
    // to approximately match it.  This ensures the SE always removes
    // text-scale gradient features while preserving picture-scale ones.
    int se_size = 35; // default fallback
    {
        int const w = gray_gradient.width();
        int const h = gray_gradient.height();

        if (w > 40 && h > 80) {
            // Compute horizontal projection profile: sum of gradient
            // values per row.  Text lines produce periodic peaks.
            std::vector<double> profile(h, 0.0);
            uint8_t const* line = gray_gradient.data();
            int const stride = gray_gradient.stride();
            for (int y = 0; y < h; ++y, line += stride) {
                double sum = 0;
                for (int x = 0; x < w; ++x) {
                    sum += line[x];
                }
                profile[y] = sum;
            }

            // Autocorrelation to find dominant line spacing.
            // Search for the first peak between lags 10 and 80 pixels
            // (at 300 DPI: ~0.85mm to ~6.8mm, covering 6pt to ~50pt text).
            int const min_lag = 10;
            int const max_lag = std::min(80, h / 3);
            double best_corr = 0;
            int best_lag = 0;

            // Compute mean for zero-centering.
            double mean = 0;
            for (int y = 0; y < h; ++y) {
                mean += profile[y];
            }
            mean /= h;

            for (int lag = min_lag; lag <= max_lag; ++lag) {
                double corr = 0;
                int const n = h - lag;
                for (int y = 0; y < n; ++y) {
                    corr += (profile[y] - mean) * (profile[y + lag] - mean);
                }
                if (corr > best_corr) {
                    best_corr = corr;
                    best_lag = lag;
                }
            }

            if (best_lag >= min_lag) {
                // SE size ≈ line spacing, clamped to [21, 71] and forced odd.
                se_size = best_lag;
                se_size = std::max(21, std::min(71, se_size));
                if ((se_size & 1) == 0) {
                    ++se_size;
                }
            }
        }
    }

    GrayImage marker(erodeGray(gray_gradient, QSize(se_size, se_size), 0x00));
    if (dbg) {
        dbg->add(marker, "marker");
    }

    status.throwIfCancelled();

    seedFillGrayInPlace(marker, gray_gradient, CONN8);
    gray_gradient = GrayImage(); // Save memory.
    GrayImage reconstructed(marker);
    marker = GrayImage(); // Save memory.

    if (dbg) {
        dbg->add(reconstructed, "reconstructed");
    }

    status.throwIfCancelled();

    grayRasterOp<GRopInvert<GRopSrc> >(reconstructed, reconstructed);
    if (dbg) {
        dbg->add(reconstructed, "reconstructed_inverted");
    }

    status.throwIfCancelled();

    GrayImage holes_filled(createFramedImage(reconstructed.size()));
    seedFillGrayInPlace(holes_filled, reconstructed, CONN8);
    reconstructed = GrayImage();
    if (dbg) {
        dbg->add(holes_filled, "holes_filled");
    }

    return holes_filled;
}

QImage
OutputGenerator::smoothToGrayscale(QImage const& src, Dpi const& dpi)
{
    int const min_dpi = std::min(dpi.horizontal(), dpi.vertical());
    int window;
    int degree;
    if (min_dpi <= 200) {
        window = 5;
        degree = 3;
    } else if (min_dpi <= 400) {
        window = 7;
        degree = 4;
    } else if (min_dpi <= 800) {
        window = 11;
        degree = 4;
    } else {
        window = 11;
        degree = 2;
    }
    return savGolFilter(src, QSize(window, window), degree, degree);
}

BinaryThreshold
OutputGenerator::adjustThreshold(BinaryThreshold threshold, const int* adjustment) const
{
    int adjusted = threshold;
    if (!adjustment) {
        adjusted += m_colorParams.blackWhiteOptions().thresholdAdjustment();
    } else {
        adjusted += *adjustment;
    }

    // Hard-bounding threshold values is necessary for example
    // if all the content went into the picture mask.
    return BinaryThreshold(qBound(30, adjusted, 225));
}

BinaryThreshold
OutputGenerator::calcBinarizationThreshold(
    QImage const& image, BinaryImage const& mask) const
{
    GrayscaleHistogram hist(image, mask);
    return adjustThreshold(BinaryThreshold::otsuThreshold(hist));
}

BinaryThreshold
OutputGenerator::calcBinarizationThreshold(
    QImage const& image, QPolygonF const& crop_area, BinaryImage const* mask) const
{
    QPainterPath path;
    path.addPolygon(crop_area);

    if (path.contains(image.rect())) {
        return adjustThreshold(BinaryThreshold::otsuThreshold(image));
    } else {
        BinaryImage modified_mask(image.size(), BLACK);
        PolygonRasterizer::fillExcept(modified_mask, WHITE, crop_area, Qt::WindingFill);
        modified_mask = erodeBrick(modified_mask, QSize(3, 3), WHITE);

        if (mask) {
            rasterOp<RopAnd<RopSrc, RopDst> >(modified_mask, *mask);
        }

        return calcBinarizationThreshold(image, modified_mask);
    }
}

BinaryImage
OutputGenerator::binarize(QImage const& image, BinaryImage const& mask, const int* adjustment) const
{
    BlackWhiteOptions const& black_white_options = m_colorParams.blackWhiteOptions();
    ThresholdFilter const thresholdMethod = black_white_options.thresholdMethod();

    int const threshold_delta = black_white_options.thresholdAdjustment();
    QSize const window_size = QSize(black_white_options.thresholdWindowSize(), black_white_options.thresholdWindowSize());
    double const threshold_coef = black_white_options.thresholdCoef();

    BinaryImage binarized;
    if ((image.format() == QImage::Format_Mono) || (image.format() == QImage::Format_MonoLSB))
    {
        binarized = BinaryImage(image);
    }
    else
    {
        switch (thresholdMethod)
        {
        case T_OTSU:
        {
            GrayscaleHistogram hist(image, mask);
            BinaryThreshold const bw_thresh(BinaryThreshold::otsuThreshold(hist));
            binarized = BinaryImage(image, adjustThreshold(bw_thresh, adjustment));
            break;
        }
        case T_SAUVOLA:
        {
            binarized = binarizeSauvola(image, window_size, threshold_coef, threshold_delta);
            break;
        }
        case T_WOLF:
        {
            binarized = binarizeWolf(image, window_size, 1, 254, threshold_coef, threshold_delta);
            break;
        }
        case T_WINDOW:
        {
            binarized = binarizeWindow(image, window_size, 1, 254, threshold_coef, threshold_delta);
            break;
        }
        case T_BRADLEY:
        {
            binarized = binarizeBradley(image, window_size, threshold_coef, threshold_delta);
            break;
        }
        case T_GRAD:
        {
            binarized = binarizeGrad(image, window_size, threshold_coef, threshold_delta);
            break;
        }
        case T_EDGEPLUS:
        {
            binarized = binarizeEdgeDiv(image, window_size, threshold_coef, 0.0, threshold_delta);
            break;
        }
        case T_BLURDIV:
        {
            binarized = binarizeEdgeDiv(image, window_size, 0.0, threshold_coef, threshold_delta);
            break;
        }
        case T_EDGEDIV:
        {
            binarized = binarizeEdgeDiv(image, window_size, threshold_coef, threshold_coef, threshold_delta);
            break;
        }
        }

    }

    // Fill masked out areas with white.
    rasterOp<RopAnd<RopSrc, RopDst> >(binarized, mask);

    return binarized;
}

BinaryImage
OutputGenerator::binarize(QImage const& image,
                          QPolygonF const& crop_area, BinaryImage const* mask, const int* adjustment) const
{
    QPainterPath path;
    path.addPolygon(crop_area);

    if (path.contains(image.rect()) && !mask) {
        BinaryThreshold const bw_thresh(BinaryThreshold::otsuThreshold(image));
        return BinaryImage(image, adjustThreshold(bw_thresh, adjustment));
    } else {
        BinaryImage modified_mask(image.size(), BLACK);
        PolygonRasterizer::fillExcept(modified_mask, WHITE, crop_area, Qt::WindingFill);
        modified_mask = erodeBrick(modified_mask, QSize(3, 3), WHITE);

        if (mask) {
            rasterOp<RopAnd<RopSrc, RopDst> >(modified_mask, *mask);
        }

        return binarize(image, modified_mask, adjustment);
    }
}

/**
 * \brief Remove small connected components that are considered to be garbage.
 *
 * Both the size and the distance to other components are taken into account.
 *
 * \param[in,out] image The image to despeckle.
 * \param image_rect The rectangle corresponding to \p image in the same
 *        coordinate system where m_contentRect and m_cropRect are defined.
 * \param mask_rect The area within the image to consider.  Defined not
 *        relative to \p image, but in the same coordinate system where
 *        m_contentRect and m_cropRect are defined.  This only affects
 *        \p speckles_img, if provided.
 * \param level Despeckling aggressiveness.
 * \param speckles_img If provided, the removed black speckles will be written
 *        there.  The speckles image is always considered to correspond
 *        to m_cropRect, so it will have the size of m_cropRect.size().
 *        Only the area within \p mask_rect will be copied to \p speckles_img.
 *        The rest will be filled with white.
 * \param dpi The DPI of the input image.  See the note below.
 * \param status Task status.
 * \param dbg An optional sink for debugging images.
 *
 * \note This function only works effectively when the DPI is symmetric,
 * that is, its horizontal and vertical components are equal.
 */
void
OutputGenerator::maybeDespeckleInPlace(
    imageproc::BinaryImage& image,
    QRect const& image_rect, QRect const& mask_rect,
    DespeckleLevel const level, BinaryImage* speckles_img,
    Dpi const& dpi, TaskStatus const& status, DebugImages* dbg) const
{
    QRect const src_rect(mask_rect.translated(-image_rect.topLeft()));
    QRect const dst_rect(mask_rect);

    if (speckles_img) {
        BinaryImage(m_outRect.size(), WHITE).swap(*speckles_img);
        if (!mask_rect.isEmpty()) {
            rasterOp<RopSrc>(*speckles_img, dst_rect, image, src_rect.topLeft());
        }
    }

    if (level != DESPECKLE_OFF) {
        Despeckle::Level lvl = Despeckle::NORMAL;
        switch (level) {
        case DESPECKLE_CAUTIOUS:
            lvl = Despeckle::CAUTIOUS;
            break;
        case DESPECKLE_NORMAL:
            lvl = Despeckle::NORMAL;
            break;
        case DESPECKLE_AGGRESSIVE:
            lvl = Despeckle::AGGRESSIVE;
            break;
        default:;
        }

        Despeckle::despeckleInPlace(image, dpi, lvl, status, dbg);

        if (dbg) {
            dbg->add(image, "despeckled");
        }
    }

    if (speckles_img) {
        if (!mask_rect.isEmpty()) {
            rasterOp<RopSubtract<RopDst, RopSrc> >(
                *speckles_img, dst_rect, image, src_rect.topLeft()
            );
        }
    }
}

void
OutputGenerator::morphologicalSmoothInPlace(
    BinaryImage& bin_img, TaskStatus const& status)
{
    // When removing black noise, remove small ones first.

    {
        char const pattern[] =
            "XXX"
            " - "
            "   ";
        hitMissReplaceAllDirections(bin_img, pattern, 3, 3);
    }

    status.throwIfCancelled();

    {
        char const pattern[] =
            "X ?"
            "X  "
            "X- "
            "X- "
            "X  "
            "X ?";
        hitMissReplaceAllDirections(bin_img, pattern, 3, 6);
    }

    status.throwIfCancelled();

    {
        char const pattern[] =
            "X ?"
            "X ?"
            "X  "
            "X- "
            "X- "
            "X- "
            "X  "
            "X ?"
            "X ?";
        hitMissReplaceAllDirections(bin_img, pattern, 3, 9);
    }

    status.throwIfCancelled();

    {
        char const pattern[] =
            "XX?"
            "XX?"
            "XX "
            "X+ "
            "X+ "
            "X+ "
            "XX "
            "XX?"
            "XX?";
        hitMissReplaceAllDirections(bin_img, pattern, 3, 9);
    }

    status.throwIfCancelled();

    {
        char const pattern[] =
            "XX?"
            "XX "
            "X+ "
            "X+ "
            "XX "
            "XX?";
        hitMissReplaceAllDirections(bin_img, pattern, 3, 6);
    }

    status.throwIfCancelled();

    {
        char const pattern[] =
            "   "
            "X+X"
            "XXX";
        hitMissReplaceAllDirections(bin_img, pattern, 3, 3);
    }
}

void
OutputGenerator::hitMissReplaceAllDirections(
    imageproc::BinaryImage& img, char const* const pattern,
    int const pattern_width, int const pattern_height)
{
    hitMissReplaceInPlace(img, WHITE, pattern, pattern_width, pattern_height);

    std::vector<char> pattern_data(pattern_width * pattern_height, ' ');
    char* const new_pattern = &pattern_data[0];

    // Rotate 90 degrees clockwise.
    char const* p = pattern;
    int new_width = pattern_height;
    int new_height = pattern_width;
    for (int y = 0; y < pattern_height; ++y) {
        for (int x = 0; x < pattern_width; ++x, ++p) {
            int const new_x = pattern_height - 1 - y;
            int const new_y = x;
            new_pattern[new_y * new_width + new_x] = *p;
        }
    }
    hitMissReplaceInPlace(img, WHITE, new_pattern, new_width, new_height);

    // Rotate upside down.
    p = pattern;
    new_width = pattern_width;
    new_height = pattern_height;
    for (int y = 0; y < pattern_height; ++y) {
        for (int x = 0; x < pattern_width; ++x, ++p) {
            int const new_x = pattern_width - 1 - x;
            int const new_y = pattern_height - 1 - y;
            new_pattern[new_y * new_width + new_x] = *p;
        }
    }
    hitMissReplaceInPlace(img, WHITE, new_pattern, new_width, new_height);

    // Rotate 90 degrees counter-clockwise.
    p = pattern;
    new_width = pattern_height;
    new_height = pattern_width;
    for (int y = 0; y < pattern_height; ++y) {
        for (int x = 0; x < pattern_width; ++x, ++p) {
            int const new_x = y;
            int const new_y = pattern_width - 1 - x;
            new_pattern[new_y * new_width + new_x] = *p;
        }
    }
    hitMissReplaceInPlace(img, WHITE, new_pattern, new_width, new_height);
}

QSize
OutputGenerator::calcLocalWindowSize(Dpi const& dpi)
{
    QSizeF const size_mm(3, 30);
    QSizeF const size_inch(size_mm * constants::MM2INCH);
    QSizeF const size_pixels_f(
        dpi.horizontal() * size_inch.width(),
        dpi.vertical() * size_inch.height()
    );
    QSize size_pixels(size_pixels_f.toSize());

    if (size_pixels.width() < 3) {
        size_pixels.setWidth(3);
    }
    if (size_pixels.height() < 3) {
        size_pixels.setHeight(3);
    }

    return size_pixels;
}

unsigned char
OutputGenerator::calcDominantBackgroundGrayLevel(QImage const& img)
{
    // TODO: make a color version.
    // In ColorPickupInteraction.cpp we have code for median color finding.
    // We can use that.

    QImage const gray(toGrayscale(img));

    BinaryImage mask(binarizeOtsu(gray));
    mask.invert();

    GrayscaleHistogram const hist(gray, mask);

    int integral_hist[256];
    integral_hist[0] = hist[0];
    for (int i = 1; i < 256; ++i) {
        integral_hist[i] = hist[i] + integral_hist[i - 1];
    }

    int const num_colors = 256;
    int const window_size = 10;

    int best_pos = 0;
    int best_sum = integral_hist[window_size - 1];
    for (int i = 1; i <= num_colors - window_size; ++i) {
        int const sum = integral_hist[i + window_size - 1] - integral_hist[i - 1];
        if (sum > best_sum) {
            best_sum = sum;
            best_pos = i;
        }
    }

    int half_sum = 0;
    for (int i = best_pos; i < best_pos + window_size; ++i) {
        half_sum += hist[i];
        if (half_sum >= best_sum / 2) {
            return i;
        }
    }

    assert(!"Unreachable");
    return 0;
}

void
OutputGenerator::applyFillZonesInPlace(
    QImage& img, ZoneSet const& zones,
    boost::function<QPointF(QPointF const&)> const& orig_to_output) const
{
    if (zones.empty()) {
        return;
    }

    QImage canvas(img.convertToFormat(QImage::Format_ARGB32_Premultiplied));

    {
        QPainter painter(&canvas);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(Qt::NoPen);

        for (Zone const& zone : zones) {
            QColor const color(zone.properties().locateOrDefault<FillColorProperty>()->color());
            painter.setBrush(color);
            if (zone.type() == Zone::SplineType) {
                QPolygonF const poly(zone.spline().transformed(orig_to_output).toPolygon());
                painter.drawPolygon(poly, Qt::WindingFill);
            } else if (zone.type() == Zone::EllipseType) {
                const SerializableEllipse e = zone.ellipse().transformed(orig_to_output);
                QPainterPath path;
                QTransform t;
                t.translate(e.center().x(), e.center().y());
                t.rotate(e.angle());
                t.translate(-e.center().x(), -e.center().y());
                path.addEllipse(e.center(), e.rx(), e.ry());
                path = t.map(path);
                painter.drawPolygon(path.toFillPolygon(), Qt::WindingFill);
            }
        }
    }

    if (img.format() == QImage::Format_Indexed8 && img.isGrayscale()) {
        img = toGrayscale(canvas);
    } else {
        img = canvas.convertToFormat(img.format());
    }
}

/**
 * A simplified version of the above, using toOutput() for translation
 * from original image to output image coordinates.
 */
void
OutputGenerator::applyFillZonesInPlace(QImage& img, ZoneSet const& zones) const
{
    typedef QPointF(QTransform::*MapPointFunc)(QPointF const&) const;
    applyFillZonesInPlace(
        img, zones, boost::bind((MapPointFunc)&QTransform::map, m_xform.transform(), _1)
    );
}

void
OutputGenerator::applyFillZonesInPlace(
    imageproc::BinaryImage& img, ZoneSet const& zones,
    boost::function<QPointF(QPointF const&)> const& orig_to_output) const
{
    if (zones.empty()) {
        return;
    }

    for (Zone const& zone : zones) {
        QColor const color(zone.properties().locateOrDefault<FillColorProperty>()->color());
        BWColor const bw_color = qGray(color.rgb()) < 128 ? BLACK : WHITE;
        if (zone.type() == Zone::SplineType) {
            QPolygonF const poly(zone.spline().transformed(orig_to_output).toPolygon());
            PolygonRasterizer::fill(img, bw_color, poly, Qt::WindingFill);
        } else if (zone.type() == Zone::EllipseType) {
            const SerializableEllipse e = zone.ellipse().transformed(orig_to_output);
            QPainterPath path;
            QTransform t;
            t.translate(e.center().x(), e.center().y());
            t.rotate(e.angle());
            t.translate(-e.center().x(), -e.center().y());
            path.addEllipse(e.center(), e.rx(), e.ry());
            path = t.map(path);
            PolygonRasterizer::fill(img, bw_color, path.toFillPolygon(), Qt::WindingFill);
        }
    }
}

/**
 * A simplified version of the above, using toOutput() for translation
 * from original image to output image coordinates.
 */
void
OutputGenerator::applyFillZonesInPlace(
    imageproc::BinaryImage& img, ZoneSet const& zones) const
{
    typedef QPointF(QTransform::*MapPointFunc)(QPointF const&) const;
    applyFillZonesInPlace(
        img, zones, boost::bind((MapPointFunc)&QTransform::map, m_xform.transform(), _1)
    );
}

//begin of modified by monday2000
//Marginal_Dewarping
void
OutputGenerator::movePointToTopMargin(BinaryImage& bw_image, XSpline& spline, int idx) const //added
{
    QPointF pos = spline.controlPointPosition(idx);

    for (int j = 0; j < pos.y(); j++) {
        if (bw_image.getPixel(pos.x(), j) == WHITE) {
            int count = 0;
            int check_num = 16;

            for (int jj = j; jj < (j + check_num); jj++) {
                if (bw_image.getPixel(pos.x(), jj) == WHITE) {
                    count++;
                }
            }

            if (count == check_num) {
                pos.setY(j);

                spline.moveControlPoint(idx, pos);

                break;
            }
        }
    }
}

void
OutputGenerator::movePointToBottomMargin(BinaryImage& bw_image, XSpline& spline, int idx) const //added
{
    QPointF pos = spline.controlPointPosition(idx);

    for (int j = bw_image.height() - 1; j > pos.y(); j--) {
        if (bw_image.getPixel(pos.x(), j) == WHITE) {
            int count = 0;
            int check_num = 16;

            for (int jj = j; jj > (j - check_num); jj--) {
                if (bw_image.getPixel(pos.x(), jj) == WHITE) {
                    count++;
                }
            }

            if (count == check_num) {
                pos.setY(j);

                spline.moveControlPoint(idx, pos);

                break;
            }
        }
    }
}

void
OutputGenerator::drawPoint(QImage& image, QPointF const& pt) const
{
    QPoint pts = pt.toPoint();

    for (int i = pts.x() - 10; i < pts.x() + 10; i++) {
        for (int j = pts.y() - 10; j < pts.y() + 10; j++) {

            QPoint p1(i, j);

            image.setPixel(p1, qRgb(255, 0, 0));

        }
    }
}

void
OutputGenerator::movePointToTopMargin(BinaryImage& bw_image, std::vector<QPointF>& polyline, int idx) const //added
{
    QPointF& pos = polyline[idx];

    for (int j = 0; j < pos.y(); j++) {
        if (bw_image.getPixel(pos.x(), j) == WHITE) {
            int count = 0;
            int check_num = 16;

            for (int jj = j; jj < (j + check_num); jj++) {
                if (bw_image.getPixel(pos.x(), jj) == WHITE) {
                    count++;
                }
            }

            if (count == check_num) {
                pos.setY(j);

                break;
            }
        }
    }
}

void
OutputGenerator::movePointToBottomMargin(BinaryImage& bw_image, std::vector<QPointF>& polyline, int idx) const //added
{
    QPointF& pos = polyline[idx];

    for (int j = bw_image.height() - 1; j > pos.y(); j--) {
        if (bw_image.getPixel(pos.x(), j) == WHITE) {
            int count = 0;
            int check_num = 16;

            for (int jj = j; jj > (j - check_num); jj--) {
                if (bw_image.getPixel(pos.x(), jj) == WHITE) {
                    count++;
                }
            }

            if (count == check_num) {
                pos.setY(j);

                break;
            }
        }
    }
}

float
OutputGenerator::vert_border_skew_angle(QPointF const& top, QPointF const& bottom) const
{
    return qFabs(qAtan((bottom.x() - top.x()) / (bottom.y() - top.y())) * 180 / M_PI);
}

double
OutputGenerator::maybe_deskew(QImage* p_dewarped, DewarpingMode dewarping_mode) const
{
    if (dewarping_mode == DewarpingMode::MARGINAL ||
            dewarping_mode == DewarpingMode::MANUAL) {
        BinaryThreshold bw_threshold(128);
        BinaryImage bw_image(*p_dewarped, bw_threshold);

        SkewFinder skew_finder;
        Skew const skew(skew_finder.findSkew(bw_image));
        double const angle = skew.angle();
        if (angle != 0.0 && skew.confidence() >= Skew::GOOD_CONFIDENCE) {
            do_deskew(p_dewarped, angle);
        }
        return angle;
    }

    return 0.;
}

void
OutputGenerator::do_deskew(QImage* p_image, double angle_deg) const
{
    if (angle_deg == 0.) {
        return;
    }

    QPointF center(p_image->width() / 2, p_image->height() / 2);

    QTransform rot;
    rot.translate(center.x(), center.y());
    rot.rotate(-angle_deg);
    rot.translate(-center.x(), -center.y());

    *p_image = imageproc::transform(*p_image, rot, p_image->rect(), OutsidePixels::assumeWeakColor(Qt::white));
}

} // namespace output
