/*
    Scan Tailor - Interactive post-processing tool for scanned pages.
    Copyright (C)  Joseph Artsimovich <joseph.artsimovich@gmail.com>

    Based on code from the GIMP project,
    Copyright (C) 1995 Spencer Kimball and Peter Mattis

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

#ifndef IMAGEPROC_GAUSSBLUR_H_
#define IMAGEPROC_GAUSSBLUR_H_

#include "ValueConv.h"
#include <QSize>
#ifndef Q_MOC_RUN
#include <boost/scoped_array.hpp>
#endif
#include <iterator>
#include <string.h>

namespace imageproc
{

class GrayImage;

/**
 * \brief Applies gaussian blur on a GrayImage.
 *
 * \param src The image to apply gaussian blur to.
 * \param h_sigma The standard deviation in horizontal direction.
 * \param v_sigma The standard deviation in vertical direction.
 * \return The blurred image.
 */
GrayImage gaussBlur(GrayImage const& src, float h_sigma, float v_sigma);

/**
 * \brief Applies a 2D gaussian filter on an arbitrary data grid.
 *
 * \param size Data grid dimensions.
 * \param h_sigma The standard deviation in horizontal direction.
 * \param v_sigma The standard deviation in vertical direction.
 * \param input A random access iterator (usually a pointer)
 *        to the beginning of input data.
 * \param input_stride The distance (in terms of iterator difference)
 *        from an input grid cell to the one directly below it.
 * \param float_reader A functor to convert whatever value corresponds to *input
 *        into a float.  Consider using one of the functors from ValueConv.h
 *        The functor will be called like this:
 * \code
 * FloatReader const reader = ...;
 * float const val = reader(input[x]);
 * \endcode
 * \param output A random access iterator (usually a pointer)
 *        to the beginning of output data.  Output may point to the same
 *        memory as input.
 * \param output_stride The distance (in terms of iterator difference)
 *        from an output grid cell to the one directly below it.
 * \param float_writer A functor that takes a float value, optionally
 *        converts it into another type and updates an output item.
 *        The functor will be called like this:
 * \code
 * FloatWriter const writer = ...;
 * float const val = ...;
 * writer(output[x], val);
 * \endcode
 * When type conversion is required, consider using one of the functors from ValueConv.h:
 * \code
 * RoundAndClipValueConv<uint8_t> const float2byte;
 * gaussBlurGeneric(..., [float2byte](uint8_t& dst, float src) { dst = float2byte(src); });
 * \endcode
 */
template<typename SrcIt, typename DstIt, typename FloatReader, typename FloatWriter>
void gaussBlurGeneric(QSize size, float h_sigma, float v_sigma,
                      SrcIt input, int input_stride, FloatReader float_reader,
                      DstIt output, int output_stride, FloatWriter float_writer);

namespace gauss_blur_impl
{

void find_iir_constants(
    float* n_p, float* n_m, float* d_p,
    float* d_m, float* bd_p, float* bd_m, float std_dev);

template<typename Src1It, typename Src2It, typename DstIt, typename FloatWriter>
void save(int num_items, Src1It src1, Src2It src2,
          DstIt dst, int dst_stride, FloatWriter writer)
{
    while (num_items-- != 0) {
        writer(*dst, *src1 + *src2);
        ++src1;
        ++src2;
        dst += dst_stride;
    }
}

class FloatToFloatWriter
{
public:
    void operator()(float& dst, float src) const
    {
        dst = src;
    }
};

} // namespace gauss_blur_impl

template<typename SrcIt, typename DstIt, typename FloatReader, typename FloatWriter>
void gaussBlurGeneric(QSize const size, float const h_sigma, float const v_sigma,
                      SrcIt const input, int const input_stride, FloatReader const float_reader,
                      DstIt const output, int const output_stride, FloatWriter const float_writer)
{
    if (size.isEmpty()) {
        return;
    }

    int const width = size.width();
    int const height = size.height();
    int const width_height_max = width > height ? width : height;

    boost::scoped_array<float> val_p(new float[width_height_max]);
    boost::scoped_array<float> val_m(new float[width_height_max]);
    boost::scoped_array<float> intermediate_image(new float[width * height]);
    int const intermediate_stride = width;

    // IIR parameters.
    float n_p[5], n_m[5], d_p[5], d_m[5], bd_p[5], bd_m[5];

    // Vertical pass.
    // Process columns in strips so that source reads and intermediate
    // writes access memory sequentially (cache-friendly) rather than
    // striding across the full image width per pixel.
    gauss_blur_impl::find_iir_constants(n_p, n_m, d_p, d_m, bd_p, bd_m, v_sigma);

    int const STRIP_W = 64; // columns per strip; one cache line of uint8_t
    boost::scoped_array<float> strip_vp(new float[STRIP_W * height]);

    for (int x0 = 0; x0 < width; x0 += STRIP_W) {
        int const sw = (x0 + STRIP_W <= width) ? STRIP_W : (width - x0);

        // --- Phase 1: forward pass (top to bottom) ---
        // Store results in strip_vp in row-major order: strip_vp[y * sw + dx].
        memset(&strip_vp[0], 0, sw * height * sizeof(float));

        // Initial values (top row of each column in strip).
        // We need these for the boundary terms.
        boost::scoped_array<float> initial_p(new float[sw]);
        boost::scoped_array<float> initial_m(new float[sw]);
        {
            SrcIt src_top(input + x0);
            SrcIt src_bot(input + x0 + (height - 1) * input_stride);
            for (int dx = 0; dx < sw; ++dx) {
                initial_p[dx] = float_reader(src_top[dx]);
                initial_m[dx] = float_reader(src_bot[dx]);
            }
        }

        for (int y = 0; y < height; ++y) {
            SrcIt src_row(input + x0 + y * input_stride);
            float* vp_row = &strip_vp[y * sw];
            int const terms = y < 4 ? y : 4;

            for (int dx = 0; dx < sw; ++dx) {
                float val = 0;
                int i = 0;
                for (; i <= terms; ++i) {
                    float src_val = float_reader(src_row[dx - i * input_stride]);
                    float prev_vp = vp_row[dx - i * sw];
                    val += n_p[i] * src_val - d_p[i] * prev_vp;
                }
                for (; i <= 4; ++i) {
                    val += (n_p[i] - bd_p[i]) * initial_p[dx];
                }
                vp_row[dx] = val;
            }
        }

        // --- Phase 2: backward pass (bottom to top) + save ---
        // Compute val_m on the fly and combine with stored val_p,
        // writing directly to intermediate_image in row-major order.
        //
        // We keep a rolling window of the last 5 val_m rows for the IIR.
        float vm_hist[5][STRIP_W];
        memset(vm_hist, 0, sizeof(vm_hist));

        for (int y = height - 1; y >= 0; --y) {
            SrcIt src_row(input + x0 + y * input_stride);
            int const terms_from_bottom = (height - 1 - y) < 4 ? (height - 1 - y) : 4;
            int const cur = y % 5;

            for (int dx = 0; dx < sw; ++dx) {
                float val = 0;
                int i = 0;
                for (; i <= terms_from_bottom; ++i) {
                    float src_val = float_reader(src_row[dx + i * input_stride]);
                    float prev_vm = vm_hist[(y + i) % 5][dx];
                    val += n_m[i] * src_val - d_m[i] * prev_vm;
                }
                for (; i <= 4; ++i) {
                    val += (n_m[i] - bd_m[i]) * initial_m[dx];
                }
                vm_hist[cur][dx] = val;

                // Combine forward + backward and write to intermediate.
                intermediate_image[y * intermediate_stride + x0 + dx] =
                    strip_vp[y * sw + dx] + val;
            }
        }
    }

    // Horizontal pass.
    gauss_blur_impl::find_iir_constants(n_p, n_m, d_p, d_m, bd_p, bd_m, h_sigma);
    float const* intermediate_line = &intermediate_image[0];
    DstIt output_line(output);
    for (int y = 0; y < height; ++y) {
        memset(&val_p[0], 0, width * sizeof(val_p[0]));
        memset(&val_m[0], 0, width * sizeof(val_m[0]));

        float const* sp_p = intermediate_line;
        float const* sp_m = intermediate_line + width - 1;
        float* vp = &val_p[0];
        float* vm = &val_m[0] + width - 1;
        float const initial_p = sp_p[0];
        float const initial_m = sp_m[0];

        for (int x = 0; x < width; ++x) {
            int const terms = x < 4 ? x : 4;
            int i = 0;
            for (; i <= terms; ++i) {
                *vp += n_p[i] * sp_p[-i] - d_p[i] * vp[-i];
                *vm += n_m[i] * sp_m[i] - d_m[i] * vm[i];
            }
            for (; i <= 4; ++i) {
                *vp += (n_p[i] - bd_p[i]) * initial_p;
                *vm += (n_m[i] - bd_m[i]) * initial_m;
            }
            ++sp_p;
            --sp_m;
            ++vp;
            --vm;
        }

        gauss_blur_impl::save(width, &val_p[0], &val_m[0], output_line, 1, float_writer);

        intermediate_line += intermediate_stride;
        output_line += output_stride;
    }
}

} // namespace imageproc

#endif
