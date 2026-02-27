/*
    Scan Tailor - Interactive post-processing tool for scanned pages.
    Copyright (C) 2007-2008  Joseph Artsimovich <joseph_a@mail.ru>

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

#include "SkewFinder.h"
#include "BinaryImage.h"
#include "BWColor.h"
#include "BitOps.h"
#include "Shear.h"
#include "ReduceThreshold.h"
#include "Constants.h"
#include <QDebug>
#include <algorithm>
#include <stdexcept>
#include <stdint.h>
#include <math.h>

namespace imageproc
{

double const Skew::GOOD_CONFIDENCE = 2.0;

double const SkewFinder::DEFAULT_MAX_ANGLE = 7.0;

double const SkewFinder::DEFAULT_ACCURACY = 0.1;

int const SkewFinder::DEFAULT_COARSE_REDUCTION = 2;

int const SkewFinder::DEFAULT_FINE_REDUCTION = 1;

double const SkewFinder::LOW_SCORE = 1000.0;

SkewFinder::SkewFinder()
    :   m_maxAngle(DEFAULT_MAX_ANGLE),
        m_accuracy(DEFAULT_ACCURACY),
        m_resolutionRatio(1.0),
        m_coarseReduction(DEFAULT_COARSE_REDUCTION),
        m_fineReduction(DEFAULT_FINE_REDUCTION)
{
}

void
SkewFinder::setMaxAngle(double const max_angle)
{
    if (max_angle < 0.0 || max_angle > 45.0) {
        throw std::invalid_argument("SkewFinder: max skew angle is invalid");
    }
    m_maxAngle = max_angle;
}

void
SkewFinder::setDesiredAccuracy(double const accuracy)
{
    m_accuracy = accuracy;
}

void
SkewFinder::setCoarseReduction(int const reduction)
{
    if (reduction < 0) {
        throw std::invalid_argument("SkewFinder: coarse reduction is invalid");
    }
    m_coarseReduction = reduction;
}

void
SkewFinder::setFineReduction(int const reduction)
{
    if (reduction < 0) {
        throw std::invalid_argument("SkewFinder: fine reduction is invalid");
    }
    m_fineReduction = reduction;
}

void
SkewFinder::setResolutionRatio(double const ratio)
{
    if (ratio <= 0.0) {
        throw std::invalid_argument("SkewFinder: resolution ratio is invalid");
    }
    m_resolutionRatio = ratio;
}

Skew
SkewFinder::findSkew(BinaryImage const& image) const
{
    if (image.isNull()) {
        throw std::invalid_argument("SkewFinder: null image was provided");
    }

    ReduceThreshold coarse_reduced(image);
    int const min_reduction = std::min(m_coarseReduction, m_fineReduction);
    for (int i = 0; i < min_reduction; ++i) {
        coarse_reduced.reduce(i == 0 ? 1 : 2);
    }

    ReduceThreshold fine_reduced(coarse_reduced.image());

    for (int i = min_reduction; i < m_coarseReduction; ++i) {
        coarse_reduced.reduce(i == 0 ? 1 : 2);
    }

    BinaryImage skewed(coarse_reduced.image().size());
    double const coarse_step = 1.0; // degrees

    // Coarse linear search.
    int num_coarse_scores = 0;
    double sum_coarse_scores = 0.0;
    double best_coarse_score = 0.0;
    double best_coarse_angle = -m_maxAngle;
    for (double angle = -m_maxAngle; angle <= m_maxAngle; angle += coarse_step) {
        double const score = process(coarse_reduced, skewed, angle);
        sum_coarse_scores += score;
        ++num_coarse_scores;
        if (score > best_coarse_score) {
            best_coarse_angle = angle;
            best_coarse_score = score;
        }
    }

    if (m_accuracy >= coarse_step) {
        double confidence = 0.0;
        if (num_coarse_scores > 1) {
            confidence = best_coarse_score /
                         sum_coarse_scores * num_coarse_scores;
        }
        return Skew(-best_coarse_angle, confidence - 1.0);
    }

    for (int i = min_reduction; i < m_fineReduction; ++i) {
        fine_reduced.reduce(i == 0 ? 1 : 2);
    }

    if (m_coarseReduction != m_fineReduction) {
        skewed = BinaryImage(fine_reduced.image().size());
    }

    // Fine binary search.
    double angle_plus = best_coarse_angle + 0.5 * coarse_step;
    double angle_minus = best_coarse_angle - 0.5 * coarse_step;
    double score_plus = process(fine_reduced, skewed, angle_plus);
    double score_minus = process(fine_reduced, skewed, angle_minus);
    double const fine_score1 = score_plus;
    double const fine_score2 = score_minus;
    while (angle_plus - angle_minus > m_accuracy) {
        if (score_plus > score_minus) {
            angle_minus = 0.5 * (angle_plus + angle_minus);
            score_minus = process(fine_reduced, skewed, angle_minus);
        } else if (score_plus < score_minus) {
            angle_plus = 0.5 * (angle_plus + angle_minus);
            score_plus = process(fine_reduced, skewed, angle_plus);
        } else {
            // This protects us from unreasonably low m_accuracy.
            break;
        }
    }

    double best_angle;
    double best_score;
    if (score_plus > score_minus) {
        best_angle = angle_plus;
        best_score = score_plus;
    } else {
        best_angle = angle_minus;
        best_score = score_minus;
    }

    if (best_score <= LOW_SCORE) {
        return Skew(-best_angle, 0.0); // Zero confidence.
    }

    double confidence = 0.0;
    if (num_coarse_scores > 1) {
        confidence = best_score / sum_coarse_scores * num_coarse_scores;
    } else {
        int num_scores = num_coarse_scores;
        double sum_scores = sum_coarse_scores;
        num_scores += 2;
        sum_scores += fine_score1;
        sum_scores += fine_score2;
        confidence = best_score / sum_scores * num_scores;
    }
    confidence -= 1.0;

    // Cross-validate with centroid-slope estimator.
    // The centroid method is more robust to pictures/illustrations
    // because uniform-density regions contribute centroids that average
    // to the image center, while text lines all slant consistently.
    double centroid_angle = 0.0;
    double centroid_quality = 0.0;
    bool const centroid_ok = calcCentroidSkew(
        image, m_resolutionRatio, centroid_angle, centroid_quality
    );

    if (centroid_ok && centroid_quality > 0.15) {
        double const angle_diff = fabs(-best_angle - centroid_angle);

        if (angle_diff < 0.5) {
            // Methods agree closely -- boost confidence.
            confidence *= 1.0 + 0.5 * centroid_quality;
        } else if (angle_diff > 2.0 && centroid_quality > 0.4) {
            // Strong disagreement with a high-quality centroid estimate.
            // Pictures likely corrupted the projection-profile result.
            // Switch to the centroid angle with reduced confidence.
            best_angle = -centroid_angle;
            confidence *= 0.5;
        } else if (angle_diff > 1.0) {
            // Moderate disagreement -- reduce confidence to flag
            // the result as uncertain without changing the angle.
            confidence *= 0.7;
        }
    }

    return Skew(-best_angle, confidence);
}

double
SkewFinder::process(BinaryImage const& src, BinaryImage& dst, double const angle) const
{
    double const tg = tan(angle * constants::DEG2RAD);
    double const x_center = 0.5 * dst.width();
    vShearFromTo(src, dst, tg / m_resolutionRatio, x_center, WHITE);
    return calcScore(dst);
}

double
SkewFinder::calcScore(BinaryImage const& image)
{
    int const width = image.width();
    int const height = image.height();
    uint32_t const* line = image.data();
    int const wpl = image.wordsPerLine();
    int const last_word_idx = (width - 1) >> 5;
    uint32_t const last_word_mask = ~uint32_t(0) << (31 - ((width - 1) & 31));

    double score = 0.0;
    int last_line_black_pixels = 0;
    for (int y = 0; y < height; ++y, line += wpl) {
        int num_black_pixels = 0;
        int i = 0;
        for (; i != last_word_idx; ++i) {
            num_black_pixels += countNonZeroBits(line[i]);
        }
        num_black_pixels += countNonZeroBits(line[i] & last_word_mask);

        if (y != 0) {
            double const diff = num_black_pixels - last_line_black_pixels;
            score += diff * diff;
        }
        last_line_black_pixels = num_black_pixels;
    }

    return score;
}

bool
SkewFinder::calcCentroidSkew(
    BinaryImage const& image, double const resolution_ratio,
    double& angle_degrees, double& quality)
{
    int const width = image.width();
    int const height = image.height();
    uint32_t const* line = image.data();
    int const wpl = image.wordsPerLine();
    int const last_word_idx = (width - 1) >> 5;
    uint32_t const last_word_mask = ~uint32_t(0) << (31 - ((width - 1) & 31));

    // Weighted least-squares fit of per-row centroids.
    // We accumulate:
    //   sum_w   = sum of weights (black pixel counts)
    //   sum_wy  = sum of weight * y
    //   sum_wc  = sum of weight * centroid_x
    //   sum_wyy = sum of weight * y^2
    //   sum_wyc = sum of weight * y * centroid_x
    //   sum_wcc = sum of weight * centroid_x^2
    // Then fit centroid_x = a + b * y via weighted least squares.
    // The skew angle is atan(b / resolution_ratio).

    double sum_w = 0;
    double sum_wy = 0;
    double sum_wc = 0;
    double sum_wyy = 0;
    double sum_wyc = 0;
    double sum_wcc = 0;
    int rows_with_data = 0;

    // Minimum black pixels per row to include in the fit.
    // This filters out nearly-empty rows that would add noise.
    int const min_pixels = std::max(3, width / 100);

    for (int y = 0; y < height; ++y, line += wpl) {
        // Compute weighted centroid of black pixels in this row.
        // For each 32-bit word, we need both the count and the
        // sum of x-positions of set bits.
        int num_black = 0;
        double x_sum = 0.0;

        for (int word_idx = 0; word_idx <= last_word_idx; ++word_idx) {
            uint32_t word = line[word_idx];
            if (word_idx == last_word_idx) {
                word &= last_word_mask;
            }
            if (word == 0) {
                continue;
            }
            int const base_x = word_idx << 5;
            // Process bits.  Using bit manipulation for speed.
            uint32_t w = word;
            while (w) {
                // Find highest set bit (MSB = x=0 in the word).
                int const bit = countNonZeroBits((w & (-w)) - 1);
                int const x = base_x + (31 - bit);
                x_sum += x;
                ++num_black;
                w &= w - 1; // Clear lowest set bit.
            }
        }

        if (num_black < min_pixels) {
            continue;
        }

        double const centroid = x_sum / num_black;
        double const wt = num_black; // Weight = black pixel count.

        sum_w += wt;
        sum_wy += wt * y;
        sum_wc += wt * centroid;
        sum_wyy += wt * y * y;
        sum_wyc += wt * y * centroid;
        sum_wcc += wt * centroid * centroid;
        ++rows_with_data;
    }

    if (rows_with_data < 10 || sum_w < 1.0) {
        angle_degrees = 0.0;
        quality = 0.0;
        return false;
    }

    // Weighted least-squares slope: b = (sum_wyc - sum_wy*sum_wc/sum_w) /
    //                                   (sum_wyy - sum_wy*sum_wy/sum_w)
    double const denom = sum_wyy - sum_wy * sum_wy / sum_w;
    if (fabs(denom) < 1e-10) {
        angle_degrees = 0.0;
        quality = 0.0;
        return false;
    }

    double const slope = (sum_wyc - sum_wy * sum_wc / sum_w) / denom;

    // Convert slope to angle, accounting for resolution ratio.
    // slope = dx/dy in pixel coords.  If resolution_ratio != 1,
    // the actual angle is atan(slope / resolution_ratio).
    angle_degrees = atan(slope / resolution_ratio) * constants::RAD2DEG;

    // Compute R^2 as quality measure.
    // R^2 = 1 - SS_res / SS_tot  where
    //   SS_tot = sum_wcc - sum_wc^2/sum_w
    //   SS_res = SS_tot - (sum_wyc - sum_wy*sum_wc/sum_w)^2 / denom
    double const ss_tot = sum_wcc - sum_wc * sum_wc / sum_w;
    if (ss_tot < 1e-10) {
        // All centroids are the same -- perfectly centered content.
        quality = 0.0;
        return true;
    }

    double const numer = sum_wyc - sum_wy * sum_wc / sum_w;
    double const ss_res = ss_tot - numer * numer / denom;
    quality = 1.0 - ss_res / ss_tot;
    quality = std::max(0.0, std::min(1.0, quality));

    return true;
}

} // namespace imageproc
