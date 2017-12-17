// Copyright (C) 2014  Davis E. King (davis@dlib.net)
// Copyright (C) 2015-2017  David Hirvonen
// License: Boost Software License   See LICENSE.txt for the full license.
#ifndef __drishti_ml_shape_predictor_h__
#define __drishti_ml_shape_predictor_h__

#define DRISHTI_DLIB_DO_DEBUG_ELLIPSE 0
#define DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS 0
#define DRISHTI_DLIB_DO_PCA_INTERNAL 1
#define DRISHTI_DLIB_DO_HALF 1
#define DRISHTI_DLIB_DO_NUMERIC_DEBUG 0

#include <opencv2/core/core.hpp>

// clang-format off
#if DRISHTI_DLIB_DO_DEBUG_ELLIPSE
#  include <opencv2/imgproc/imgproc.hpp>
#endif
// clang-format on

// clang-format off
#if DRISHTI_DLIB_DO_DEBUG_ELLIPSE || DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS
#  include <opencv2/highgui/highgui.hpp>
#endif
// clang-format on

#include <dlib/opencv.h>
#include <dlib/opencv/cv_image.h>
#include <dlib/geometry/vector.h>
#include <dlib/image_transforms/assign_image.h>
#include <dlib/image_processing/full_object_detection.h>

#include "Eigen/Eigen"

#include "drishti/ml/drishti_ml.h"
#include "drishti/ml/PCA.h"
#include "drishti/geometry/Ellipse.h"
#include "drishti/core/Logger.h"

// Check input preprocessor definitions for SIMD and FIXED_POINT behavior:
//
// DRISHTI_BUILD_REGRESSION_SIMD
// DRISHTI_BUILD_REGRESSION_FIXED_POINT

#define DRISHTI_BUILD_PARALLEL_BOOSTING 1

// clang-format off
#if DRISHTI_BUILD_PARALLEL_BOOSTING
#  include "drishti/core/Parallel.h"
#endif
// clang-format on

#define FIXED_PRECISION 10

// clang-format off
#if defined(ANDROID)
#  define HALF_ENABLE_CPP11_CMATH 0
#endif
// clang-format on

// clang-format off
#if DRISHTI_DLIB_DO_HALF
#  include "half/half.hpp"
#endif
// clang-format on

#include "drishti/core/arithmetic.h"

// OpenCV
#include <opencv2/core/core.hpp>

// STL
#include <deque>

DRISHTI_ML_NAMESPACE_BEGIN

using drishti::geometry::operator*;

// Utility conversion routines:
inline cv::Point2f cv_point(const dlib::point& p)
{
    return cv::Point2f(int(p.x()), int(p.y()));
}
inline cv::Rect cv_rect(const dlib::rectangle& r)
{
    return cv::Rect(cv_point(r.tl_corner()), cv_point(r.br_corner()));
}
inline dlib::rectangle dlib_rect(const cv::Rect& r)
{
    return dlib::rectangle(r.x, r.y, r.br().x, r.br().y);
}
inline dlib::point dlib_point(const cv::Point& p)
{
    return dlib::point(p.x, p.y);
}

typedef dlib::matrix<float, 0, 1> fshape;
typedef dlib::vector<float, 2> fpoint;
typedef std::vector<fpoint> PointVecf;
typedef std::vector<PointVecf> PointVecVecf;

static cv::RotatedRect vectorToEllipse(const std::vector<float>& phi)
{
    return drishti::geometry::phiToEllipse(phi);
}

using DVec32s = dlib::matrix<int32_t, 0, 1>;
using DVec16s = dlib::matrix<int16_t, 0, 1>;

using StandardizedPCAPtr = std::shared_ptr<StandardizedPCA>;

inline static void add16sAnd32s(const DVec32s& a, const DVec16s& b, DVec32s& c)
{
    if (!c.size())
    {
        c.set_size(b.size());
        for (int i = 0; i < b.size(); i++)
        {
            c(i) = b(i);
        }
    }
    else
    {

#if DRISHTI_BUILD_REGRESSION_SIMD
        drishti::core::add16sAnd32s(&a(0), &b(0), &c(0), int(b.size()));
#else
        for (int i = 0; i < b.size(); i++)
        {
            c(i) += int32_t(b(i));
        }
#endif
    }
}

inline static void add16sAnd16s(const DVec16s& a, const DVec16s& b, DVec16s& c)
{
    if (!c.size())
    {
        c += b;
    }
    else
    {
#if DRISHTI_BUILD_REGRESSION_SIMD
        drishti::core::add16sAnd16s(&a(0), &b(0), &c(0), int(b.size()));
#else
        c += b;
#endif
    }
}

inline static void add32F(const drishti::ml::fshape& a, const fshape& b, fshape& c)
{
    if (!c.size())
    {
        c += b;
    }
    else
    {
#if DRISHTI_BUILD_REGRESSION_SIMD
        drishti::core::add32f(&a(0), &b(0), &c(0), int(a.size()));
#else
        c += b;
#endif
    }
}

template <typename T>
T compute_npd(const T& a, const T& b)
{
    return (a - b) / (a + b + T(1e-6));
    //return (a - b)/(a + b);
    //return ((a + b) == T(0.0)) ? std::numeric_limits<float>::lowest() : (a - b) / (a + b);
}

// struct Ellipse { double xs, ys, ang, scl, asp; };

template <typename T1, typename T2>
void copy(std::vector<std::vector<T1>>& src, std::vector<std::vector<T2>>& dst)
{
    dst.resize(src.size());
    for (int i = 0; i < src.size(); i++)
    {
        dst[i].resize(src[i].size());
        std::copy(src[i].begin(), src[i].end(), dst[i].begin());
    }
}

struct InterpolatedFeature
{
    uint16_t f1;
    uint16_t f2;
    uint16_t f3;
    float w12;
    float w13;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& f1;
        ar& f2;
        ar& f3;
#if DRISHTI_DLIB_DO_HALF
        if (Archive::is_loading::value)
        {
            half_float::detail::uint16 value1, value2;
            ar& value1;
            ar& value2;
            w12 = half_float::detail::half2float(value1);
            w13 = half_float::detail::half2float(value2);
        }
        else
        {
            half_float::detail::uint16 value1, value2;
            value1 = half_float::detail::float2half<std::round_to_nearest>(w12);
            value2 = half_float::detail::float2half<std::round_to_nearest>(w13);
            ar& value1;
            ar& value2;
        }
#else
        ar& w12;
        ar& w13;
#endif
    }
};

DRISHTI_BEGIN_NAMESPACE(impl)

static fpoint interpolate_feature_point(const InterpolatedFeature& f, const fshape& shape)
{
    fpoint p1(shape(f.f1 * 2 + 0), shape(f.f1 * 2 + 1));
    fpoint p2(shape(f.f2 * 2 + 0), shape(f.f2 * 2 + 1));
    fpoint p3(shape(f.f3 * 2 + 0), shape(f.f3 * 2 + 1));
    return p1 + ((p2 - p1) * f.w12)  + ((p3 - p1) * f.w13);
}

struct recipe
{
    unsigned long _num_trees;
    // TBD
};

struct split_feature
{
    unsigned short idx1 = 0;
    unsigned short idx2 = 0;
    float thresh = 0.f;

    split_feature() {}
    split_feature(unsigned short idx1, unsigned short idx2, float thresh)
        : idx1(idx1)
        , idx2(idx2)
        , thresh(thresh)
    {
    }

    friend inline void serialize(const split_feature& item, std::ostream& out)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        dlib::serialize(item.idx1, out);
        dlib::serialize(item.idx2, out);
        dlib::serialize(item.thresh, out);
#endif // !DRISHTI_BUILD_MIN_SIZE
    }
    friend inline void deserialize(split_feature& item, std::istream& in)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        dlib::deserialize(item.idx1, in);
        dlib::deserialize(item.idx2, in);
        dlib::deserialize(item.thresh, in);
#endif // !DRISHTI_BUILD_MIN_SIZE
    }
};

// a tree is just a std::vector<impl::split_feature>.  We use this function to navigate the
// tree nodes
inline unsigned long left_child(unsigned long idx)
{
    return 2 * idx + 1;
}
/*!
    ensures
        - returns the index of the left child of the binary tree node idx
!*/
inline unsigned long right_child(unsigned long idx)
{
    return 2 * idx + 2;
}
/*!
    ensures
        - returns the index of the left child of the binary tree node idx
!*/

struct Fixed
{
};
struct regression_tree
{
    std::vector<split_feature> splits;
    std::vector<fshape> leaf_values;
    std::vector<DVec16s> leaf_values_16;

    inline const DVec16s& operator()(
        const std::vector<float>& feature_pixel_values,
        const Fixed& fixed,
        bool do_npd = false) const
    /*!
     requires
     - All the index values in splits are less than feature_pixel_values.size()
     - leaf_values.size() is a power of 2.
     (i.e. we require a tree with all the levels fully filled out.
     - leaf_values.size() == splits.size()+1
     (i.e. there needs to be the right number of leaves given the number of splits in the tree)
     ensures
     - runs through the tree and returns the vector at the leaf we end up in.
     !*/
    {
        if (do_npd)
        {
            unsigned long i = 0;
            while (i < splits.size())
            {
                auto& node = splits[i];
                if (compute_npd(feature_pixel_values[node.idx1], feature_pixel_values[node.idx2]) > node.thresh)
                {
                    i = left_child(i);
                }
                else
                {
                    i = right_child(i);
                }
            }
            return leaf_values_16[i - splits.size()];
        }
        else
        {
            unsigned long i = 0;
            while (i < splits.size())
            {
                auto& node = splits[i];
                if (feature_pixel_values[node.idx1] - feature_pixel_values[node.idx2] > node.thresh)
                {
                    i = left_child(i);
                }
                else
                {
                    i = right_child(i);
                }
            }
            return leaf_values_16[i - splits.size()];
        }
    }

    inline const fshape& operator()(
        const std::vector<float>& feature_pixel_values,
        bool do_npd = false) const
    /*!
        requires
            - All the index values in splits are less than feature_pixel_values.size()
            - leaf_values.size() is a power of 2.
              (i.e. we require a tree with all the levels fully filled out.
            - leaf_values.size() == splits.size()+1
              (i.e. there needs to be the right number of leaves given the number of splits in the tree)
        ensures
            - runs through the tree and returns the vector at the leaf we end up in.
    !*/
    {
        if (do_npd)
        {
            unsigned long i = 0;
            while (i < splits.size())
            {
                auto& node = splits[i];
                if (compute_npd(feature_pixel_values[node.idx1], feature_pixel_values[node.idx2]) > node.thresh)
                {
                    i = left_child(i);
                }
                else
                {
                    i = right_child(i);
                }
            }
            return leaf_values[i - splits.size()];
        }
        else
        {
            unsigned long i = 0;
            while (i < splits.size())
            {
                auto& node = splits[i];
                if (feature_pixel_values[node.idx1] - feature_pixel_values[node.idx2] > node.thresh)
                {
                    i = left_child(i);
                }
                else
                {
                    i = right_child(i);
                }
            }
            return leaf_values[i - splits.size()];
        }
    }

    friend void serialize(const regression_tree& item, std::ostream& out)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        dlib::serialize(item.splits, out);
        dlib::serialize(item.leaf_values, out);
#endif // !DRISHTI_BUILD_MIN_SIZE
    }
    friend void deserialize(regression_tree& item, std::istream& in)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        dlib::deserialize(item.splits, in);
        dlib::deserialize(item.leaf_values, in);
#endif // !DRISHTI_BUILD_MIN_SIZE
    }
};

// ------------------------------------------------------------------------------------

inline dlib::vector<float, 2> location(
    const fshape& shape,
    unsigned long idx)
/*!
    requires
        - idx < shape.size()/2
        - shape.size()%2 == 0
    ensures
        - returns the idx-th point from the shape vector.
!*/
{
    return dlib::vector<float, 2>(shape(idx * 2), shape(idx * 2 + 1));
}

// ------------------------------------------------------------------------------------

inline unsigned long nearest_shape_point(
    const fshape& shape,
    const dlib::vector<float, 2>& pt,
    int ellipse_count = 0)
{
    // find the nearest part of the shape to this pixel
    float best_dist = std::numeric_limits<float>::infinity();
    const unsigned long num_shape_parts = (shape.size() - (ellipse_count * 5)) / 2;
    unsigned long best_idx = 0;
    for (unsigned long j = 0; j < num_shape_parts; ++j)
    {
        const float dist = length_squared(location(shape, j) - pt);
        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = j;
        }
    }
    return best_idx;
}

// ------------------------------------------------------------------------------------

inline void create_shape_relative_encoding(
    const fshape& shape,
    const PointVecf& pixel_coordinates,
    std::vector<unsigned short>& anchor_idx,
    PointVecf& deltas,
    int ellipse_count = 0)
/*!
    requires
        - shape.size()%2 == 0
        - shape.size() > 0
    ensures
        - #anchor_idx.size() == pixel_coordinates.size()
        - #deltas.size()     == pixel_coordinates.size()
        - for all valid i:
            - pixel_coordinates[i] == location(shape,#anchor_idx[i]) + #deltas[i]
!*/
{
    anchor_idx.resize(pixel_coordinates.size());
    deltas.resize(pixel_coordinates.size());

    for (unsigned long i = 0; i < pixel_coordinates.size(); ++i)
    {
        anchor_idx[i] = nearest_shape_point(shape, pixel_coordinates[i], ellipse_count);
        deltas[i] = pixel_coordinates[i] - location(shape, anchor_idx[i]);
    }
}

// ------------------------------------------------------------------------------------

inline dlib::point_transform_affine find_tform_between_shapes(
    const fshape& from_shape,
    const fshape& to_shape,
    int ellipse_count,
    bool do_affine = false)
{
    DLIB_ASSERT(from_shape.size() == to_shape.size() && ((from_shape.size() - (ellipse_count * 5)) % 2) == 0 && from_shape.size() > 0, "");
    PointVecf from_points, to_points;
    const unsigned long num = (from_shape.size() - (ellipse_count * 5)) / 2;
    from_points.reserve(num);
    to_points.reserve(num);
    if (num == 1)
    {
        // Just use an identity transform if there is only one landmark.
        return dlib::point_transform_affine();
    }

    for (unsigned long i = 0; i < num; ++i)
    {
        from_points.push_back(location(from_shape, i));
        to_points.push_back(location(to_shape, i));
    }
    return do_affine ? find_affine_transform(from_points, to_points) : find_similarity_transform(from_points, to_points);
}

// ------------------------------------------------------------------------------------

inline dlib::point_transform_affine normalizing_tform(
    const dlib::rectangle& rect)
/*!
    ensures
        - returns a transform that maps rect.tl_corner() to (0,0) and rect.br_corner()
          to (1,1).
!*/
{
    PointVecf from_points{ rect.tl_corner(), rect.tr_corner(), rect.br_corner() };
    PointVecf to_points{ { 0, 0 }, { 1, 0 }, { 1, 1 } };
    return find_affine_transform(from_points, to_points);
}

// ------------------------------------------------------------------------------------

inline dlib::point_transform_affine unnormalizing_tform(
    const dlib::rectangle& rect)
/*!
    ensures
        - returns a transform that maps (0,0) to rect.tl_corner() and (1,1) to
          rect.br_corner().
!*/
{
    PointVecf to_points{ rect.tl_corner(), rect.tr_corner(), rect.br_corner() };
    PointVecf from_points{ { 0, 0 }, { 1, 0 }, { 1, 1 } };
    return find_affine_transform(from_points, to_points);
}

// ------------------------------------------------------------------------------------

template <typename image_type>
void extract_feature_pixel_values(
    const image_type& img_,
    const dlib::rectangle& rect,
    const fshape& current_shape,
    const std::vector<InterpolatedFeature>& interpolated_features,
    std::vector<float>& feature_pixel_values)
{
    const dlib::point_transform_affine tform_to_img = unnormalizing_tform(rect);
    const dlib::rectangle area = get_rect(img_);
    dlib::const_image_view<image_type> img(img_);
    feature_pixel_values.resize(interpolated_features.size());
    for (unsigned long i = 0; i < feature_pixel_values.size(); ++i)
    {
        auto p = interpolate_feature_point(interpolated_features[i], current_shape);
        dlib::point q = tform_to_img(p);
        if (area.contains(q))
        {
            feature_pixel_values[i] = dlib::get_pixel_intensity(img[q.y()][q.x()]);
        }
        else
        {
            feature_pixel_values[i] = 0;
        }
    }
}

template <typename image_type>
void extract_feature_pixel_values(
    const image_type& img_,
    const dlib::rectangle& rect,
    const fshape& current_shape,
    const fshape& reference_shape,
    const std::vector<unsigned short>& reference_pixel_anchor_idx,
    const PointVecf& reference_pixel_deltas,
    std::vector<float>& feature_pixel_values,
    int ellipse_count = 0,
    bool do_affine = false)
/*!
    requires
        - image_type == an image object that implements the interface defined in
          dlib/image_processing/generic_image.h
        - reference_pixel_anchor_idx.size() == reference_pixel_deltas.size()
        - current_shape.size() == reference_shape.size()
        - reference_shape.size()%2 == 0
        - max(mat(reference_pixel_anchor_idx)) < reference_shape.size()/2
    ensures
        - #feature_pixel_values.size() == reference_pixel_deltas.size()
        - for all valid i:
            - #feature_pixel_values[i] == the value of the pixel in img_ that
              corresponds to the pixel identified by reference_pixel_anchor_idx[i]
              and reference_pixel_deltas[i] when the pixel is located relative to
              current_shape rather than reference_shape.
!*/
{
    const dlib::matrix<float, 2, 2> tform = dlib::matrix_cast<float>(find_tform_between_shapes(reference_shape, current_shape, ellipse_count, do_affine).get_m());
    const dlib::point_transform_affine tform_to_img = unnormalizing_tform(rect);

    const dlib::rectangle area = get_rect(img_);

    dlib::const_image_view<image_type> img(img_);
    feature_pixel_values.resize(reference_pixel_deltas.size());

#if DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS
    cv::Mat canvas;
    cv::cvtColor(dlib::toMat(const_cast<image_type&>(img_)), canvas, cv::COLOR_GRAY2BGR);
//cv::Mat canvas(area.height(), area.width(), CV_8UC3, cv::Scalar::all(0));
#endif

    for (unsigned long i = 0; i < feature_pixel_values.size(); ++i)
    {
        // Compute the point in the current shape corresponding to the i-th pixel and
        // then map it from the normalized shape space into pixel space.
        // point p = tform_to_img(tform*reference_pixel_deltas[i] + location(current_shape, reference_pixel_anchor_idx[i]));

        auto p0 = location(current_shape, reference_pixel_anchor_idx[i]);
        auto p1 = tform * reference_pixel_deltas[i];
        dlib::point p = tform_to_img(p0 + p1);
        if (area.contains(p))
        {
            feature_pixel_values[i] = dlib::get_pixel_intensity(img[p.y()][p.x()]);
#if DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS
            cv::circle(canvas, cv::Point(p.x(), p.y()), 2, { 0, 255, 255 }, -1, 8);
#endif
        }
        else
        {
            feature_pixel_values[i] = 0; // original behaviour (fastest)
        }
    }

#if DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS
    for (int i = 0; i < current_shape.size() / 2; i++)
    {
        auto q = tform_to_img(location(current_shape, i));
        cv::circle(canvas, cv::Point(q.x(), q.y()), 2, { 0, 255, 0 }, -1, 8);
    }
    cv::imshow("features", canvas), cv::waitKey(0); // opt
#endif
}

DRISHTI_END_NAMESPACE(impl) // end namespace impl

// ----------------------------------------------------------------------------------------

// ellipse is converted back to opencv format, where each parameter of the ellipse
// is stored the x-coordinate of a new point

template <typename T>
std::vector<dlib::vector<T, 2>> convert_shape_to_points(const fshape& shape, int ellipse_count)
{
    int point_length = int((shape.size() - (ellipse_count * 5)) / 2);
    std::vector<dlib::vector<T, 2>> points(point_length + (ellipse_count * 5));

    for (unsigned long i = 0; i < point_length; ++i)
    {
        points[i] = impl::location(shape, i);
    }

    // Convert trailing ellipse back to standard form:
    for (int i = 0; i < ellipse_count; i++)
    {
        const float* ptr = &shape(point_length * 2 + (i * 5));
        std::vector<float> phi(ptr, ptr + 5);
        cv::RotatedRect e = geometry::phiToEllipse(phi);
        std::vector<float> data = geometry::ellipseToVector(e);
        for (int j = 0; j < 5; j++)
        {
            points[point_length + (i * 5) + j] = dlib::vector<T, 2>(data[j], 0);
        }
    }

    return points;
}

class shape_predictor
{
public:
    shape_predictor() = default;
    ~shape_predictor() = default;

    shape_predictor(
        const fshape& initial_shape_,
        const std::vector<std::vector<impl::regression_tree>>& forests_,
        const std::vector<std::vector<InterpolatedFeature>>& interpolated_features,
        StandardizedPCAPtr& pca,
        bool npd = false,
        bool do_affine = false,
        int ellipse_count = 0)
        : initial_shape(initial_shape_)
        , forests(forests_)
        , m_pca(pca)
        , m_ellipse_count(ellipse_count)
        , m_npd(npd)
        , m_do_affine(do_affine)
        , interpolated_features(interpolated_features)
    /*!
         requires
         - initial_shape.size()%2 == 0
         - forests.size() == pixel_coordinates.size() == the number of cascades
         - for all valid i:
         - all the index values in forests[i] are less than pixel_coordinates[i].size()
         - for all valid i and j:
         - forests[i][j].leaf_values.size() is a power of 2.
         (i.e. we require a tree with all the levels fully filled out.
         - forests[i][j].leaf_values.size() == forests[i][j].splits.size()+1
         (i.e. there need to be the right number of leaves given the number of splits in the tree)
         !*/
    {
    }

    void getShapeUpdates(std::vector<float>& values, bool pca)
    {
        const auto& eT = m_pca->getTransposedEigenvectors();
        for (/*const*/ auto& f : forests)
        {
            std::cout << f.front().leaf_values.size() << std::endl;
            for (/*const*/ auto& g : f)
            {
                for (/*const*/ auto& s : g.leaf_values)
                {
                    if (m_pca && !pca) // if using pca and we want full points:
                    {
                        fshape shape_full;
                        shape_full = fshape(eT.rows);
                        back_project(*m_pca, s.size(), s, shape_full);
                        for (const auto& r : shape_full)
                        {
                            values.push_back(r);
                        }
                    }
                    else
                    {
                        for (const auto& r : s)
                        {
                            values.push_back(r);
                        }
                    }
                }
            }
        }
    }

    void populate_f16()
    {
        for (auto& f : forests)
        {
            for (auto& g : f)
            {
                g.leaf_values_16.resize(g.leaf_values.size());
                for (int i = 0; i < g.leaf_values.size(); i++)
                {
                    auto& leaf = g.leaf_values[i];
                    auto& leaf16 = g.leaf_values_16[i];
                    leaf16.set_size(leaf.size());
                    drishti::core::convertFixedPoint(&leaf(0, 0), &leaf16(0, 0), int(leaf.size()), FIXED_PRECISION);
                }
            }
        }
    }

    shape_predictor(
        const fshape& initial_shape_,
        const std::vector<std::vector<impl::regression_tree>>& forests_,
        const std::vector<PointVecf>& pixel_coordinates,
        StandardizedPCAPtr& pca,
        bool npd = false,
        bool do_affine = false,
        int ellipse_count = 0)
        : initial_shape(initial_shape_)
        , forests(forests_)
        , m_pca(pca)
        , m_ellipse_count(ellipse_count)
        , m_npd(npd)
        , m_do_affine(do_affine)
    /*!
        requires
            - initial_shape.size()%2 == 0
            - forests.size() == pixel_coordinates.size() == the number of cascades
            - for all valid i:
                - all the index values in forests[i] are less than pixel_coordinates[i].size()
            - for all valid i and j:
                - forests[i][j].leaf_values.size() is a power of 2.
                  (i.e. we require a tree with all the levels fully filled out.
                - forests[i][j].leaf_values.size() == forests[i][j].splits.size()+1
                  (i.e. there need to be the right number of leaves given the number of splits in the tree)
        !*/
    {
        anchor_idx.resize(pixel_coordinates.size());
        deltas.resize(pixel_coordinates.size());
        // Each cascade uses a different set of pixels for its features.  We compute
        // their representations relative to the initial shape now and save it.
        for (unsigned long i = 0; i < pixel_coordinates.size(); ++i)
        {
            impl::create_shape_relative_encoding(initial_shape, pixel_coordinates[i], anchor_idx[i], deltas[i]);
        }

        m_num_workers = 4; //cv::getNumberOfCPUs();
    }

    unsigned long num_parts() const
    {
        return initial_shape.size() / 2;
    }

    static void project(drishti::ml::StandardizedPCA& pca, fshape& src, fshape& dst)
    {
        cv::Mat1f projection = pca.project(cv::Mat1f(1, src.size(), &src(0)));
        dst.set_size(projection.cols, 1);
        memcpy(&dst(0), projection.ptr<float>(0), sizeof(float) * projection.cols);
    }

    static void back_project(drishti::ml::StandardizedPCA& pca, int n, fshape& src, fshape& dst)
    {
        cv::Mat1f back_projection = pca.backProject(cv::Mat1f(1, n, &src(0)));
        memcpy(&dst(0), back_projection.ptr<float>(), sizeof(float) * back_projection.cols);
    }

    template <typename image_type>
    dlib::full_object_detection operator()(
        const image_type& img,
        const dlib::rectangle& rect,
        fshape starter_shape,
        int stages = std::numeric_limits<int>::max()) const // early temrination
    {
        using namespace impl;

        bool do_pca = m_pca ? true : false;

        cv::Mat1f cs;
        fshape current_shape = starter_shape, current_shape_full_; // for PCA mode

        if (do_pca)
        {
            project(*m_pca, starter_shape, current_shape_full_);
        }

        std::vector<float> feature_pixel_values;
        size_t forestCount = std::min(int(forests.size()), stages);
        for (unsigned long iter = 0; iter < forestCount; ++iter)
        {
            auto& cs_ = current_shape;
            auto& is_ = initial_shape; // this is used to map pose indexed features to current shape

            // Previously had for loop
            if (do_pca)
            {
                // Get euclidean model for current shape space estimate:
                int current_pca_dim = int(forests[iter][0].leaf_values[0].size());
                back_project(*m_pca, current_pca_dim, current_shape_full_, cs_);
            }

            if (interpolated_features.size())
            {
                extract_feature_pixel_values(img, rect, cs_, interpolated_features[iter], feature_pixel_values);
            }
            else
            {
                extract_feature_pixel_values(img, rect, cs_, is_, anchor_idx[iter], deltas[iter], feature_pixel_values, m_ellipse_count, m_do_affine);
            }

            fshape current_shape_;
            auto& active_shape = do_pca ? current_shape_ : current_shape;

#if DRISHTI_BUILD_REGRESSION_FIXED_POINT
            // Fixed point is currently only working for PCA in most cases (check numerical overflow)
            DVec16s shape_accumulator;
#if DRISHTI_BUILD_PARALLEL_BOOSTING
            {
                const unsigned long num = forests[iter].size();
                const unsigned long block_size = std::max(1UL, (num + m_num_workers - 1) / m_num_workers);
                std::vector<fshape> block_sums(m_num_workers);
                std::vector<DVec16s> shape_accumulators(m_num_workers);
                drishti::core::ParallelHomogeneousLambda harness = [&](int block) {
                    const unsigned long block_begin = block * block_size;
                    const unsigned long block_end = std::min(num, block_begin + block_size);
                    for (unsigned long i = block_begin; i < block_end; ++i)
                    {
                        auto& f = forests[iter][i];
                        add16sAnd16s(shape_accumulators[block], f(feature_pixel_values, Fixed(), m_npd), shape_accumulators[block]);
                    }
                };

                //harness({0,static_cast<int>(num_workers)});
                cv::parallel_for_({ 0, static_cast<int>(m_num_workers) }, harness);

                for (auto& s : shape_accumulators)
                {
                    add16sAnd16s(shape_accumulator, s, shape_accumulator);
                }
            }
#else
            for (auto& f : forests[iter])
            {
                add16sAnd16s(shape_accumulator, f(feature_pixel_values, Fixed(), m_npd), shape_accumulator);
            }
#endif

            // fixed -> float
            active_shape.set_size(shape_accumulator.size());
            for (int i = 0; i < shape_accumulator.size(); i++)
            {
                active_shape(i) = float(shape_accumulator(i)) / float(1 << FIXED_PRECISION);
            }

#else  /* else don't DRISHTI_BUILD_REGRESSION_FIXED_POINT */
            for (auto& f : forests[iter])
            {
                add32F(active_shape, f(feature_pixel_values, m_npd), active_shape);
            }
#endif /* DRISHTI_BUILD_REGRESSION_FIXED_POINT */

            if (do_pca)
            {
                dlib::set_rowm(current_shape_full_, dlib::range(0, current_shape_.size() - 1)) += current_shape_;
            }
        }

        if (do_pca)
        {
            // Convert the final model back to euclidean
            int current_pca_dim = int(forests.back()[0].leaf_values[0].size());
            back_project(*m_pca, current_pca_dim, current_shape_full_, current_shape);
        }

        // convert the current_shape into a full_object_detection
        const dlib::point_transform_affine tform_to_img = unnormalizing_tform(rect);

        int point_length = int((current_shape.size() - (m_ellipse_count * 5)) / 2);
        std::vector<dlib::point> parts(point_length + (m_ellipse_count * 5));
        for (unsigned long i = 0; i < point_length; ++i)
        {
            parts[i] = tform_to_img(location(current_shape, i));
        }

        // Convert trailing ellipse back to standard form:
        for (int i = 0; i < m_ellipse_count; i++)
        {
            std::vector<float> phi(5, 0.f);
            for (int j = 0; j < 5; j++)
            {
                phi[j] = current_shape(point_length * 2 + (i * 5) + j);
            }

            const auto& m = tform_to_img.get_m();
            const auto& b = tform_to_img.get_b();
            cv::Matx33f H(m(0, 0), m(0, 1), b(0), m(1, 0), m(1, 1), b(1), 0, 0, 1);
            cv::RotatedRect e = vectorToEllipse(phi);
            cv::RotatedRect e2 = H * e;

            int end = point_length + (i * 5);
            parts[end + 0] = dlib::point(e2.center.x, 0.f);
            parts[end + 1] = dlib::point(e2.center.y, 0.f);
            parts[end + 2] = dlib::point(e2.size.width, 0.f);
            parts[end + 3] = dlib::point(e2.size.height, 0.f);
            parts[end + 4] = dlib::point(e2.angle, 0.f);
        }

#if DRISHTI_DLIB_DO_DEBUG_ELLIPSE
        {
            //  Convert image to opencv, then draw ellipse and shape:
            cv::Mat image = dlib::toMat(const_cast<image_type&>(img));
            cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            for (int j = 0; j < point_length; j++)
            {
                cv::circle(image, cv::Point(parts[j].x(), parts[j].y()), 2, { 0, 255, 0 }, 1, 8);
            }

            for (int i = 0; i < ellipse_count; i++)
            {
                cv::RotatedRect e1;
                e1.center.x = parts[point_length + 0].x();
                e1.center.y = parts[point_length + 1].x();
                e1.size.width = parts[point_length + 2].x();
                e1.size.height = parts[point_length + 3].x();
                e1.angle = parts[point_length + 4].x();

                cv::ellipse(image, e1, { 0, 255, 0 }, 1, 8);
            }
            cv::imshow("image", image), cv::waitKey(0);
        }
#endif
        return dlib::full_object_detection(rect, parts);
    }

    template <typename image_type>
    dlib::full_object_detection operator()(
        const image_type& img,
        const dlib::rectangle& rect) const
    {
        fshape current_shape = initial_shape;
        return (*this)(img, rect, current_shape);
    }

    friend void serialize(const shape_predictor& item, std::ostream& out)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        int version = 1;
        dlib::serialize(version, out);
        dlib::serialize(item.initial_shape, out);
        dlib::serialize(item.forests, out);
        dlib::serialize(item.anchor_idx, out);
        dlib::serialize(item.deltas, out);
#endif // !DRISHTI_BUILD_MIN_SIZE
    }
    friend void deserialize(shape_predictor& item, std::istream& in)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        int version = 0;
        dlib::deserialize(version, in);
        if (version != 1)
        {
            throw dlib::serialization_error("Unexpected version found while deserializing dlib::shape_predictor.");
        }
        dlib::deserialize(item.initial_shape, in);
        dlib::deserialize(item.forests, in);
        dlib::deserialize(item.anchor_idx, in);
        dlib::deserialize(item.deltas, in);
#endif // !DRISHTI_BUILD_MIN_SIZE
    }

    void setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
    {
        m_streamLogger = logger;
    }

    fshape initial_shape;
    std::vector<std::vector<impl::regression_tree>> forests;

    // Pose indexing relative to nearest landmark points:
    std::vector<std::vector<unsigned short>> anchor_idx;
    std::vector<PointVecf> deltas;

    // PCA reduction:
    std::shared_ptr<drishti::ml::StandardizedPCA> m_pca; // global pca
    int m_ellipse_count = 0;
    bool m_npd = false;
    bool m_do_affine = false;
    unsigned long m_num_workers = 1;

    // Use interpolated "line indexed" features (stead of the relative encoding above):
    std::vector<std::vector<InterpolatedFeature>> interpolated_features;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

// ----------------------------------------------------------------------------------------

void serialize(const drishti::ml::shape_predictor& item, std::ostream& out);
void deserialize(drishti::ml::shape_predictor& item, std::istream& in);

DRISHTI_ML_NAMESPACE_END

typedef drishti::ml::impl::regression_tree RTType;
typedef dlib::vector<float, 2> Vec2Type;

#if DRISHTI_DLIB_DO_HALF
struct PointHalf
{
    PointHalf() {}

    PointHalf(const Vec2Type& src)
        : x(half_float::detail::float2half<std::round_to_nearest>(src(0)))
        , y(half_float::detail::float2half<std::round_to_nearest>(src(1)))
    {
    }

    operator Vec2Type() const
    {
        Vec2Type tmp;
        tmp(0) = asFloat(x);
        tmp(1) = asFloat(y);
        return tmp;
    }

    static float asFloat(const half_float::detail::uint16& src)
    {
        return half_float::detail::half2float(src);
    }

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& x;
        ar& y;
    }

    half_float::detail::uint16 x, y;
};
#endif

#define _SHAPE_PREDICTOR drishti::ml::shape_predictor

#endif // __drishti_ml_shape_predictor_h__
