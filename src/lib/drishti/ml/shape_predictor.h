// Copyright (C) 2014  Davis E. King (davis@dlib.net)
// Copyright (C) 2015-2016  David Hirvonen
// License: Boost Software License   See LICENSE.txt for the full license.
#ifndef __drishti_ml_shape_predictor_h__
#define __drishti_ml_shape_predictor_h__

#define DRISHTI_DLIB_DO_DEBUG_ELLIPSE 0
#define DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS 0
#define DRISHTI_DLIB_DO_PCA_INTERNAL 1
#define DRISHTI_DLIB_DO_HALF 1
#define DRISHTI_DLIB_DO_NUMERIC_DEBUG 0
#define DRISHTI_DLIB_DO_SQUARE_PIXELS 1

#include <opencv2/core/core.hpp>

#if DRISHTI_DLIB_DO_DEBUG_ELLIPSE
#  include <opencv2/imgproc/imgproc.hpp>
#endif

#if DRISHTI_DLIB_DO_DEBUG_ELLIPSE || DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <dlib/opencv.h>
#include <dlib/opencv/cv_image.h>
#include <dlib/geometry/vector.h>
#include <dlib/image_transforms/assign_image.h>
#include <dlib/image_processing/full_object_detection.h>

#if !DRISHTI_BUILD_MIN_SIZE
#  include <dlib/serialize.h>
#  include <dlib/console_progress_indicator.h>
#endif

#include "Eigen/Eigen"

#include "drishti/ml/drishti_ml.h"
#include "drishti/ml/PCA.h"
#include "drishti/geometry/Ellipse.h"
#include "drishti/core/Parallel.h"
#include "drishti/core/Logger.h"

// Check input preprocessor definitions for SIMD and FIXED_POINT behavior:
//
// DRISHTI_BUILD_REGRESSION_SIMD
// DRISHTI_BUILD_REGRESSION_FIXED_POINT

#define FIXED_PRECISION 10

#if defined(ANDROID)
#  define HALF_ENABLE_CPP11_CMATH 0
#endif

#if DRISHTI_DLIB_DO_HALF
#  include "half/half.hpp"
#endif

#include "drishti/core/arithmetic.h"


#if DRISHTI_SERIALIZE_WITH_BOOST        
#  include "drishti/core/boost_serialize_common.h"
#endif

// OpenCV
#include <opencv2/core/core.hpp>

// STL
#include <deque>

DRISHTI_ML_NAMESPACE_BEGIN

using drishti::geometry::operator *;

// Utility conversion routines:
inline cv::Point2f cv_point(const dlib::point &p)
{
    return cv::Point2f(int(p.x()), int(p.y()));
}
inline cv::Rect cv_rect(const dlib::rectangle &r)
{
    return cv::Rect(cv_point(r.tl_corner()), cv_point(r.br_corner()));
}
inline dlib::rectangle dlib_rect(const cv::Rect &r)
{
    return dlib::rectangle(r.x, r.y, r.br().x, r.br().y);
}
inline dlib::point dlib_point(const cv::Point &p)
{
    return dlib::point(p.x, p.y);
}

typedef dlib::matrix<float,0,1> fshape;
typedef dlib::vector<float,2> fpoint;
typedef std::vector<fpoint> PointVecf;
typedef std::vector<PointVecf> PointVecVecf;

static cv::RotatedRect vectorToEllipse(const std::vector<float> &phi)
{
    return drishti::geometry::phiToEllipse(phi, false);
}

inline static
void add16sAnd32s(const dlib::matrix<int32_t,0,1> &a, const dlib::matrix<int16_t,0,1> &b, dlib::matrix<int32_t,0,1> &c)
{
    if(!c.size())
    {
        c.set_size(b.size());
        for(int i = 0; i < b.size(); i++)
        {
            c(i) = b(i);
        }
    }
    else
    {

#if DRISHTI_BUILD_REGRESSION_SIMD
        drishti::core::add16sAnd32s(&a(0), &b(0), &c(0), int(b.size()));
#else
        for(int i = 0; i < b.size(); i++)
        {
            c(i) += int32_t(b(i));
        }
#endif
    }
}

inline static
void add16sAnd16s(const dlib::matrix<int16_t,0,1> &a, const dlib::matrix<int16_t,0,1> &b, dlib::matrix<int16_t,0,1> &c)
{
    if(!c.size())
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

inline static
void add32F(const drishti::ml::fshape &a, const fshape &b, fshape &c)
{
    if(!c.size())
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

template <typename T> T compute_npd(const T &a, const T &b)
{
    //return (a - b)/(a + b);
    return ((a + b) == T(0.0)) ? std::numeric_limits<float>::lowest() : (a - b)/(a + b);
}

// struct Ellipse { double xs, ys, ang, scl, asp; };

template <typename T1, typename T2> void copy(std::vector<std::vector<T1>> &src, std::vector<std::vector<T2>> &dst)
{
    dst.resize(src.size());
    for(int i = 0; i < src.size(); i++)
    {
        dst[i].resize(src[i].size());
        std::copy(src[i].begin(), src[i].end(), dst[i].begin());
    }
}

struct InterpolatedFeature
{
    uint16_t f1;
    uint16_t f2;
    float alpha;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & f1;
        ar & f2;
#if DRISHTI_DLIB_DO_HALF
        if(Archive::is_loading::value)
        {
            half_float::detail::uint16 value;
            ar & value;
            alpha = half_float::detail::half2float(value);
        }
        else
        {
            half_float::detail::uint16 value;
            value = half_float::detail::float2half<std::round_to_nearest>(alpha);
            ar & value;
        }
#else
        ar & alpha;
#endif
    }
};


DRISHTI_BEGIN_NAMESPACE(impl)

static fpoint interpolate_feature_point(const InterpolatedFeature &f, const fshape &shape)
{
    fpoint p1(shape(f.f1*2+0), shape(f.f1*2+1));
    fpoint p2(shape(f.f2*2+0), shape(f.f2*2+1));
    fpoint v = p2 - p1;
    fpoint p = p1 + (v * f.alpha);
    return p;
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
    {}

    friend inline void serialize (const split_feature& item, std::ostream& out)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        dlib::serialize(item.idx1, out);
        dlib::serialize(item.idx2, out);
        dlib::serialize(item.thresh, out);
#endif // !DRISHTI_BUILD_MIN_SIZE
    }
    friend inline void deserialize (split_feature& item, std::istream& in)
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
inline unsigned long left_child (unsigned long idx)
{
    return 2 * idx + 1;
}
/*!
    ensures
        - returns the index of the left child of the binary tree node idx
!*/
inline unsigned long right_child (unsigned long idx)
{
    return 2 * idx + 2;
}
/*!
    ensures
        - returns the index of the left child of the binary tree node idx
!*/

struct Fixed {};
struct regression_tree
{
    std::vector<split_feature> splits;
    std::vector<fshape > leaf_values;
    std::vector<dlib::matrix<int16_t,0,1>> leaf_values_16;

    inline const dlib::matrix<int16_t,0,1>& operator()(
        const std::vector<float>& feature_pixel_values,
        const Fixed &fixed,
        bool do_npd = false
    ) const
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
        if(do_npd)
        {
            unsigned long i = 0;
            while (i < splits.size())
            {
                auto &node = splits[i];
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
                auto &node = splits[i];
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
        bool do_npd = false
    ) const
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
        if(do_npd)
        {
            unsigned long i = 0;
            while (i < splits.size())
            {
                auto &node = splits[i];
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
                auto &node = splits[i];
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

    friend void serialize (const regression_tree& item, std::ostream& out)
    {
#if !DRISHTI_BUILD_MIN_SIZE        
        dlib::serialize(item.splits, out);
        dlib::serialize(item.leaf_values, out);
#endif // !DRISHTI_BUILD_MIN_SIZE
    }
    friend void deserialize (regression_tree& item, std::istream& in)
    {
#if !DRISHTI_BUILD_MIN_SIZE        
        dlib::deserialize(item.splits, in);
        dlib::deserialize(item.leaf_values, in);
#endif // !DRISHTI_BUILD_MIN_SIZE
    }
};

// ------------------------------------------------------------------------------------

inline dlib::vector<float,2> location (
    const fshape& shape,
    unsigned long idx
)
/*!
    requires
        - idx < shape.size()/2
        - shape.size()%2 == 0
    ensures
        - returns the idx-th point from the shape vector.
!*/
{
    return dlib::vector<float,2>(shape(idx*2), shape(idx*2+1));
}

// ------------------------------------------------------------------------------------

inline unsigned long nearest_shape_point (
    const fshape& shape,
    const dlib::vector<float,2>& pt,
    int ellipse_count = 0
)
{
    // find the nearest part of the shape to this pixel
    float best_dist = std::numeric_limits<float>::infinity();
    const unsigned long num_shape_parts = (shape.size() - (ellipse_count * 5))/2;
    unsigned long best_idx = 0;
    for (unsigned long j = 0; j < num_shape_parts; ++j)
    {
        const float dist = length_squared(location(shape,j)-pt);
        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = j;
        }
    }
    return best_idx;
}

// ------------------------------------------------------------------------------------

inline void create_shape_relative_encoding (
    const fshape& shape,
    const PointVecf& pixel_coordinates,
    std::vector<unsigned short>& anchor_idx,
    PointVecf& deltas,
    int ellipse_count = 0
)
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
        deltas[i] = pixel_coordinates[i] - location(shape,anchor_idx[i]);
    }
}

// ------------------------------------------------------------------------------------

inline dlib::point_transform_affine find_tform_between_shapes (
    const fshape& from_shape,
    const fshape& to_shape,
    int ellipse_count,
    bool do_affine = false
)
{
    DLIB_ASSERT(from_shape.size() == to_shape.size() && ((from_shape.size()-(ellipse_count*5))%2) == 0 && from_shape.size() > 0,"");
    PointVecf from_points, to_points;
    const unsigned long num = (from_shape.size()-(ellipse_count*5))/2;
    from_points.reserve(num);
    to_points.reserve(num);
    if (num == 1)
    {
        // Just use an identity transform if there is only one landmark.
        return dlib::point_transform_affine();
    }

    for (unsigned long i = 0; i < num; ++i)
    {
        from_points.push_back(location(from_shape,i));
        to_points.push_back(location(to_shape,i));
    }
    return do_affine? find_affine_transform(from_points, to_points) : find_similarity_transform(from_points, to_points);
}

// ------------------------------------------------------------------------------------

inline dlib::point_transform_affine normalizing_tform (
    const dlib::rectangle& rect
)
/*!
    ensures
        - returns a transform that maps rect.tl_corner() to (0,0) and rect.br_corner()
          to (1,1).
!*/
{
#if DRISHTI_DLIB_DO_SQUARE_PIXELS
    const float scale = (1.0 / float(rect.right()));
    dlib::matrix<double,2,2> m;
    m = scale,0,0,scale;
    const dlib::vector<double,2> b(0,0);
    return dlib::point_transform_affine(m,b);
#else
    PointVecf from_points { rect.tl_corner(), rect.tr_corner(), rect.br_corner() };
    PointVecf to_points   { {0,0}, {1,0}, {1,1} };
    return find_affine_transform(from_points, to_points);
#endif
}

// ------------------------------------------------------------------------------------

inline dlib::point_transform_affine unnormalizing_tform (
    const dlib::rectangle& rect
)
/*!
    ensures
        - returns a transform that maps (0,0) to rect.tl_corner() and (1,1) to
          rect.br_corner().
!*/
{
#if DRISHTI_DLIB_DO_SQUARE_PIXELS
    const float scale = float(rect.right());
    dlib::matrix<double,2,2> m;
    m = scale,0,0,scale;
    const dlib::vector<double,2> b(0,0);
    return dlib::point_transform_affine(m,b);
#else
    PointVecf to_points { rect.tl_corner(), rect.tr_corner(), rect.br_corner() };
    PointVecf from_points { {0,0}, {1,0}, {1,1} };
    return find_affine_transform(from_points, to_points);
#endif
}

// ------------------------------------------------------------------------------------

template <typename image_type>
void extract_feature_pixel_values(
    const image_type &img_,
    const dlib::rectangle &rect,
    const fshape& current_shape,
    const std::vector<InterpolatedFeature> &interpolated_features,
    std::vector<float >& feature_pixel_values
)
{
    const dlib::point_transform_affine tform_to_img = unnormalizing_tform(rect);
    const dlib::rectangle area = get_rect(img_);
    dlib::const_image_view<image_type> img(img_);
    feature_pixel_values.resize(interpolated_features.size());
    for (unsigned long i = 0; i < feature_pixel_values.size(); ++i)
    {
        auto p = interpolate_feature_point(interpolated_features[i], current_shape);
        dlib::point q = tform_to_img(p);
        if(area.contains(q))
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
void extract_feature_pixel_values (
    const image_type& img_,
    const dlib::rectangle& rect,
    const fshape& current_shape,
    const fshape& reference_shape,
    const std::vector<unsigned short>& reference_pixel_anchor_idx,
    const PointVecf& reference_pixel_deltas,
    std::vector<float>& feature_pixel_values,
    int ellipse_count = 0,
    bool do_affine = false
)
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
    const dlib::matrix<float,2,2> tform = dlib::matrix_cast<float>(find_tform_between_shapes(reference_shape, current_shape, ellipse_count, do_affine).get_m());
    const dlib::point_transform_affine tform_to_img = unnormalizing_tform(rect);

    const dlib::rectangle area = get_rect(img_);

    dlib::const_image_view<image_type> img(img_);
    feature_pixel_values.resize(reference_pixel_deltas.size());

#if DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS
    cv::Mat canvas;
    cv::cvtColor( dlib::toMat(const_cast<image_type&>(img_)), canvas, cv::COLOR_GRAY2BGR);
    //cv::Mat canvas(area.height(), area.width(), CV_8UC3, cv::Scalar::all(0));
#endif

    for (unsigned long i = 0; i < feature_pixel_values.size(); ++i)
    {
        // Compute the point in the current shape corresponding to the i-th pixel and
        // then map it from the normalized shape space into pixel space.
        // point p = tform_to_img(tform*reference_pixel_deltas[i] + location(current_shape, reference_pixel_anchor_idx[i]));

        auto p0 = location(current_shape, reference_pixel_anchor_idx[i]);
        auto p1 = tform*reference_pixel_deltas[i];
        dlib::point p = tform_to_img(p0 + p1);
        if (area.contains(p))
        {
            feature_pixel_values[i] = dlib::get_pixel_intensity(img[p.y()][p.x()]);
#if DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS
            cv::circle(canvas, cv::Point(p.x(), p.y()), 2, {0,255,255}, -1, 8);
#endif
        }
        else
        {
            feature_pixel_values[i] = 0; // original behaviour (fastest)
        }
    }

#if DRISHTI_DLIB_DO_VISUALIZE_FEATURE_POINTS
    for(int i = 0; i < current_shape.size()/2; i++)
    {
        auto q = tform_to_img(location(current_shape, i));
        cv::circle(canvas, cv::Point(q.x(), q.y()), 2, {0,255,0}, -1, 8);
    }
    cv::imshow("features", canvas), cv::waitKey(0); // opt
#endif
}

DRISHTI_END_NAMESPACE(impl) // end namespace impl

// ----------------------------------------------------------------------------------------

// ellipse is converted back to opencv format, where each parameter of the ellipse
// is stored the x-coordinate of a new point

template <typename T>
std::vector<dlib::vector<T,2>> convert_shape_to_points(const fshape &shape, int ellipse_count)
{
    int point_length = int((shape.size()-(ellipse_count*5))/2);
    std::vector<dlib::vector<T,2>> points(point_length + (ellipse_count*5));

    for (unsigned long i = 0; i < point_length; ++i)
    {
        points[i] = impl::location(shape, i);
    }

    // Convert trailing ellipse back to standard form:
    for(int i = 0; i < ellipse_count; i++)
    {
        const float *ptr = &shape(point_length * 2 + (i * 5));
        std::vector<float> phi(ptr, ptr + 5);
        cv::RotatedRect e = geometry::phiToEllipse(phi);
        std::vector<float> data = geometry::ellipseToVector(e);
        for(int j = 0; j < 5; j++)
        {
            points[point_length + (i * 5) + j] = dlib::vector<T,2>(data[j], 0);
        }
    }

    return points;
}


class shape_predictor
{
public:

    shape_predictor (
    )
    {}

    shape_predictor (
        const fshape& initial_shape_,
        const std::vector<std::vector<impl::regression_tree> >& forests_,
        const std::vector<std::vector<InterpolatedFeature>>& interpolated_features,
        std::shared_ptr<StandardizedPCA> &pca,
        bool npd = false,
        bool do_affine = false,
        int ellipse_count = 0
    ) : initial_shape(initial_shape_)
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

    void populate_f16()
    {
        for(auto &f : forests)
        {
            for(auto &g : f)
            {
                g.leaf_values_16.resize(g.leaf_values.size());
                for(int i = 0; i < g.leaf_values.size(); i++)
                {
                    auto &leaf = g.leaf_values[i];
                    auto &leaf16 = g.leaf_values_16[i];
                    leaf16.set_size(leaf.size());
                    drishti::core::convertFixedPoint(&leaf(0,0), &leaf16(0,0), int(leaf.size()), FIXED_PRECISION);
                }
            }
        }
    }

    shape_predictor (
        const fshape& initial_shape_,
        const std::vector<std::vector<impl::regression_tree>>& forests_,
        const std::vector<PointVecf>& pixel_coordinates,
        std::shared_ptr<StandardizedPCA> &pca,
        bool npd = false,
        bool do_affine = false,
        int ellipse_count = 0
    ) : initial_shape(initial_shape_)
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
    }

    unsigned long num_parts (
    ) const
    {
        return initial_shape.size()/2;
    }

    static void project(drishti::ml::StandardizedPCA &pca, fshape &src, fshape &dst)
    {
        cv::Mat1f projection = pca.project(cv::Mat1f(1, src.size(), &src(0)));
        dst.set_size(projection.cols, 1);
        memcpy(&dst(0), projection.ptr<float>(0), sizeof(float) * projection.cols);
    }

    static void back_project(drishti::ml::StandardizedPCA &pca, int n, fshape &src, fshape &dst)
    {
        cv::Mat1f back_projection = pca.backProject(cv::Mat1f(1, n, &src(0)));
        memcpy(&dst(0), back_projection.ptr<float>(), sizeof(float) * back_projection.cols);
    }

    template <typename image_type>
    dlib::full_object_detection operator()(
        const image_type& img,
        const dlib::rectangle& rect,
        fshape starter_shape,
        int iters = 2,
        int stages = std::numeric_limits<int>::max()
    ) const
    {
        DRISHTI_STREAM_LOG_FUNC(5,1,m_streamLogger);
        using namespace impl;

        bool do_pca = m_pca ? true : false;

        cv::Mat1f cs;
        fshape current_shape = starter_shape, current_shape_full_; // for PCA mode

        if(do_pca)
        {
            project(*m_pca, starter_shape, current_shape_full_);
        }

        std::vector<float> feature_pixel_values;
        size_t forestCount = std::min(int(forests.size()), stages);
        for (unsigned long iter = 0; iter < forestCount; ++iter)
        {
            auto &cs_ = current_shape;
            auto &is_ = initial_shape; // this is used to map pose indexed features to current shape

            for(int k = 0; k < iters; k++)
            {
                // Previously had for loop
                DRISHTI_STREAM_LOG_FUNC(5,2,m_streamLogger);

                if(do_pca)
                {
                    // Get euclidean model for current shape space estimate:
                    int current_pca_dim = int(forests[iter][0].leaf_values[0].size());
                    back_project(*m_pca, current_pca_dim, current_shape_full_, current_shape);
                }

                DRISHTI_STREAM_LOG_FUNC(5,3,m_streamLogger);
                if(interpolated_features.size())
                {
                    extract_feature_pixel_values(img, rect, cs_, interpolated_features[iter], feature_pixel_values);
                }
                else
                {
                    extract_feature_pixel_values(img, rect, cs_, is_, anchor_idx[iter], deltas[iter], feature_pixel_values, m_ellipse_count, m_do_affine);
                }

                fshape current_shape_;
                auto & active_shape = do_pca ? current_shape_ : current_shape;

                DRISHTI_STREAM_LOG_FUNC(5,4,m_streamLogger);
#if DRISHTI_BUILD_REGRESSION_FIXED_POINT
                // Fixed point is currently only working for PCA in most cases (check numerical overflow)
                dlib::matrix<int16_t,0,1> shape_accumulator;
                for(auto &f : forests[iter])
                {
                    add16sAnd16s(shape_accumulator, f(feature_pixel_values, Fixed(), m_npd), shape_accumulator);
                }
                active_shape.set_size(shape_accumulator.size());
                for(int i = 0; i < shape_accumulator.size(); i++)
                {
                    active_shape(i) = float(shape_accumulator(i)) / float(1 << FIXED_PRECISION);
                }

#else /* else don't DRISHTI_BUILD_REGRESSION_FIXED_POINT */
                for(auto &f : forests[iter])
                {
                    add32F(active_shape, f(feature_pixel_values, m_npd), active_shape);
                }
#endif /* DRISHTI_BUILD_REGRESSION_FIXED_POINT */

                if(do_pca)
                {
                    DRISHTI_STREAM_LOG_FUNC(5,5,m_streamLogger);
                    dlib::set_rowm(current_shape_full_, dlib::range(0, current_shape_.size()-1)) += current_shape_;
                }
            }
        }

        if(do_pca)
        {
            // Convert the final model back to euclidean
            DRISHTI_STREAM_LOG_FUNC(5,6,m_streamLogger);
            int current_pca_dim = int(forests.back()[0].leaf_values[0].size());
            back_project(*m_pca, current_pca_dim, current_shape_full_, current_shape);
        }

        // convert the current_shape into a full_object_detection
        const dlib::point_transform_affine tform_to_img = unnormalizing_tform(rect);

        int point_length = int((current_shape.size()-(m_ellipse_count*5))/2);
        std::vector<dlib::point> parts(point_length + (m_ellipse_count*5));
        for (unsigned long i = 0; i < point_length; ++i)
        {
            parts[i] = tform_to_img(location(current_shape, i));
        }

        // Convert trailing ellipse back to standard form:
        DRISHTI_STREAM_LOG_FUNC(5,7,m_streamLogger);
        for(int i = 0; i < m_ellipse_count; i++)
        {
            std::vector<float> phi(5, 0.f);
            for(int j = 0; j < 5; j++)
            {
                phi[j] = current_shape(point_length * 2 + (i * 5) + j);
            }

            const auto &m = tform_to_img.get_m();
            const auto &b = tform_to_img.get_b();
            cv::Matx33f H(m(0,0), m(0,1), b(0), m(1,0), m(1,1), b(1), 0, 0, 1);
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
            cv::Mat image = dlib::toMat( const_cast<image_type&>(img) );
            cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
            for(int j = 0; j < point_length; j++)
            {
                cv::circle(image, cv::Point(parts[j].x(), parts[j].y()), 2, {0,255,0}, 1, 8);
            }

            for(int i = 0; i < ellipse_count; i++)
            {
                cv::RotatedRect e1;
                e1.center.x = parts[point_length + 0].x();
                e1.center.y = parts[point_length + 1].x();
                e1.size.width = parts[point_length + 2].x();
                e1.size.height = parts[point_length + 3].x();
                e1.angle= parts[point_length + 4].x();

                cv::ellipse(image, e1, {0,255,0}, 1, 8);
            }
            cv::imshow("image", image), cv::waitKey(0);
        }
#endif
        return dlib::full_object_detection(rect, parts);
    }

    template <typename image_type>
    dlib::full_object_detection operator()(
        const image_type& img,
        const dlib::rectangle& rect
    ) const
    {
        fshape current_shape = initial_shape;
        return (*this)(img, rect, current_shape);
    }

    friend void serialize (const shape_predictor& item, std::ostream& out)
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
    friend void deserialize (shape_predictor& item, std::istream& in)
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

    void setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
    {
        m_streamLogger = logger;
    }

    fshape initial_shape;
    std::vector<std::vector<impl::regression_tree> > forests;

    // Pose indexing relative to nearest landmark points:
    std::vector<std::vector<unsigned short> > anchor_idx;
    std::vector<PointVecf > deltas;

    // PCA reduction:
    std::shared_ptr<drishti::ml::StandardizedPCA> m_pca; // global pca
    int m_ellipse_count = 0;
    bool m_npd = false;
    bool m_do_affine = false;

    // Use interpolated "line indexed" features (stead of the relative encoding above):
    std::vector<std::vector<InterpolatedFeature>> interpolated_features;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

// ----------------------------------------------------------------------------------------
#if !DRISHTI_BUILD_MIN_SIZE

class shape_predictor_trainer
{
    /*!
        This thing really only works with unsigned char or rgb_pixel images (since we assume the threshold
        should be in the range [-128,128]).
    !*/
public:

    shape_predictor_trainer (
    )
    {
        _cascade_depth = 10;
        _tree_depth = 4;
        _num_trees_per_cascade_level = 500;
        _nu = 0.1;
        _oversampling_amount = 20;
        _feature_pool_size = 400;
        _lambda = 0.1;
        _num_test_splits = 20;
        _feature_pool_region_padding = 0;
        _verbose = false;
    }

    unsigned long get_cascade_depth (
    ) const
    {
        return _cascade_depth;
    }

    void set_cascade_depth (
        unsigned long depth
    )
    {
        DLIB_CASSERT(depth > 0,
                     "\t void shape_predictor_trainer::set_cascade_depth()"
                     << "\n\t Invalid inputs were given to this function. "
                     << "\n\t depth:  " << depth
                    );

        _cascade_depth = depth;
    }

    unsigned long get_tree_depth (
    ) const
    {
        return _tree_depth;
    }

    void set_tree_depth (
        unsigned long depth
    )
    {
        DLIB_CASSERT(depth > 0,
                     "\t void shape_predictor_trainer::set_tree_depth()"
                     << "\n\t Invalid inputs were given to this function. "
                     << "\n\t depth:  " << depth
                    );

        _tree_depth = depth;
    }

    unsigned long get_num_trees_per_cascade_level (
    ) const
    {
        return _num_trees_per_cascade_level;
    }

    void set_num_trees_per_cascade_level (
        unsigned long num
    )
    {
        DLIB_CASSERT( num > 0,
                      "\t void shape_predictor_trainer::set_num_trees_per_cascade_level()"
                      << "\n\t Invalid inputs were given to this function. "
                      << "\n\t num:  " << num
                    );
        _num_trees_per_cascade_level = num;
    }

    double get_nu (
    ) const
    {
        return _nu;
    }
    void set_nu (
        double nu
    )
    {
        DLIB_CASSERT(0 < nu && nu <= 1,
                     "\t void shape_predictor_trainer::set_nu()"
                     << "\n\t Invalid inputs were given to this function. "
                     << "\n\t nu:  " << nu
                    );

        _nu = nu;
    }

    std::string get_random_seed (
    ) const
    {
        return rnd.get_seed();
    }
    void set_random_seed (
        const std::string& seed
    )
    {
        rnd.set_seed(seed);
    }

    unsigned long get_oversampling_amount (
    ) const
    {
        return _oversampling_amount;
    }
    void set_oversampling_amount (
        unsigned long amount
    )
    {
        DLIB_CASSERT(amount > 0,
                     "\t void shape_predictor_trainer::set_oversampling_amount()"
                     << "\n\t Invalid inputs were given to this function. "
                     << "\n\t amount: " << amount
                    );

        _oversampling_amount = amount;
    }

    unsigned long get_feature_pool_size (
    ) const
    {
        return _feature_pool_size;
    }
    void set_feature_pool_size (
        unsigned long size
    )
    {
        DLIB_CASSERT(size > 1,
                     "\t void shape_predictor_trainer::set_feature_pool_size()"
                     << "\n\t Invalid inputs were given to this function. "
                     << "\n\t size: " << size
                    );

        _feature_pool_size = size;
    }

    double get_lambda (
    ) const
    {
        return _lambda;
    }
    void set_lambda (
        double lambda
    )
    {
        DLIB_CASSERT(lambda > 0,
                     "\t void shape_predictor_trainer::set_lambda()"
                     << "\n\t Invalid inputs were given to this function. "
                     << "\n\t lambda: " << lambda
                    );

        _lambda = lambda;
    }

    unsigned long get_num_test_splits (
    ) const
    {
        return _num_test_splits;
    }
    void set_num_test_splits (
        unsigned long num
    )
    {
        DLIB_CASSERT(num > 0,
                     "\t void shape_predictor_trainer::set_num_test_splits()"
                     << "\n\t Invalid inputs were given to this function. "
                     << "\n\t num: " << num
                    );

        _num_test_splits = num;
    }


    double get_feature_pool_region_padding (
    ) const
    {
        return _feature_pool_region_padding;
    }
    void set_feature_pool_region_padding (
        double padding
    )
    {
        _feature_pool_region_padding = padding;
    }

    void be_verbose (
    )
    {
        _verbose = true;
    }

    void be_quiet (
    )
    {
        _verbose = false;
    }

    static void copyShape(const float *ptr, int n, fshape &shape, fshape &shape_full)
    {
        shape_full.set_size(n, 1);
        memcpy(&shape_full(0), ptr, sizeof(float) * n);
        shape = shape_full;
    }

    template <typename image_array>
    shape_predictor train (
        const image_array& images,
        const std::vector<std::vector<dlib::full_object_detection> >& objects,
        const std::vector<int> &dimensions,
        const int ellipse_count = 0, /* trailing N * 5 params represent ellipses and need different normalization */
        bool do_npd = false,
        bool do_affine = false,
        const dlib::drectangle &roi= {0.f,0.f,0.f,0.f},
        bool do_line_indexed = false
    ) const
    {
        using namespace impl;
        DLIB_CASSERT(images.size() == objects.size() && images.size() > 0,
                     "\t shape_predictor shape_predictor_trainer::train()"
                     << "\n\t Invalid inputs were given to this function. "
                     << "\n\t images.size():  " << images.size()
                     << "\n\t objects.size(): " << objects.size()
                    );
        // make sure the objects agree on the number of parts and that there is at
        // least one full_object_detection.
        unsigned long num_parts = 0;
        for (unsigned long i = 0; i < objects.size(); ++i)
        {
            for (unsigned long j = 0; j < objects[i].size(); ++j)
            {
                if (num_parts == 0)
                {
                    num_parts = objects[i][j].num_parts();
                    DLIB_CASSERT(objects[i][j].num_parts() != 0,
                                 "\t shape_predictor shape_predictor_trainer::train()"
                                 << "\n\t You can't give objects that don't have any parts to the trainer."
                                );
                }
                else
                {
                    DLIB_CASSERT(objects[i][j].num_parts() == num_parts,
                                 "\t shape_predictor shape_predictor_trainer::train()"
                                 << "\n\t All the objects must agree on the number of parts. "
                                 << "\n\t objects["<<i<<"]["<<j<<"].num_parts(): " << objects[i][j].num_parts()
                                 << "\n\t num_parts:  " << num_parts
                                );
                }
            }
        }
        DLIB_CASSERT(num_parts != 0,
                     "\t shape_predictor shape_predictor_trainer::train()"
                     << "\n\t You must give at least one full_object_detection if you want to train a shape model and it must have parts."
                    );

        rnd.set_seed(get_random_seed());

        DLIB_CASSERT (!(ellipse_count % 2), "\t currently limited to ellipse pairs"); // point representation limitations

        // For ellipse only, pose indexing is performed via homography:
        bool is_ellipse_only = (ellipse_count*5) == (num_parts * 2); // really only works for ellipse pairs

        std::vector<training_sample> samples;
        const fshape initial_shape = populate_training_sample_shapes(objects, samples, ellipse_count);

        std::vector<PointVecf> pixel_coordinates;
        std::vector<std::vector<InterpolatedFeature>> interpolated_features;

        if(is_ellipse_only)
        {
            assert(false);
            // radomly sample pixels on unit sphere here:
            // TODO
        }
        else if(do_line_indexed)
        {
            randomly_sample_pixel_coordinates_between_features(interpolated_features, pixel_coordinates, initial_shape, ellipse_count, roi);
        }
        else
        {
            pixel_coordinates = randomly_sample_pixel_coordinates(initial_shape, ellipse_count, roi);
        }

        // PCA for shape space regression:
        const int num_dim = int(initial_shape.size());
        bool do_pca = dimensions.size() > 0;
        std::shared_ptr<StandardizedPCA> pca;
        if(do_pca)
        {
            pca = compute_pca(samples, num_dim, dimensions);
        }

        unsigned long trees_fit_so_far = 0;
        dlib::console_progress_indicator pbar(get_cascade_depth()*get_num_trees_per_cascade_level());
        if (_verbose)
        {
            std::cout << "Fitting trees..." << std::endl;
        }

        std::vector<std::vector<impl::regression_tree> > forests(get_cascade_depth());
        // Now start doing the actual training by filling in the forests
        for (unsigned long cascade = 0; cascade < get_cascade_depth(); ++cascade)
        {
            int current_pca_dim = do_pca ? dimensions[cascade] : num_dim;

            // We proceed to fit models coarse-to-fine in shape space, increasing dimensionality at each cascade:
            if(do_pca)
            {
                initialize_shape_space_models(samples, current_pca_dim, pca);
            }

            // Each cascade uses a different set of pixels for its features.  We compute
            // their representations relative to the initial shape first.
            std::vector<unsigned short> anchor_idx;
            PointVecf deltas;

            // For line indexed features we don't need the delta
            if(is_ellipse_only)
            {
                // TODO: Just use homography corresopnding to first ellipse
                assert(false);
            }
            else if(!do_line_indexed)
            {
                create_shape_relative_encoding(initial_shape, pixel_coordinates[cascade], anchor_idx, deltas, ellipse_count);
            }

            // First compute the feature_pixel_values for each training sample at this
            // level of the cascade.
            for (auto &s : samples)
            {
                auto &is = initial_shape;
                const auto &cs = s.current_shape;
                const auto &image = images[s.image_idx];

                if(is_ellipse_only)
                {
                    // TODO: Just use homography corresponding to first ellipse:
                    assert(false);
                }
                else if(do_line_indexed)
                {
                    extract_feature_pixel_values(image, s.rect, cs, interpolated_features[cascade], s.feature_pixel_values);
                }
                else
                {
                    extract_feature_pixel_values(image, s.rect, cs, is, anchor_idx, deltas, s.feature_pixel_values, ellipse_count, do_affine);
                }
            }

            // Now start building the trees at this cascade level.
            for (unsigned long i = 0; i < get_num_trees_per_cascade_level(); ++i)
            {
                forests[cascade].push_back(make_regression_tree(samples, pixel_coordinates[cascade], do_npd, do_pca));
                if (_verbose)
                {
                    ++trees_fit_so_far;
                    pbar.print_status(trees_fit_so_far);
                }
            }

            if(do_pca)
            {
                update_shape_space_models(samples, current_pca_dim);
            }
        }

        if (_verbose)
        {
            std::cout << "Training complete                          " << std::endl;
        }

        if(interpolated_features.size())
        {
            return shape_predictor(initial_shape, forests, interpolated_features, pca, do_npd, do_affine, ellipse_count);
        }
        else
        {
            return shape_predictor(initial_shape, forests, pixel_coordinates, pca, do_npd, do_affine, ellipse_count);
        }
    }

private:

    static fshape object_to_shape (
        const dlib::full_object_detection& obj,
        int ellipse_count = 0
    )
    {
        fshape shape(obj.num_parts()*2 - (5 * ellipse_count));
        const dlib::point_transform_affine tform_from_img = impl::normalizing_tform(obj.get_rect());

        int end = int(obj.num_parts()) - (ellipse_count * 5);
        for (unsigned long i = 0; i < end; ++i)
        {
            dlib::vector<float,2> p = tform_from_img(obj.part(i));
            shape(2*i+0) = p.x();
            shape(2*i+1) = p.y();
        }

        // Normalize ellipse in homogeneous coordinates:
        for(int i = end, j = 2 * end; i < (end + ellipse_count*5); i+= 5)
        {
            cv::RotatedRect e, e2;
            e.center.x =    obj.part(i+0).x();
            e.center.y =    obj.part(i+1).x();
            e.size.width =  obj.part(i+2).x();
            e.size.height = obj.part(i+3).x();
            e.angle =       obj.part(i+4).x();

            const auto &m = tform_from_img.get_m();
            const auto &b = tform_from_img.get_b();
            cv::Matx33f H(m(0,0), m(0,1), b(0), m(1,0), m(1,1), b(1), 0, 0, 1);

            e2 = H * e; // this will preserve the orientation

            std::vector<float> phi = geometry::ellipseToVector(e2);
            for(int k = 0; k < phi.size(); k++, j++)
            {
                shape(j) = phi[k];
            }
        }

        return shape;
    }

    struct training_sample
    {
        /*!

        CONVENTION
            - feature_pixel_values.size() == get_feature_pool_size()
            - feature_pixel_values[j] == the value of the j-th feature pool
              pixel when you look it up relative to the shape in current_shape.

            - target_shape == The truth shape.  Stays constant during the whole
              training process.
            - rect == the position of the object in the image_idx-th image.  All shape
              coordinates are coded relative to this rectangle.

            - trailing _ indicates shape space projection:
        !*/

        unsigned long image_idx;
        dlib::rectangle rect;

        fshape target_shape, target_shape_, target_shape_full_;

        fshape current_shape, current_shape_, current_shape_full_;
        std::vector<float> feature_pixel_values;

        void swap(training_sample& item)
        {
            std::swap(image_idx, item.image_idx);
            std::swap(rect, item.rect);
            target_shape.swap(item.target_shape);
            current_shape.swap(item.current_shape);
            feature_pixel_values.swap(item.feature_pixel_values);

            target_shape_.swap(item.target_shape_);
            current_shape_.swap(item.current_shape_);

            target_shape_full_.swap(item.target_shape_full_);
            current_shape_full_.swap(item.current_shape_full_);
        }
    };

    void update_shape_space_models(std::vector<training_sample> &samples, int current_pca_dim) const
    {
        auto current_range = dlib::range(0, current_pca_dim-1);
        for(auto &s : samples)
        {
            // Update our shape space model:
            s.current_shape_full_ = 0;
            dlib::set_rowm(s.current_shape_full_, current_range) = s.current_shape_;
        }
    }

    void initialize_shape_space_models(std::vector<training_sample> &samples, int current_pca_dim, std::shared_ptr<StandardizedPCA> &pca) const
    {
        auto current_range = dlib::range(0, current_pca_dim-1);
        for(auto &s : samples)
        {
            s.current_shape_ = dlib::rowm(s.current_shape_full_, current_range);
            s.target_shape_ = dlib::rowm(s.target_shape_full_, current_range);

            CV_Assert(s.current_shape_.size() == current_pca_dim);
            CV_Assert(s.target_shape_.size() == current_pca_dim);

            // Back project from (potentially coarse) shape space to the full euclidean model:
            cv::Mat1f cs1_(1, int(s.current_shape_.size()), &s.current_shape_(0));
            cv::Mat1f cs1 = pca->backProject(cs1_);
            memcpy(&s.current_shape(0), cs1.ptr<float>(), sizeof(float) * cs1.cols);

#if DRISHTI_DLIB_DO_NUMERIC_DEBUG
            for(auto &v : s.current_shape_)
            {
                CV_Assert( !std::isnan(v) );
            }
            for(auto &v : s.target_shape_)
            {
                CV_Assert( !std::isnan(v) );
            }
            for(auto &v : s.current_shape)
            {
                CV_Assert( !std::isnan(v) );
            }
            for(auto &v : s.current_shape_full_)
            {
                CV_Assert( !std::isnan(v) );
            }
            if(first)
            {
                std::cout << dlib::trans(s.current_shape) << std::endl;
                std::cout << dlib::trans(s.current_shape_full_) << std::endl;
                std::cout << dlib::trans(s.current_shape_) << std::endl;
                std::cout << dlib::trans(s.target_shape_) << std::endl;
                first = false;
            }
#endif
        }
    }

    using IntVec = std::vector<int>;
    using SampleVec = std::vector<training_sample>;

    std::shared_ptr<StandardizedPCA> compute_pca(SampleVec &samples, int num_dim, const IntVec &dimensions) const
    {
        using namespace impl;
        CV_Assert(int(dimensions.size()) == get_cascade_depth());

        cv::Mat1f ts, cs, ts_, cs_;
        int max_pca_dim = *std::max_element(dimensions.begin(), dimensions.end());

        std::vector<cv::Mat1f> tss, css, extra;
        for(int i = 0; i < samples.size(); i++)
        {
            tss.push_back(cv::Mat1f(1, num_dim, &samples[i].target_shape(0)));
            css.push_back(cv::Mat1f(1, num_dim, &samples[i].current_shape(0)));
        }

        cv::vconcat(tss, ts);
        cv::vconcat(css, cs);

        // Use the full set of target and current (randomized) poses for PCA
        cv::Mat full;
        cv::vconcat(ts, cs, full);

        auto pca = std::make_shared<StandardizedPCA>();

        cv::Mat full_;
        pca->compute(full, full_, max_pca_dim);

        //cv::Mat full_;
        //pca->compute(ts, full_, max_pca_dim);

        ts_ = pca->project(ts);
        cs_ = pca->project(cs);

        for(int i = 0; i < samples.size(); i++)
        {
            copyShape(cs_.ptr<float>(i), cs_.cols, samples[i].current_shape_, samples[i].current_shape_full_);
            copyShape(ts_.ptr<float>(i), ts_.cols, samples[i].target_shape_, samples[i].target_shape_full_);
        }

        return pca;
    }

    impl::regression_tree make_regression_tree (
        std::vector<training_sample>& samples,
        const PointVecf& pixel_coordinates,
        bool do_npd = false,
        bool do_pca = false
    ) const
    {
        using namespace impl;
        std::deque<std::pair<unsigned long, unsigned long> > parts;
        parts.push_back(std::make_pair(0, (unsigned long)samples.size()));

        impl::regression_tree tree;

        // walk the tree in breadth first order
        const unsigned long num_split_nodes = static_cast<unsigned long>(std::pow(2.0, (double)get_tree_depth())-1);
        std::vector<fshape > sums(num_split_nodes*2+1);
        for (unsigned long i = 0; i < samples.size(); ++i)
        {
            if(do_pca) // #if DO_PCA_INTERNAL
            {
                sums[0] += samples[i].target_shape_ - samples[i].current_shape_;
                //CV_Assert(sums[0].size() == 2);
            }
            else
            {
                sums[0] += samples[i].target_shape - samples[i].current_shape;
            }
        }

        for (unsigned long i = 0; i < num_split_nodes; ++i)
        {
            std::pair<unsigned long,unsigned long> range = parts.front();
            parts.pop_front();

            auto & sumsL = sums[left_child(i)];
            auto & sumsR = sums[right_child(i)];
            impl::split_feature split = generate_split(samples, range.first, range.second, pixel_coordinates, sums[i], sumsL, sumsR, do_npd, do_pca);
            tree.splits.push_back(split);
            const unsigned long mid = partition_samples(split, samples, range.first, range.second, do_npd);

            parts.push_back(std::make_pair(range.first, mid));
            parts.push_back(std::make_pair(mid, range.second));
        }

        // Now all the parts contain the ranges for the leaves so we can use them to
        // compute the average leaf values.
        tree.leaf_values.resize(parts.size());
        for (unsigned long i = 0; i < parts.size(); ++i)
        {
            if (parts[i].second != parts[i].first)
            {
                tree.leaf_values[i] = sums[num_split_nodes+i]*get_nu()/(parts[i].second - parts[i].first);
            }
            else
            {
                if(do_pca)
                {
                    tree.leaf_values[i] = zeros_matrix(samples[0].target_shape_);
                }
                else
                {
                    tree.leaf_values[i] = zeros_matrix(samples[0].target_shape);
                }
            }

            // now adjust the current shape based on these predictions
            for (unsigned long j = parts[i].first; j < parts[i].second; ++j)
            {
                if(do_pca)
                {
                    samples[j].current_shape_ += tree.leaf_values[i];
                }
                else
                {
                    samples[j].current_shape += tree.leaf_values[i];
                }
            }
        }

        return tree;
    }

    impl::split_feature randomly_generate_split_feature (
        const PointVecf& pixel_coordinates,
        bool do_npd = false
    ) const
    {
        const double lambda = get_lambda();
        impl::split_feature feat;
        double accept_prob;
        do
        {
            feat.idx1 = rnd.get_random_16bit_number()%get_feature_pool_size();
            feat.idx2 = rnd.get_random_16bit_number()%get_feature_pool_size();
            const double dist = length(pixel_coordinates[feat.idx1]-pixel_coordinates[feat.idx2]);
            accept_prob = std::exp(-dist/lambda);
        }
        while(feat.idx1 == feat.idx2 || !(accept_prob > rnd.get_random_double()));

        if(do_npd)
        {
            feat.thresh = ((rnd.get_random_double() * 2.0) - 1.0);
        }
        else
        {
            feat.thresh = (rnd.get_random_double()*256 - 128)/2.0;
        }

        return feat;
    }

    impl::split_feature generate_split (
        const std::vector<training_sample>& samples,
        unsigned long begin,
        unsigned long end,
        const PointVecf& pixel_coordinates,
        const fshape& sum,
        fshape& left_sum,
        fshape& right_sum,
        bool do_npd = false,
        bool do_pca = false

    ) const
    {
        // generate a bunch of random splits and test them and return the best one.

        const unsigned long num_test_splits = get_num_test_splits();

        // sample the random features we test in this function
        std::vector<impl::split_feature> feats;
        feats.reserve(num_test_splits);
        for (unsigned long i = 0; i < num_test_splits; ++i)
        {
            feats.push_back(randomly_generate_split_feature(pixel_coordinates, do_npd));
        }

        std::vector<fshape > left_sums(num_test_splits);
        std::vector<unsigned long> left_cnt(num_test_splits);

        // now compute the sums of vectors that go left for each feature
        fshape temp;
        for (unsigned long j = begin; j < end; ++j)
        {
            if(do_pca) // #if DO_PCA_INTERNAL
            {
                temp = samples[j].target_shape_-samples[j].current_shape_;
            } // #else
            else
            {
                temp = samples[j].target_shape-samples[j].current_shape;
            } // #endif

            if(do_npd)
            {
                for (unsigned long i = 0; i < num_test_splits; ++i)
                {
                    if( compute_npd(samples[j].feature_pixel_values[feats[i].idx1], samples[j].feature_pixel_values[feats[i].idx2]) > feats[i].thresh )
                    {
                        left_sums[i] += temp;
                        ++left_cnt[i];
                    }
                }
            }
            else
            {
                for (unsigned long i = 0; i < num_test_splits; ++i)
                {
                    if (samples[j].feature_pixel_values[feats[i].idx1] - samples[j].feature_pixel_values[feats[i].idx2] > feats[i].thresh)
                    {
                        left_sums[i] += temp;
                        ++left_cnt[i];
                    }
                }
            }
        }

        // now figure out which feature is the best
        double best_score = -1;
        unsigned long best_feat = 0;
        for (unsigned long i = 0; i < num_test_splits; ++i)
        {
            // check how well the feature splits the space.
            double score = 0;
            unsigned long right_cnt = end-begin-left_cnt[i];
            if (left_cnt[i] != 0 && right_cnt != 0)
            {
                temp = sum - left_sums[i];
                score = dot(left_sums[i],left_sums[i])/left_cnt[i] + dot(temp,temp)/right_cnt;
                if (score > best_score)
                {
                    best_score = score;
                    best_feat = i;
                }
            }
        }

        left_sums[best_feat].swap(left_sum);
        if (left_sum.size() != 0)
        {
            right_sum = sum - left_sum;
        }
        else
        {
            right_sum = sum;
            left_sum = zeros_matrix(sum);
        }
        return feats[best_feat];
    }

    unsigned long partition_samples (
        const impl::split_feature& split,
        std::vector<training_sample>& samples,
        unsigned long begin,
        unsigned long end,
        bool do_npd = false
    ) const
    {
        // splits samples based on split (sorta like in quick sort) and returns the mid
        // point.  make sure you return the mid in a way compatible with how we walk
        // through the tree.

        unsigned long i = begin;

        if(do_npd)
        {
            for (unsigned long j = begin; j < end; ++j)
            {
                if(compute_npd(samples[j].feature_pixel_values[split.idx1], samples[j].feature_pixel_values[split.idx2]) > split.thresh)
                {
                    samples[i].swap(samples[j]);
                    ++i;
                }
            }
        }
        else
        {
            for (unsigned long j = begin; j < end; ++j)
            {
                if (samples[j].feature_pixel_values[split.idx1] - samples[j].feature_pixel_values[split.idx2] > split.thresh)
                {
                    samples[i].swap(samples[j]);
                    ++i;
                }
            }

        }
        return i;
    }

    fshape populate_training_sample_shapes(
        const std::vector<std::vector<dlib::full_object_detection> >& objects,
        std::vector<training_sample>& samples,
        int ellipse_count = 0
    ) const
    {
        samples.clear();
        fshape mean_shape;
        long count = 0;
        // first fill out the target shapes
        for (unsigned long i = 0; i < objects.size(); ++i)
        {
            for (unsigned long j = 0; j < objects[i].size(); ++j)
            {
                training_sample sample;
                sample.image_idx = i;
                sample.rect = objects[i][j].get_rect();
                sample.target_shape = object_to_shape(objects[i][j], ellipse_count);
                for (unsigned long itr = 0; itr < get_oversampling_amount(); ++itr)
                {
                    samples.push_back(sample);
                }
                mean_shape += sample.target_shape;
                ++count;
            }
        }

        mean_shape /= count;

        // now go pick random initial shapes
        for (unsigned long i = 0; i < samples.size(); ++i)
        {
            if ((i%get_oversampling_amount()) == 0)
            {
                // The mean shape is what we really use as an initial shape so always
                // include it in the training set as an example starting shape.
                samples[i].current_shape = mean_shape;
            }
            else
            {
                // Pick a random convex combination of two of the target shapes and use
                // that as the initial shape for this sample.
                const unsigned long rand_idx1 = rnd.get_random_32bit_number()%samples.size();
                const unsigned long rand_idx2 = rnd.get_random_32bit_number()%samples.size();
                const double alpha = rnd.get_random_double();
                samples[i].current_shape = alpha*samples[rand_idx1].target_shape + (1.0-alpha)*samples[rand_idx2].target_shape;
            }
        }

        return mean_shape;
    }

    // ================ line features =====================
    void randomly_sample_pixel_coordinates_between_features (
        const fshape &initial_shape,
        std::vector<InterpolatedFeature> &locations,
        PointVecf& pixel_coordinates,
        double padding,
        const dlib::drectangle &roi= {0.f,0.f,0.f,0.f} // exclusion roi
    ) const
    /*!
     ensures
     - #pixel_coordinates.size() == get_feature_pool_size()
     - for all valid i:
     - location[i] == a line indexed feature point (between two features)
     !*/
    {
        int point_length = int(initial_shape.size()) / 2;
        locations.resize(get_feature_pool_size());
        pixel_coordinates.resize(get_feature_pool_size());
        for (unsigned long i = 0; i < get_feature_pool_size(); ++i)
        {
            fpoint p;
            do
            {
                // Avoid points inside the roi:
                locations[i].f1 = rnd.get_random_32bit_number() % point_length;
                locations[i].f2 = rnd.get_random_32bit_number() % point_length;
                locations[i].alpha = (rnd.get_random_double() * (1.0 + 2.0 * padding)) - padding; // [-padding ... 1.0+padding]
                pixel_coordinates[i] = impl::interpolate_feature_point(locations[i], initial_shape);
            }
            while(roi.contains(pixel_coordinates[i]));
        }
    }

    void randomly_sample_pixel_coordinates_between_features (
        std::vector<std::vector<InterpolatedFeature>>& indexed_features,
        std::vector<PointVecf>& pixel_coordinates,
        const fshape& initial_shape,
        int ellipse_count = 0,
        const dlib::drectangle &roi= {0.f,0.f,0.f,0.f}
    ) const
    {
        const double padding = get_feature_pool_region_padding();
        // Figure out the bounds on the object shapes.  We will sample uniformly
        // from this box.

        // matrix range op
        int ellipseLength = (5 * ellipse_count);
        dlib::matrix<float> temp_ = dlib::subm(initial_shape, dlib::range(0, initial_shape.size() - 1 - ellipseLength), dlib::range(0,1));

        dlib::matrix<float> temp = reshape(temp_, temp_.size()/2, 2);
        pixel_coordinates.resize(get_cascade_depth());
        indexed_features.resize(get_cascade_depth());
        for (unsigned long i = 0; i < get_cascade_depth(); ++i)
        {
            randomly_sample_pixel_coordinates_between_features(temp_, indexed_features[i], pixel_coordinates[i], padding, roi);
        }
    }

    void randomly_sample_pixel_coordinates (
        PointVecf& pixel_coordinates,
        const double min_x,
        const double min_y,
        const double max_x,
        const double max_y,
        const dlib::drectangle &roi= {0.f,0.f,0.f,0.f}
    ) const
    /*!
        ensures
            - #pixel_coordinates.size() == get_feature_pool_size()
            - for all valid i:
                - pixel_coordinates[i] == a point in the box defined by the min/max x/y arguments.
    !*/
    {
        pixel_coordinates.resize(get_feature_pool_size());
        for (unsigned long i = 0; i < get_feature_pool_size(); ++i)
        {
            do
            {
                // Avoid points inside the roi:
                pixel_coordinates[i].x() = rnd.get_random_double()*(max_x-min_x) + min_x;
                pixel_coordinates[i].y() = rnd.get_random_double()*(max_y-min_y) + min_y;
            }
            while(roi.contains(pixel_coordinates[i]));
        }
    }

    std::vector<PointVecf > randomly_sample_pixel_coordinates (
        const fshape& initial_shape,
        int ellipse_count = 0,
        const dlib::drectangle &roi= {0.f,0.f,0.f,0.f}
    ) const
    {
        const double padding = get_feature_pool_region_padding();
        // Figure out the bounds on the object shapes.  We will sample uniformly
        // from this box.

        int ellipseLength = (5 * ellipse_count);
        // matrix range op
        dlib::matrix<float> temp_ = dlib::subm(initial_shape, dlib::range(0, initial_shape.size() - 1 - ellipseLength),  dlib::range(0,1));

        dlib::matrix<float> temp = reshape(temp_, temp_.size()/2, 2);
        const double min_x = min(colm(temp,0))-padding;
        const double min_y = min(colm(temp,1))-padding;
        const double max_x = max(colm(temp,0))+padding;
        const double max_y = max(colm(temp,1))+padding;

        std::vector<PointVecf > pixel_coordinates;
        pixel_coordinates.resize(get_cascade_depth());
        for (unsigned long i = 0; i < get_cascade_depth(); ++i)
        {
            randomly_sample_pixel_coordinates(pixel_coordinates[i], min_x, min_y, max_x, max_y, roi);
        }
        return pixel_coordinates;
    }

    mutable dlib::rand rnd;

    unsigned long _cascade_depth;
    unsigned long _tree_depth;
    unsigned long _num_trees_per_cascade_level;
    double _nu;
    unsigned long _oversampling_amount;
    unsigned long _feature_pool_size;
    double _lambda;
    unsigned long _num_test_splits;
    double _feature_pool_region_padding;
    bool _verbose;

    // experimental
    std::map<int,impl::recipe> _recipe_for_cascade_level;
};

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

template <
    typename image_array
    >
double test_shape_predictor (
    const shape_predictor& sp,
    const image_array& images,
    const std::vector<std::vector<dlib::full_object_detection> >& objects,
    const std::vector<std::vector<double> >& scales,
    int ellipse_count = 0
)
{
    // make sure requires clause is not broken
#ifdef ENABLE_ASSERTS
    DLIB_CASSERT( images.size() == objects.size() ,
                  "\t double test_shape_predictor()"
                  << "\n\t Invalid inputs were given to this function. "
                  << "\n\t images.size():  " << images.size()
                  << "\n\t objects.size(): " << objects.size()
                );
    for (unsigned long i = 0; i < objects.size(); ++i)
    {
        for (unsigned long j = 0; j < objects[i].size(); ++j)
        {
            DLIB_CASSERT(objects[i][j].num_parts() == sp.num_parts(),
                         "\t double test_shape_predictor()"
                         << "\n\t Invalid inputs were given to this function. "
                         << "\n\t objects["<<i<<"]["<<j<<"].num_parts(): " << objects[i][j].num_parts()
                         << "\n\t sp.num_parts(): " << sp.num_parts()
                        );
        }
        if (scales.size() != 0)
        {
            DLIB_CASSERT(objects[i].size() == scales[i].size(),
                         "\t double test_shape_predictor()"
                         << "\n\t Invalid inputs were given to this function. "
                         << "\n\t objects["<<i<<"].size(): " << objects[i].size()
                         << "\n\t scales["<<i<<"].size(): " << scales[i].size()
                        );

        }
    }
#endif

    dlib::running_stats<double> rs;
    for (unsigned long i = 0; i < objects.size(); ++i)
    {
        for (unsigned long j = 0; j < objects[i].size(); ++j)
        {
            // Just use a scale of 1 (i.e. no scale at all) if the caller didn't supply
            // any scales.
            const double scale = scales.size()==0 ? 1 : scales[i][j];

            dlib::full_object_detection det = sp(images[i], objects[i][j].get_rect());

            for (unsigned long k = 0; k < det.num_parts(); ++k)
            {
                double score = length(det.part(k) - objects[i][j].part(k))/scale;
                rs.add(score);
            }
        }
    }
    return rs.mean();
}

template <
    typename image_array
    >
double test_shape_predictor (
    const shape_predictor& sp,
    const image_array& images,
    const std::vector<std::vector<dlib::full_object_detection> >& objects
)
{
    std::vector<std::vector<double> > no_scales;
    return test_shape_predictor(sp, images, objects, no_scales);
}

#endif // !DRISHTI_BUILD_MIN_SIZE

void serialize(const drishti::ml::shape_predictor& item, std::ostream& out);
void deserialize(drishti::ml::shape_predictor& item, std::istream& in);

DRISHTI_ML_NAMESPACE_END

typedef drishti::ml::impl::regression_tree RTType;
typedef dlib::vector<float,2> Vec2Type;

#if DRISHTI_DLIB_DO_HALF
struct PointHalf
{
    PointHalf() {}

    PointHalf( const Vec2Type &src )
        : x( half_float::detail::float2half<std::round_to_nearest>(src(0)) )
        , y( half_float::detail::float2half<std::round_to_nearest>(src(1)) )
    {}

    operator Vec2Type() const
    {
        Vec2Type tmp;
        tmp(0) = asFloat(x);
        tmp(1) = asFloat(y);
        return tmp;
    }

    static float asFloat(const half_float::detail::uint16 &src)
    {
        return half_float::detail::half2float(src);
    }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
    }

    half_float::detail::uint16 x, y;
};
#endif

#define _SHAPE_PREDICTOR drishti::ml::shape_predictor

// #############
// ### BOOST ###
// #############

#if DRISHTI_SERIALIZE_WITH_BOOST

DRISHTI_BEGIN_NAMESPACE(boost)
DRISHTI_BEGIN_NAMESPACE(serialization)
#include "drishti/ml/shape_predictor_archive.h"
DRISHTI_END_NAMESPACE(serialization) // namespace serialization
DRISHTI_END_NAMESPACE(boost) // namespace boost

BOOST_CLASS_IMPLEMENTATION(_SHAPE_PREDICTOR, boost::serialization::object_class_info);
BOOST_CLASS_TRACKING(_SHAPE_PREDICTOR, boost::serialization::track_always);
BOOST_CLASS_VERSION(_SHAPE_PREDICTOR, 4);

#endif // DRISHTI_SERIALIZE_WITH_BOOST

// ##############
// ### CEREAL ###
// ##############

#if DRISHTI_SERIALIZE_WITH_CEREAL
#include <cereal/cereal.hpp>
DRISHTI_BEGIN_NAMESPACE(cereal)
#include "drishti/ml/shape_predictor_archive.h"
DRISHTI_END_NAMESPACE(cereal)

CEREAL_CLASS_VERSION(_SHAPE_PREDICTOR, 4);
#endif // DRISHTI_SERIALIZE_WITH_CEREAL

#endif // __drishti_ml_shape_predictor_h__
