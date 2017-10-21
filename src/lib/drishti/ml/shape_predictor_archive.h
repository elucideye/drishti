// Warning: This file is used as a template and is included/inlined for each archive library
// so the include guards should not be used here.

#ifndef shape_predictor_archive_h
#define shape_predictor_archive_h

#include "drishti/ml/shape_predictor.h"
#include "drishti/core/ThrowAssert.h"

DRISHTI_BEGIN_NAMESPACE(cereal)

template <class Archive>
void serialize(Archive& ar, drishti::ml::fshape& g, const unsigned int version)
{
    if (Archive::is_loading::value)
    {
        // #### READING
        std::vector<float> values;
#if DRISHTI_DLIB_DO_HALF
        std::vector<half_float::detail::uint16> half;
        ar& half;
        values.resize(half.size());
        std::transform(half.begin(), half.end(), values.begin(), [](half_float::detail::uint16 a) {
            return half_float::detail::half2float(a);
        });
#else
        ar& values;
#endif
        g.set_size(values.size(), 1);
        memcpy(&g(0), &values[0], sizeof(float) * values.size());
    }
    else
    {
        // #### WRITING
        std::vector<float> values(&g(0), &g(0) + g.size());
#if DRISHTI_DLIB_DO_HALF
        std::vector<half_float::detail::uint16> half(values.size());
        std::transform(values.begin(), values.end(), half.begin(), [](float a) {
            return half_float::detail::float2half<std::round_to_nearest>(a);
        });
        ar& half;
#else
        ar& values;
#endif
    }
}

template <class Archive>
void serialize(Archive& ar, dlib::vector<float, 2>& g, const unsigned int version)
{
#if DRISHTI_DLIB_DO_HALF
    half_float::detail::uint16 half1, half2;
    if (Archive::is_loading::value)
    {
        ar& half1;
        ar& half2;
        g(0) = half_float::detail::half2float(half1);
        g(1) = half_float::detail::half2float(half2);
    }
    else
    {
        half1 = half_float::detail::float2half<std::round_to_nearest>(g(0));
        half2 = half_float::detail::float2half<std::round_to_nearest>(g(1));
        ar& half1;
        ar& half2;
    }
#else
    ar& g(0);
    ar& g(1);
#endif
}

template <class Archive>
void serialize(Archive& ar, drishti::ml::impl::split_feature& g, const unsigned int version)
{
    ar& g.idx1;
    ar& g.idx2;

#if DRISHTI_DLIB_DO_HALF
    half_float::detail::uint16 thresh;
    if (Archive::is_loading::value)
    {
        ar& thresh;
        g.thresh = half_float::detail::half2float(thresh);
    }
    else
    {
        thresh = half_float::detail::float2half<std::round_to_nearest>(g.thresh);
        ar& thresh;
    }
#else
    ar& g.thresh;
#endif
}

// 0
// 1 2
// 3 4 5 6               = (n+1)/2-1
// 7 8 9 10 11 12 13 14  = n - ((n+1)/2-1)

template <class Archive>
void serialize(Archive& ar, drishti::ml::impl::regression_tree& g, const unsigned int version)
{
    ar& g.splits;
    ar& g.leaf_values;

    if (Archive::is_loading::value)
    {
        g.leaf_values_16.resize(g.leaf_values.size());
        for (int i = 0; i < g.leaf_values.size(); i++)
        {
            g.leaf_values_16[i].set_size(g.leaf_values[i].size());

            const float* pSrc = &g.leaf_values[i](0, 0);
            int16_t* pDst = &g.leaf_values_16[i](0, 0);
            drishti::core::convertFixedPoint(pSrc, pDst, int(g.leaf_values[i].size()), FIXED_PRECISION);
        }
    }
}

template <class Archive>
void serialize(Archive& ar, drishti::ml::shape_predictor& sp, const unsigned int version)
{
    drishti_throw_assert(version == 4, "Incorrect shape_predictor archive format, please update models");

    drishti::ml::fshape& initial_shape = sp.initial_shape;
    std::vector<std::vector<RTType>>& forests = sp.forests;
    std::vector<std::vector<unsigned short>>& anchor_idx = sp.anchor_idx;
    std::vector<std::vector<Vec2Type>>& deltas = sp.deltas;

    // Without forests a 2.3 MB compressed archive drops to 48K//
    ar& initial_shape;
    ar& forests;
    ar& anchor_idx;

#if DRISHTI_DLIB_DO_HALF
    std::vector<std::vector<PointHalf>> deltas_;
    if (Archive::is_loading::value)
    {
        ar& deltas_;
        drishti::ml::copy(deltas_, deltas);
    }
    else
    {
        drishti::ml::copy(deltas, deltas_);
        ar& deltas_;
    }
#else
    ar& deltas;
#endif

    ar& sp.m_pca;
    ar& sp.m_npd;
    ar& sp.m_do_affine;
    ar& sp.m_ellipse_count;
    ar& sp.interpolated_features;
}

DRISHTI_END_NAMESPACE(cereal)

#include <cereal/cereal.hpp>
CEREAL_CLASS_VERSION(drishti::ml::shape_predictor, 4);

#endif /* shape_predictor_archive_h */
