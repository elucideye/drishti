/*!
  @file   ACFIO.h
  @author David Hirvonen (C++ implementation)
  @brief  Declaration of serialization routines for ACF.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_acf_ACFIOArchive_h__
#define __drishti_acf_ACFIOArchive_h__

#include "drishti/core/drishti_core.h"
#include "drishti/acf/drishti_acf.h"
#include "drishti/acf/ACF.h"
#include "drishti/acf/ACFField.h"

#include <opencv2/core/core.hpp>

// clang-format off
#if defined(ANDROID)
#  define HALF_ENABLE_CPP11_CMATH 0
#endif
// clang-format on

#define DRISHTI_ACF_DO_HALF 1

// clang-format off
#if DRISHTI_ACF_DO_HALF
#  include "half/half.hpp"
#endif
// clang-format on

#include <iomanip>

DRISHTI_BEGIN_NAMESPACE(cv)

template <class Archive>
void serialize(Archive& ar, cv::Rect& rect, const uint32_t version)
{
    ar& rect.x;
    ar& rect.y;
    ar& rect.width;
    ar& rect.height;
}

template <class Archive>
void serialize(Archive& ar, cv::Size& size, const uint32_t version)
{
    ar& size.width;
    ar& size.height;
}

template <class Archive>
void serialize(Archive& ar, cv::Size2f& size, const uint32_t version)
{
    ar& size.width;
    ar& size.height;
}

template <class Archive>
void serialize(Archive& ar, cv::Point2f& p, const uint32_t version)
{
    ar& p.x;
    ar& p.y;
}

DRISHTI_END_NAMESPACE(cv)

DRISHTI_ACF_NAMESPACE_BEGIN

// ### Boost ####

// Boost serialization:
template <class Archive>
void Detector::serialize(Archive& ar, const unsigned int version)
{
    ar& clf;
    ar& opts;
}

template <class Archive>
void Detector::Classifier::serialize(Archive& ar, const std::uint32_t version)
{
    ar& fids;    // cv::Mat_<int>
    ar& thrs;    // cv::Mat_<float>
    ar& child;   // cv::Mat_<int>
    ar& hs;      // cv::Mat_<float>
    ar& weights; // cv::Mat_<float>
    ar& depth;   // cv::Mat_<int>

    ar& errs;
    ar& losses;
    ar& treeDepth;

    if (Archive::is_loading::value)
    {
        thrsU8 = thrs * 255.0; // precompute uint8_t thresholds
    }
}

template <class Archive>
void Detector::Options::serialize(Archive& ar, const unsigned int version)
{
    ar& pPyramid; // *
    ar& modelDs;
    ar& modelDsPad;
    ar& pNms; // *
    ar& stride;
    ar& cascThr;
    ar& cascCal;
    ar& nWeak;
    ar& pBoost; // *

    // === TRAINING ===
    ar& posGtDir;
    ar& posImgDir;
    ar& negImgDir;
    ar& posWinDir;
    ar& negWinDir;
    ar& nPos;
    ar& nNeg;
    ar& nPerNeg;
    ar& nAccNeg;
    ar& pJitter;
    ar& winsSave;
}

template <class Archive>
void Detector::Options::Boost::serialize(Archive& ar, const uint32_t version)
{
    ar& pTree;
    ar& nWeak;
    ar& discrete;
    ar& verbose;
}

template <class Archive>
void Detector::Options::Boost::Tree::serialize(Archive& ar, const uint32_t version)
{
    ar& nBins;
    ar& maxDepth;
    ar& minWeight;
    ar& fracFtrs;
    ar& nThreads;
}

template <class Archive>
void Detector::Options::Pyramid::serialize(Archive& ar, const uint32_t version)
{
    ar& pChns; // *
    ar& nPerOct;
    ar& nOctUp;
    ar& nApprox;
    ar& lambdas;
    ar& pad;
    ar& minDs;
    ar& smooth;
    ar& concat;
    ar& complete;
}

template <class Archive>
void Detector::Options::Nms::serialize(Archive& ar, const uint32_t version)
{
    ar& type;
    ar& overlap;
    ar& ovrDnm;
}

template <class Archive>
void Detector::Options::Pyramid::Chns::serialize(Archive& ar, const uint32_t version)
{
    ar& shrink;
    ar& complete;
    ar& pColor;    // *
    ar& pGradMag;  // *
    ar& pGradHist; // *

    /* ar & pCustom */ // NA
}

template <class Archive>
void Detector::Options::Pyramid::Chns::Color::serialize(Archive& ar, const uint32_t version)
{
    ar& enabled;
    ar& smooth;
    ar& colorSpace;
}

template <class Archive>
void Detector::Options::Pyramid::Chns::GradMag::serialize(Archive& ar, const uint32_t version)
{
    ar& enabled;
    ar& colorChn;
    ar& normRad;
    ar& normConst;
    ar& full;
}

template <class Archive>
void Detector::Options::Pyramid::Chns::GradHist::serialize(Archive& ar, const uint32_t version)
{
    ar& enabled;
    ar& binSize;
    ar& nOrients;
    ar& softBin;
    ar& useHog;
    ar& clipHog;
}

template <class Archive>
void Detector::Options::Jitter::serialize(Archive& ar, const uint32_t version)
{
    ar& flip;
}

// ###############################################
// ### CV_32F <=> half_float::detail::uint16   ###
// ################################################

inline void float2half(const cv::Mat& src, std::vector<half_float::detail::uint16>& dst)
{
    int i = 0;
    dst.resize(src.total());
    for (auto iter = src.begin<float>(); iter != src.end<float>(); iter++, i++)
    {
        dst[i] = half_float::detail::float2half<std::round_to_nearest>(*iter);
    }
}

inline void half2float(int rows, int cols, const std::vector<half_float::detail::uint16>& src, cv::Mat& dst)
{
    int i = 0;
    dst.create(rows, cols, CV_32F);
    for (auto iter = dst.begin<float>(); iter != dst.end<float>(); iter++, i++)
    {
        (*iter) = half_float::detail::half2float(src[i]);
    }
}

// ###############################################
// ### CV_32S <=> uint16_t                     ###
// ################################################

inline void transform32Sto16U(const cv::Mat& src, std::vector<uint16_t>& dst)
{
    int i = 0;
    dst.resize(src.total());
    for (auto iter = src.begin<int32_t>(); iter != src.end<int32_t>(); iter++, i++)
    {
        dst[i] = static_cast<uint16_t>(*iter);
    }
}

inline void transform16Uto32S(int rows, int cols, const std::vector<uint16_t>& src, cv::Mat& dst)
{
    int i = 0;
    dst.create(rows, cols, CV_32S);
    for (auto iter = dst.begin<int32_t>(); iter != dst.end<int32_t>(); iter++, i++)
    {
        (*iter) = static_cast<int32_t>(src[i]);
    }
}

DRISHTI_ACF_NAMESPACE_END

#endif //  __drishti_acf_ACFIOArchive_h__
