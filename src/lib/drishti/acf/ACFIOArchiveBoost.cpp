#include "drishti/acf/ACFIOArchive.h"
#include "drishti/core/drishti_core.h"
#include "drishti/core/drishti_cvmat_boost.h"

#include <opencv2/opencv.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

//----

DRISHTI_ACF_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
template void Detector::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::Boost::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::Boost::Tree::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::Jitter::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::Nms::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<OArchive>(OArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<OArchive>(OArchive& ar, const uint32_t version);
#endif

typedef portable_binary_iarchive IArchive;
template void Detector::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::Boost::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::Boost::Tree::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::Jitter::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::Nms::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<IArchive>(IArchive& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<IArchive>(IArchive& ar, const uint32_t version);

#if DRISHTI_USE_TEXT_ARCHIVES

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

typedef boost::archive::text_oarchive OArchiveTXT;
template void Detector::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Boost::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Boost::Tree::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Jitter::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Nms::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<OArchiveTXT>(OArchiveTXT& ar, const uint32_t version);

typedef boost::archive::text_iarchive IArchiveTXT;
template void Detector::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Boost::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Boost::Tree::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Jitter::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Nms::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<IArchiveTXT>(IArchiveTXT& ar, const uint32_t version);

#endif

DRISHTI_ACF_NAMESPACE_END
