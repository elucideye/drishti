#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/acf/ACFIOArchive.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cvmat_cereal.h"

#include <opencv2/opencv.hpp>

DRISHTI_ACF_NAMESPACE_BEGIN

// ##################################################################
// #################### PortableBinary[IO]Archive ###################
// ##################################################################

#if DRISHTI_BUILD_CEREAL_OUTPUT_ARCHIVES
typedef cereal::PortableBinaryOutputArchive OArchive;
template void Detector::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::Boost::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::Boost::Tree::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::Jitter::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::Pyramid::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::Nms::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::Pyramid::Chns::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::Pyramid::Chns::Color::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<OArchive>(OArchive& ar, const std::uint32_t);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<OArchive>(OArchive& ar, const std::uint32_t);
#endif

typedef cereal::PortableBinaryInputArchive IArchive;
template void Detector::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::Boost::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::Boost::Tree::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::Jitter::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::Pyramid::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::Nms::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<IArchive>(IArchive& ar, const std::uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<IArchive>(IArchive& ar, const std::uint32_t version);

DRISHTI_ACF_NAMESPACE_END
