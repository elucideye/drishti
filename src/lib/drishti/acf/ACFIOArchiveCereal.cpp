#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/acf/ACFIOArchive.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cvmat_cereal.h"

#include <opencv2/opencv.hpp>

#if 0
DRISHTI_BEGIN_NAMESPACE(cv)

template<class Archive>
void save(Archive &ar, const cv::Mat &m, const std::uint32_t BOOST_ATTRIBUTE_UNUSED version)
{
    size_t elem_size = m.elemSize();
    size_t elem_type = m.type();
    ar & m.cols;
    ar & m.rows;
    ar & elem_size; // full resolution type
    ar & elem_type;
    
    std::vector<std::uint16_t> data;
    switch(elem_type)
    {
        case CV_32F: drishti::acf::float2half(m, data); break;
        case CV_32S: drishti::acf::transform32Sto16U(m, data); break;
        default : assert(false);
    }
    ar & data;
}

/** Serialization support for cv::Mat */
template<class Archive>
void load(Archive &ar, cv::Mat &m, const std::uint32_t BOOST_ATTRIBUTE_UNUSED version)
{
    int    cols, rows;
    size_t elem_size, elem_type;
    ar & cols;
    ar & rows;
    ar & elem_size; // full resolution type
    ar & elem_type;
    
    m.create(rows, cols, int(elem_type));
    
    std::vector<std::uint16_t> data;
    ar & data;
    
    switch(elem_type)
    {
        case CV_32F: drishti::acf::half2float(rows, cols, data, m); break;
        case CV_32S: drishti::acf::transform16Uto32S(rows, cols, data, m); break;
        default : assert(false);
    }
}

DRISHTI_END_NAMESPACE(cv)
#else
#include "drishti/core/drishti_cvmat_cereal.h"
#endif

DRISHTI_ACF_NAMESPACE_BEGIN

// ##################################################################
// #################### PortableBinary[IO]Archive ###################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef cereal::PortableBinaryOutputArchive3 OArchive;
template void Detector::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::Boost::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::Boost::Tree::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::Jitter::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::Pyramid::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::Nms::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::Pyramid::Chns::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::Pyramid::Chns::Color::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<OArchive>(OArchive & ar, const std::uint32_t);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<OArchive>(OArchive & ar, const std::uint32_t);
#endif

typedef  cereal::PortableBinaryInputArchive3 IArchive;
template void Detector::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::Boost::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::Boost::Tree::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::Jitter::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::Pyramid::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::Nms::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<IArchive>(IArchive & ar, const std::uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<IArchive>(IArchive & ar, const std::uint32_t version);

DRISHTI_ACF_NAMESPACE_END
