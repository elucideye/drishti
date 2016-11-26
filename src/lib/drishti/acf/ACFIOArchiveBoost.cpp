#include "drishti/acf/ACFIOArchive.h"

#include "drishti/core/drishti_core.h"

#include <opencv2/opencv.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>


#if 0
BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)
// Macros fix some IDE formatting
DRISHTI_BEGIN_NAMESPACE(boost)
DRISHTI_BEGIN_NAMESPACE(serialization)

/** Serialization support for cv::Mat */
template<class Archive>
void save(Archive &ar, const cv::Mat &m, const std::uint32_t BOOST_ATTRIBUTE_UNUSED version)
{
    size_t elem_size = m.elemSize();
    size_t elem_type = m.type();
    ar & m.cols;
    ar & m.rows;
    ar & elem_size;
    ar & elem_type;
    
    std::vector<std::uint16_t> data(m.total());
    switch(elem_type)
    {
        case CV_32F: drishti::acf::float2half(m, data); break;
        case CV_32S: drishti::acf::transform32Sto16U(m, data); break;
        default : assert(false);
    }
    
    // Compare size of array vs vector:
    ar & boost::serialization::make_array(data.data(), data.size());
}

/** Serialization support for cv::Mat */
template<class Archive>
void load(Archive &ar, cv::Mat &m, const std::uint32_t BOOST_ATTRIBUTE_UNUSED version)
{
    int    cols, rows;
    size_t elem_size, elem_type;
    ar & cols;
    ar & rows;
    ar & elem_size;
    ar & elem_type;

    m.create(rows, cols, int(elem_type));

    std::vector<std::uint16_t> data(m.total());
    ar & boost::serialization::make_array(data.data(), data.size());
    switch(elem_type)
    {
        case CV_32F: drishti::acf::half2float(rows, cols, data, m); break;
        case CV_32S: drishti::acf::transform16Uto32S(rows, cols, data, m); break;
        default : assert(false);
    }
}
DRISHTI_END_NAMESPACE(serialization)
DRISHTI_END_NAMESPACE(boost)
#else
#include "drishti/core/drishti_cvmat_boost.h"
#endif

//----

DRISHTI_ACF_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
template void Detector::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::Boost::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::Boost::Tree::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::Jitter::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::Nms::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<OArchive>(OArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<OArchive>(OArchive & ar, const uint32_t version);
#endif

typedef portable_binary_iarchive IArchive;
template void Detector::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::Boost::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::Boost::Tree::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::Jitter::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::Nms::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<IArchive>(IArchive & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<IArchive>(IArchive & ar, const uint32_t version);

#if DRISHTI_USE_TEXT_ARCHIVES

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

typedef boost::archive::text_oarchive OArchiveTXT;
template void Detector::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Boost::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Boost::Tree::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Jitter::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Nms::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<OArchiveTXT>(OArchiveTXT & ar, const uint32_t version);

typedef boost::archive::text_iarchive IArchiveTXT;
template void Detector::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Boost::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Boost::Tree::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Jitter::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Nms::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::Color::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradMag::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);
template void Detector::Options::Pyramid::Chns::GradHist::serialize<IArchiveTXT>(IArchiveTXT & ar, const uint32_t version);

#endif

DRISHTI_ACF_NAMESPACE_END


