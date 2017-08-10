#include "JitterParams.h"
#include "FaceSpecification.h"

// http://uscilab.github.io/cereal/serialization_archives.html
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cv_cereal.h"

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>

template <class Archive>
void serialize(Archive& ar, JitterParams::Distribution& d, const unsigned int version)
{
    ar& GENERIC_NVP("isGaussian", d.isGaussian);
    ar& GENERIC_NVP("params", d.params);
}

std::pair<cv::Matx33f, bool> JitterParams::operator()(cv::RNG& rng, const cv::Size& size, const cv::Point& tl) const
{
    float signum = 1.f;
    bool doMirror = false;
    if ((flop > 0.f) && (rng.uniform(0.f, 1.f) < flop))
    {
        signum = -1.f;
        doMirror = true;
    }

    // denormalize x,y wrt unpadded size.{width,height}, but apply offset for centering
    const float x = tx(rng) * size.width;
    const float y = ty(rng) * size.height;
    const float t = theta(rng) * M_PI / 180.0;
    const float s = std::pow(2.0, scalePower(rng));
    const cv::Point2f center(tl.x + (size.width / 2), tl.y + (size.height / 2));

    const cv::Matx33f R = transformation::rotate(t);
    const cv::Matx33f S = transformation::scale(s * signum, s);
    const cv::Matx33f T = transformation::translate(x, y);
    const cv::Matx33f T1 = transformation::translate(-center);
    const cv::Matx33f T2 = transformation::translate(center);
    const cv::Matx33f H = T2 * T * R * S * T1;

    return std::make_pair(H, doMirror);
}

std::pair<cv::Matx33f, bool> JitterParams::mirror(cv::RNG& rng, const cv::Size& size, const cv::Point& tl) const
{
    if ((flop > 0.f) && (rng.uniform(0.f, 1.f) < flop))
    {
        return std::make_pair(scale(size, tl, -1.f, +1.f), true);
    }
    else
    {
        return std::make_pair(scale(size, tl, +1.f, +1.f), false);
    }
}

cv::Matx33f JitterParams::scale(const cv::Size& size, const cv::Point& tl, float sx, float sy) const
{
    const cv::Point2f center(tl.x + (size.width / 2), tl.y + (size.height / 2));
    const cv::Matx33f S = transformation::scale(sx, sy);
    const cv::Matx33f T1 = transformation::translate(-center);
    const cv::Matx33f T2 = transformation::translate(center);
    const cv::Matx33f H = T2 * S * T1;

    return H;
}

float JitterParams::getGain(cv::RNG& rng) const
{
    return std::pow(2.0, gainPower(rng));
}

template <class Archive>
void JitterParams::serialize(Archive& ar, const unsigned int version)
{
    ar& GENERIC_NVP("gainPower", gainPower);
    ar& GENERIC_NVP("scalePower", scalePower);
    ar& GENERIC_NVP("tx", tx);
    ar& GENERIC_NVP("ty", ty);
    ar& GENERIC_NVP("theta", theta);
    ar& GENERIC_NVP("flop", flop);
    ar& GENERIC_NVP("margin", margin);
    ar& GENERIC_NVP("origin", origin);
}

typedef cereal::JSONOutputArchive OArchiveJSON;
template void JitterParams::serialize<OArchiveJSON>(OArchiveJSON& ar, const unsigned int);

typedef cereal::JSONInputArchive IArchiveJSON;
template void JitterParams::serialize<IArchiveJSON>(IArchiveJSON& ar, const unsigned int);
