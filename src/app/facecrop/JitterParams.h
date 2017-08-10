#ifndef __drishti_facecrop_JitterParams_h__
#define __drishti_facecrop_JitterParams_h__

#include <opencv2/core.hpp>
#include <cmath>

struct JitterParams
{
    JitterParams() {}

    struct Distribution
    {
        Distribution(bool isGaussian, const cv::Vec2f& params)
            : isGaussian(isGaussian)
            , params(params)
        {
        }

        float operator()(cv::RNG& rng) const
        {
            if (isGaussian)
            {
                return params[0] + rng.gaussian(params[1]);
            }
            else
            {
                return rng.uniform(params[0], params[1]);
            }
        }

        bool isGaussian; // else uniform
        cv::Vec2f params;
    };

    std::pair<cv::Matx33f, bool> operator()(cv::RNG& rng, const cv::Size& size, const cv::Point& tl = {}) const;
    std::pair<cv::Matx33f, bool> mirror(cv::RNG& rng, const cv::Size& size, const cv::Point& tl = {}) const;
    void mirror(const cv::Size& size, const cv::Point& tl = {}) const;
    cv::Matx33f scale(const cv::Size& size, const cv::Point& tl, float sx, float sy) const;

    float getGain(cv::RNG& rng) const;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version);

    /*
     * For multiplicative parameters the distribution is specified
     * as an exponential term, as in 2^n
     */

    // uniform distribution
    Distribution gainPower = { false, { -1.000f, +1.000f } };  // 2^n
    Distribution scalePower = { false, { -0.250f, +0.250f } }; // 2^n
    Distribution tx = { false, { -0.150f, +0.150f } };
    Distribution ty = { false, { -0.150f, +0.150f } };
    Distribution theta = { false, { static_cast<float>(-15.0 * M_PI / 180.0), static_cast<float>(+15.0f * M_PI / 180.0) } };

    float margin = 0.05f;
    float flop = 0.0f;                     // flop ratio
    cv::Point2f origin = { +0.5f, +0.5f }; // normalized origin
};

#endif // __drishti_facecrop_JitterParams_h__
