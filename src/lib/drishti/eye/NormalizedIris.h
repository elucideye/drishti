/*!
  @file   NormalizedIris.h
  @author David Hirvonen
  @brief  Declaration of ellipso-polar iris normalization.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishtisdk__NormalizedIris__
#define __drishtisdk__NormalizedIris__

#include <stdio.h>

#include "drishti/eye/drishti_eye.h"
#include "drishti/eye/Eye.h"

DRISHTI_EYE_BEGIN

class NormalizedIris
{
public:

    NormalizedIris() {}
    NormalizedIris(const cv::Mat &image, const cv::Mat &mask, const cv::Rect &roi) :  roi(roi), image(image), mask(mask) {}

    const cv::Mat getImage() const
    {
        return image(roi);
    }
    const cv::Mat getMask() const
    {
        return mask(roi);
    }

    const cv::Rect & getRoi() const
    {
        return roi;
    }
    cv::Rect & getRoi()
    {
        return roi;
    }

    const cv::Mat &getPaddedImage() const
    {
        return image;
    }
    cv::Mat &getPaddedImage()
    {
        return image;
    }

    const cv::Mat &getPaddedMask() const
    {
        return mask;
    }
    cv::Mat &getPaddedMask()
    {
        return mask;
    }

    NormalizedIris rotate(int x) const;

    void oblp();

    static void rotate(const cv::Mat &src, cv::Mat &dst, int x);

protected:

    cv::Rect roi;
    cv::Mat image;
    cv::Mat mask;

    std::vector< cv::Mat > features;

};

struct NormalizedCurve
{
    NormalizedCurve() {}

    const cv::Rect & getRoi() const
    {
        return roi;
    }
    cv::Rect & getRoi()
    {
        return roi;
    }

    const cv::Mat &getImage() const
    {
        return image;
    }
    cv::Mat &getImage()
    {
        return image;
    }

    const cv::Mat &getMask() const
    {
        return mask;
    }
    cv::Mat &getMask()
    {
        return mask;
    }

protected:

    cv::Rect roi;
    cv::Mat image;
    cv::Mat mask;
};

DRISHTI_EYE_END

#endif /* defined(__drishtisdk__NormalizedIris__) */
