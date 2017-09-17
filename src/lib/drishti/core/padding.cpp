/*! -*-c++-*-
  @file   padding.cpp
  @author David Hirvonen
  @brief  Implementation of simple aspect ratio aware padding routines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/drishti_core.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/videostab.hpp>

#include <iostream>

DRISHTI_CORE_NAMESPACE_BEGIN

cv::Point padWithInpainting(const cv::Mat& image, cv::Mat& padded, int top, int bottom, int left, int right, bool inPaint)
{
    if (top <= 0 && bottom <= 0 && left <= 0 && right <= 0)
    {
        padded = image;
        return cv::Point(0, 0);
    }

    cv::Mat mask;
    cv::copyMakeBorder(image, padded, std::max(top, 0), std::max(bottom, 0), std::max(left, 0), std::max(right, 0), cv::BORDER_CONSTANT);
    mask = cv::Mat::zeros(padded.size(), CV_8UC1);
    mask({ { std::max(left, 0), std::max(top, 0) }, image.size() }).setTo(255);

    cv::videostab::ColorAverageInpainter inpainter;
    //cv::videostab::ColorInpainter inpainter(cv::INPAINT_TELEA, std::min(image.cols, image.rows)/16.0);
    inpainter.inpaint(0, padded, mask);

    return cv::Point(left, top);
}

cv::Point padToAspectRatio(const cv::Mat& image, cv::Mat& padded, double aspectRatio, bool inPaint)
{
    CV_Assert(image.channels() == 3);

    int top = 0, left = 0, bottom = 0, right = 0;
    if (double(image.cols) / image.rows > aspectRatio)
    {
        int padding = int(double(image.cols) / aspectRatio + 0.5) - image.rows;
        top = padding / 2;
        bottom = padding - top;
    }
    else
    {
        int padding = int(double(image.rows) * aspectRatio + 0.5) - image.cols;
        left = padding / 2;
        right = padding - left;
    }

    return padWithInpainting(image, padded, top, bottom, left, right, inPaint);
}

cv::Point padToWidthUsingAspectRatio(const cv::Mat& canvas, cv::Mat& padded, int width, double aspectRatio, bool inPaint)
{
    int height = double(width) / aspectRatio;
    int top = 0, left = 0, bottom = 0, right = 0;

    int hPad = (width - canvas.cols);
    left = hPad / 2;
    right = hPad - left;

    cv::Point tl;
    if (height > canvas.rows)
    {
        int vPad = (height - canvas.rows);
        top = vPad / 2;
        bottom = vPad - top;
        tl = padWithInpainting(canvas, padded, top, bottom, left, right, inPaint);
    }
    else
    {
        int vCrop = (canvas.rows - height);
        top = vCrop / 2;
        bottom = vCrop - top;
        tl = padWithInpainting(canvas, padded, top, bottom, left, right, inPaint);
    }

    if (left < 0 || right < 0 || top < 0 || bottom < 0)
    {
        padded = padded(cv::Rect(std::max(-left, 0), std::max(-top, 0), width, height));
    }

    return tl;
}

cv::Mat borderMask(const cv::Mat& image)
{
    cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
    for (int y = 0; y < image.rows; y++)
    {
        uint8_t* pm = &mask.at<uint8_t>(y, 0);
        const cv::Vec3b* ptr = &image.at<cv::Vec3b>(y, 0);
        cv::Vec3b pix = ptr[0];
        for (int x = 0; (x < image.cols / 2) && (ptr[0] == pix); ++x, ++ptr, ++pm)
        {
            pm[0] = 255;
        }

        pm = &mask.at<uint8_t>(y, image.cols - 1);
        ptr = &image.at<cv::Vec3b>(y, image.cols - 1);
        pix = ptr[0];
        for (int x = image.cols - 1; (x > image.cols / 2) && (pix == ptr[0]); x--, ptr--, pm--)
        {
            pm[0] = 255;
        }
    }

    return mask;
}

void inpaintBorder(const cv::Mat& input, cv::Mat& output, cv::Mat& mask)
{
    cv::Mat black;
    cv::reduce(input.reshape(1, input.size().area()), black, 1, CV_REDUCE_MAX);
    black = black.reshape(1, input.rows);

    mask = cv::Mat1b::zeros(input.size());
    for (int y = 0; y < black.rows; y++)
    {
        int xl = 0, xr = black.cols - 1;
        const uint8_t *pli = black.ptr<uint8_t>(y), *pri = pli + (black.cols - 1);
        uint8_t *plo = mask.ptr<uint8_t>(y), *pro = plo + (black.cols - 1);
        for (; xl < black.cols; xl++, pli++, plo++)
        {
            if (pli[0])
            {
                break;
            }
            plo[0] = 255;
        }
        for (; xr > xl; xr--, pri--, pro--)
        {
            if (pri[0])
            {
                break;
            }
            pro[0] = 255;
        }
    }

    for (int x = 0; x < black.cols; x++)
    {
        const int step = black.step1();
        int yt = 0, yb = black.rows - 1;
        const uint8_t *pti = black.ptr<uint8_t>(0) + x, *pbi = pti + (black.rows - 1) * step;
        uint8_t *pto = mask.ptr<uint8_t>(0) + x, *pbo = pto + (black.rows - 1) * step;
        for (; yt < black.rows; yt++, pti += step, pto += step)
        {
            if (pti[0])
            {
                break;
            }
            pto[0] = 255;
        }
        for (; yb > yt; yb--, pbi -= step, pbo -= step)
        {
            if (pbi[0])
            {
                break;
            }
            pbo[0] = 255;
        }
    }

    cv::dilate(mask, mask, {}, { -1, -1 }, 3);

    cv::Scalar mu = cv::mean(input, ~mask);
    output = input.clone();
    output.setTo(mu, mask);
}

DRISHTI_CORE_NAMESPACE_END
