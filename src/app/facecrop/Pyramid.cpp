/*! -*-c++-*-
  @file   Pyramid.h
  @author David Hirvonen
  @brief  Gaussian and laplacian pyramid class + blending.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "Pyramid.h"

bool gaussianPyramid(const cv::Mat& src, std::vector<cv::Mat>& result, int level)
{
    result.clear();
    cv::Mat tmp = src;
    cv::Mat down;
    for (int i = 0; i < level; ++i)
    {
        result.push_back(tmp);
        cv::pyrDown(tmp, down, cv::Size((tmp.cols + 1) / 2, (tmp.rows + 1) / 2));
        tmp = down;
    }
    result.push_back(down);
    return true;
}

bool laplacianPyramid(const cv::Mat& src, std::vector<cv::Mat>& result, int level)
{
    result.clear();
    cv::Mat tmp = src;
    cv::Mat down, down_up;
    for (int i = 0; i < level; ++i)
    {
        cv::pyrDown(tmp, down, cv::Size((tmp.cols + 1) / 2, (tmp.rows + 1) / 2));
        cv::pyrUp(down, down_up, cv::Size(tmp.cols, tmp.rows));
        cv::Mat lap = tmp - down_up;
        result.push_back(lap);
        tmp = down;
    }
    result.push_back(down);
    return true;
}

bool inverseLaplacianPyramid(std::vector<cv::Mat>& src, cv::Mat& result)
{
    if (src.size() == 0)
    {
        return false;
    }

    cv::Mat up;
    cv::Mat reconstruct = src[src.size() - 1];
    for (unsigned int i = src.size() - 1; i >= 1; --i)
    {
        cv::pyrUp(reconstruct, up, cv::Size(src[i - 1].cols, src[i - 1].rows));
        reconstruct = src[i - 1] + up;
    }
    result = reconstruct;

    return true;
}

// https://github.com/t-suzuki/laplacian_pyramid_blend_test/blob/master/lp_blend.cpp
cv::Mat blend(const cv::Mat& im1, const cv::Mat& im2, const cv::Mat& mask, int level)
{
    cv::Mat blended;

    // reference:
    // http://graphics.cs.cmu.edu/courses/15-463/2005_fall/www/Lectures/Pyramids.pdf

    cv::Mat im1f, im2f, maskf;
    im1.convertTo(im1f, CV_32FC3, 1.0 / 255.0);
    im2.convertTo(im2f, CV_32FC3, 1.0 / 255.0);
    mask.convertTo(maskf, CV_32FC3, 1.0 / 255.0);

    std::vector<cv::Mat> lap1, lap2, gaussian_blend;
    if (!laplacianPyramid(im1f, lap1, level))
    {
        return blended;
    }
    if (!laplacianPyramid(im2f, lap2, level))
    {
        return blended;
    }
    if (!gaussianPyramid(maskf, gaussian_blend, level))
    {
        return blended;
    }

    std::vector<cv::Mat> lap_blended;
    for (int i = 0; i < level + 1; ++i)
    {
        cv::Mat res = lap1[i].mul(cv::Scalar(1.0f, 1.0f, 1.0f) - gaussian_blend[i]) + lap2[i].mul(gaussian_blend[i]);
        lap_blended.push_back(res);
    }

    if (!inverseLaplacianPyramid(lap_blended, blended))
    {
        return blended;
    }

    return blended;
}
