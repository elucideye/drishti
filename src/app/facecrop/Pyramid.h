/*! -*-c++-*-
  @file   Pyramid.h
  @author David Hirvonen
  @brief  Gaussian and laplacian pyramid class + blending.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_facecrop_Pyramid_h__
#define __drishti_facecrop_Pyramid_h__

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// https://github.com/t-suzuki/laplacian_pyramid_blend_test/blob/master/lp_blend.cpp
bool gaussianPyramid(const cv::Mat& src, std::vector<cv::Mat>& result, int level);
bool laplacianPyramid(const cv::Mat& src, std::vector<cv::Mat>& result, int level);
bool inverseLaplacianPyramid(std::vector<cv::Mat>& src, cv::Mat& result);
cv::Mat blend(const cv::Mat& im1, const cv::Mat& im2, const cv::Mat& mask, int level);

#endif // __drishti_facecrop_Pyramid_h__
