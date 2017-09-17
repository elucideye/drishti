/*! -*-c++-*-
  @file   test-hessian-cpu.h
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Approximate CPU simulatino of ogles_gpgpu::HessianProc

*/

#ifndef test_hessian_cpu_h
#define test_hessian_cpu_h 1

#include <opencv2/core.hpp>
#include <array>

void hessian3x3(const cv::Mat4f& Iin, cv::Mat4f& Iout);
void hessian3x3(const cv::Mat4b& Iin, cv::Mat4b& Iout, float scale);
void gaussian3x3(const cv::Mat4f& Iin, cv::Mat4f& Iout);
void gaussian3x3(const cv::Mat4b& Iin, cv::Mat4b& Iout, float scale);
void blobs3x3(const cv::Mat4b& Iin, cv::Mat4b& Iout, float alpha1 = 1.f, float alpha2 = 1.f);
void fir3(const std::array<cv::Mat4f, 3>& Iin, cv::Mat4f& Iout, const std::array<float, 3>& weights);
void fir3(const std::array<cv::Mat4f, 3>& Iin, cv::Mat4b& Iout, const std::array<float, 3>& weights, float alpha, float beta);

#endif // test_hessian_cpu_h
