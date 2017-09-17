/*! -*-c++-*-
  @file   test-hessian-cpu.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Approximate CPU simulatino of ogles_gpgpu::HessianProc

*/

///////////////////////////

// Iyy
//  1     2     1
// -2    -4    -2
//  1     2     1

// Ixx
//  1    -2     1
//  2    -4     2
//  1    -2     1

// Ixy
//  1     0    -1
//  0     0     0
// -1     0     1

/*
 * The Mat4b interfaces are added to specifically enforce
 * 8 bit output limitations in standard OpenGL ES 2.0 shader
 * processing.
 */

#include "test-hessian-cpu.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <array>

//static void myshow(const std::string &name, const cv::Mat &I)
//{
//    cv::imshow(name, I);
//}

void hessian3x3(const cv::Mat4f& Iin, cv::Mat4f& Iout)
{
    cv::Matx33f dyy(1.f, 2.f, 1.f, -2.f, -4.f, -2.f, 1.f, 2.f, 1.f);
    cv::Matx33f dxx = dyy.t();
    cv::Matx33f dxy(+1.f, 0.f, -1.f, 0.f, 0.f, 0.f, -1.f, 0.f, +1.f);

    dxx *= (1.0f / 16.f);
    dxy *= (1.0f / 4.f);
    dyy *= (1.0f / 16.f);

    cv::Mat4f Ixx(Iin.size()), Iyy(Iin.size()), Ixy(Iin.size());
    cv::filter2D(Iin, Ixx, CV_32FC4, cv::Mat1f(dxx), { -1, -1 }, 0, cv::BORDER_DEFAULT);
    cv::filter2D(Iin, Iyy, CV_32FC4, cv::Mat1f(dyy), { -1, -1 }, 0, cv::BORDER_DEFAULT);
    cv::filter2D(Iin, Ixy, CV_32FC4, cv::Mat1f(dxy), { -1, -1 }, 0, cv::BORDER_DEFAULT);

    Iout.create(Iin.size());
    Iout = Ixx.mul(Iyy) - (Ixy.mul(Ixy));

    cv::Mat1f Rxx, Ryy, Rout;
    cv::extractChannel(Ixx, Rxx, 0);
    cv::extractChannel(Iyy, Ryy, 0);
    cv::extractChannel(Iout, Rout, 0);
    cv::Mat1b mask = ((Rxx + Ryy) > 0.f) | (Rout < 0.f);

    Iout.setTo(0, mask);
    Rout.setTo(0, mask);

    { // Set alpha channel to 1
        std::vector<cv::Mat> channels;
        cv::split(Iout, channels);
        channels.back() = cv::Scalar::all(1.0);
        cv::merge(channels, Iout);
    }
}

void hessian3x3(const cv::Mat4b& Iin, cv::Mat4b& Iout, float scale)
{
    cv::Mat4f Iinf(Iin.size()), Ioutf(Iin.size());

    Iin.convertTo(Iinf, CV_32FC4, 1.f / 255.f);
    hessian3x3(Iinf, Ioutf);

    Iout.create(Iin.size());
    Ioutf.convertTo(Iout, CV_8UC4, 255.f * scale);
}

void gaussian3x3(const cv::Mat4f& Iin, cv::Mat4f& Iout)
{
    cv::Matx33f g(1.f, 2.f, 1.f, 2.f, 4.f, 2.f, 1.f, 2.f, 1.f);
    g *= (1.f / 16.f);

    Iout.create(Iin.size());
    cv::filter2D(Iin, Iout, CV_32FC4, cv::Mat1f(g), { -1, -1 }, 0, cv::BORDER_DEFAULT);
}

void gaussian3x3(const cv::Mat4b& Iin, cv::Mat4b& Iout, float scale)
{
    cv::Mat4f Iinf(Iin.size()), Ioutf(Iin.size());

    Iin.convertTo(Iinf, CV_32FC4, 1.f / 255.f);
    gaussian3x3(Iinf, Ioutf);

    Iout.create(Iin.size());
    Ioutf.convertTo(Iout, CV_8UC4, 255.f * scale);
}

#define DRISHTI_HCI_TEST_HESSIAN_WITH_FLOAT 0

void blobs3x3(const cv::Mat4b& Iin, cv::Mat4b& Iout, float alpha1, float alpha2)
{
#if DRISHTI_HCI_TEST_HESSIAN_WITH_FLOAT
    {
        // Use float precision:
        cv::Mat4f Iinf(Iin.size()), Ismoothf(Iin.size()), Ioutf(Iin.size());
        Iin.convertTo(Iinf, CV_32FC4, 1.f / 255.f);

        //gaussian3x3(Iinf, Ismoothf);
        // This is closer to the large sigma separable blur used in ogles_gpgpu
        cv::GaussianBlur(Iinf, Ismoothf, { 11, 11 }, 1.5);
        hessian3x3(Ismoothf * alpha1, Ioutf);
        Ioutf.convertTo(Iout, CV_8UC4, 255.f * alpha2);

        { // Set alpha channel to 1
            std::vector<cv::Mat> channels;
            cv::split(Iout, channels);
            channels.back() = cv::Scalar::all(255);
            cv::merge(channels, Iout);
        }
    }
#else
    {
        //cv::Mat4b Ismooth1(Iin.size()), Ismooth2(Iin.size());
        //gaussian3x3(Iin, Ismooth1, alpha1);
        //gaussian3x3(Iin, Ismooth2, alpha1);

        cv::Mat4b Ismooth(Iin.size());
        cv::GaussianBlur(Iin, Ismooth, { 11, 11 }, 1.5);
        hessian3x3(Ismooth, Iout, alpha2);
    }
#endif
}

void fir3(const std::array<cv::Mat4f, 3>& Iin, cv::Mat4f& Iout, const std::array<float, 3>& weights)
{
    Iout.create(Iin[0].size());
    Iout = (Iin[0] * weights[0]) + (Iin[1] * weights[1]) + (Iin[2] * weights[2]);
}

void fir3(const std::array<cv::Mat4f, 3>& Iin, cv::Mat4b& Iout, const std::array<float, 3>& weights, float alpha, float beta)
{
    std::array<cv::Mat4f, 3> Iinf;
    for (int i = 0; i < 3; i++)
    {
        Iinf[i].create(Iin[i].size());
        Iin[i].convertTo(Iinf[i], CV_32FC4, 1.f / 255.f);
    }

    cv::Mat4f Ioutf;
    fir3(Iinf, Ioutf, weights);

    Iout.create(Iin[0].size());
    Ioutf.convertTo(Iout, CV_8UC4, 255.f * alpha, beta);
}
