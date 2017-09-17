/*! -*-c++-*-
  @file   MatP.cpp
  @author David Hirvonen
  @brief  Internal ACF namespace.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of an OpenCV cv::Mat planar format wrapper class.
*/

#include "MatP.h"

MatP::MatP(const MatP& src)
    : data(src.data)
    , planes(src.planes)
{
}

MatP::MatP(const cv::Mat& src)
{
    if (src.channels() == 1)
    {
        data = src;
        planes = { data };
    }
    else
    {
        create(src.size(), src.depth(), src.channels());
        planes.resize(src.channels());
        for (int i = 0; i < src.channels(); i++)
        {
            cv::extractChannel(src, planes[i], i);
        }
    }
}

MatP::MatP(const cv::Size& size, int depth, int channels, bool transpose)
{
    create(size, depth, channels, transpose);
}

MatP& MatP::operator=(const MatP& src)
{
    if (this != &src)
    {
        data = src.data;
        planes = src.planes;
    }
    return (*this);
}

void MatP::create(const cv::Size& size, int depth, int channels, bool transpose)
{
    //std::cout << size.height*channels  << " " << size.width << " " << depth << std::endl;
    if (transpose)
    {
        data.create(size.height, size.width * channels, depth);
        planes.resize(channels);
        cv::Rect roi({ 0, 0 }, size);
        for (int i = 0; i < channels; i++, roi.x += roi.width)
        {
            planes[i] = data(roi);
        }
    }
    else
    {
        data.create(size.height * channels, size.width, depth);
        planes.resize(channels);
        cv::Rect roi({ 0, 0 }, size);
        for (int i = 0; i < channels; i++, roi.y += roi.height)
        {
            planes[i] = data(roi);
        }
    }
}

void MatP::create(const cv::Size& size, int depth, int channels, void* pix, bool keep)
{
    data = cv::Mat(size.height * channels, size.width, depth, pix).clone(); // TODO: avoid clone
    planes.resize(channels);
    cv::Rect roi({ 0, 0 }, size);
    for (int i = 0; i < channels; i++, roi.y += roi.height)
    {
        planes[i] = data(roi);
    }
}

MatP MatP::operator()(const cv::Range& rows, const cv::Range& cols) const
{
    MatP view;
    for (auto& p : planes)
    {
        view.planes.push_back(p(rows, cols));
    }

    return view;
}

double sum(const MatP& src)
{
    double total = 0.0;
    for (auto& p : src)
    {
        total += cv::sum(p)[0];
    }

    return total;
}

int numel(const MatP& src)
{
    return src.channels() * src.size().area();
}

void resize(const MatP& src, MatP& dst, const cv::Size& size, int type)
{
    dst.create(size, src.depth(), src.channels());
    for (int i = 0; i < src.channels(); i++)
    {
        cv::resize(src[i], dst[i], size, 0.0, 0.0, type);
    }
}

void copyMakeBorder(const MatP& src, MatP& dst, int t, int b, int l, int r, int type)
{
    dst.get().resize(src.channels());
    for (int i = 0; i < src.channels(); i++)
    {
        cv::copyMakeBorder(src[i], dst[i], t, b, l, r, type);
    }
}
