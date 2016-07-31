/*!
  @file   MatP.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Declaration of OpenCV cv::Mat planar format object.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __DRISHTI__MatP__
#define __DRISHTI__MatP__

#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class MatP
{
public:

    typedef cv::Mat value_type;

    MatP() {}
    MatP(const MatP &src);
    MatP(const cv::Mat &src);
    MatP(const cv::Size &size, int depth, int channels, bool transpose=false);

    MatP& operator=(const MatP &src);

    void create(const cv::Size &size, int depth, int channels, bool transpose=false);
    void create(const cv::Size &size, int depth, int channels, void *data, bool keep);

    std::vector<cv::Mat>::const_iterator begin() const
    {
        return planes.begin();
    }
    std::vector<cv::Mat>::const_iterator end() const
    {
        return planes.end();
    }

    std::vector<cv::Mat>::iterator begin()
    {
        return planes.begin();
    }
    std::vector<cv::Mat>::iterator end()
    {
        return planes.end();
    }

    const std::vector<cv::Mat> &get() const
    {
        return planes;
    }
    std::vector<cv::Mat> &get()
    {
        return planes;
    }

    const cv::Mat& base() const
    {
        return data;
    }
    cv::Mat& base()
    {
        return data;
    }

    MatP operator()(const cv::Range &rows, const cv::Range &cols) const;

    template <typename T> void setTo(T value)
    {
        data.setTo(value);
    }

    bool empty() const
    {
        return (planes.size() == 0 || planes[0].empty());
    }

    template <typename T> const T& at(int y, int x, int z=0) const
    {
        return planes[z].at<T>(y,x);
    }

    template <typename T> T& at(int y, int x, int z=0)
    {
        return planes[z].at<T>(y,x);
    }

    void resize(int n)
    {
        planes.resize(n);
    }

    int depth() const
    {
        return planes.size() ? planes.front().depth() : 0;
    }
    int rows() const
    {
        return planes.size() ? planes.front().rows : 0;
    }
    int cols() const
    {
        return planes.size() ? planes.front().cols : 0;
    }
    const cv::Size size() const
    {
        return planes.size() ? planes.front().size() : cv::Size(0,0);
    }
    const int channels() const
    {
        return int(planes.size());
    }
    cv::Mat &operator[](int i)
    {
        return planes[i];
    }
    const cv::Mat &operator[](int i) const
    {
        return planes[i];
    }

    cv::Mat &back()
    {
        return planes.back();
    }
    cv::Mat &front()
    {
        return planes.front();
    }
    void push_back(const cv::Mat &I)
    {
        planes.push_back(I);
    }
    void pop_back()
    {
        planes.pop_back();
    }

    uint8_t* ptr()
    {
        return data.ptr();
    }
    const uint8_t *ptr() const
    {
        return data.ptr() ;
    }

    template <typename T> T* ptr()
    {
        return data.ptr<T>();
    }
    template <typename T> const T* ptr() const
    {
        return data.ptr<T>();
    }

    MatP& operator*=(double ratio)
    {
        for(auto &p : planes)
        {
            p *= ratio;
        }
        return *this;
    }

    void swap(MatP & other) // the swap member function (should never fail!)
    {
        cv::swap(data, other.data);
        std::swap(planes, other.planes);
    }

protected:

    cv::Mat data; // (row*channels x col)
    std::vector<cv::Mat> planes;
};

void resize(const MatP &src, MatP &dst, const cv::Size &size= {}, int type=cv::INTER_LINEAR);
double sum(const MatP &src);
int numel(const MatP &src);
void copyMakeBorder(const MatP& src, MatP &dst, int t, int b, int l, int r, int type);


#endif /* defined(__DRISHTI__MatP__) */
