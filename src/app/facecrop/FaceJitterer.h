/*!
  @file   FaceJitterer
  @author David Hirvonen
  @brief  Face jitter utility to facilitate creation of training data.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __facecrop_FaceJitterer_h__
#define __facecrop_FaceJitterer_h__

#include "FaceSpecification.h"
#include "JitterParams.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "landmarks/FACE.h"

#include <opencv2/core.hpp>

#include <array>

using Landmark5 = std::array<cv::Point2f, 5>;

// Overload some operators for the landmark points for readable arithmetic (not performance critical):
inline Landmark5 operator *(const Landmark5 &src, double w)
{
    std::array<cv::Point2f, 5> dst;
    for(int i = 0; i < src.size(); i++)
    {
        dst[i] = src[i] * w;
    }
    return dst;
}

inline Landmark5 operator -(const Landmark5 &a, const Landmark5 &b)
{
    Landmark5 dst;
    for(int i = 0; i < a.size(); i++)
    {
        dst[i] = a[i] - b[i];
    }
    return dst;
}

inline Landmark5 operator +(const Landmark5 &a, const Landmark5 &b)
{
    Landmark5 dst;
    for(int i = 0; i < a.size(); i++)
    {
        dst[i] = a[i] + b[i];
    }
    return dst;
}

struct FaceWithLandmarks
{
    cv::Mat image;
    Landmark5 landmarks; // {eyeR, eyeL, nose, mouthR, mouthL}
    
    FaceWithLandmarks& operator =(const FaceWithLandmarks &src)
    {
        image = src.image;
        landmarks = src.landmarks;
        return (*this);
    }
    
    FaceWithLandmarks& operator +=(const FaceWithLandmarks &src)
    {
        image += src.image;
        for(int i = 0; i < src.landmarks.size(); i++)
        {
            landmarks[i] += src.landmarks[i];
        }
        return (*this);
    }
    
    void flop()
    {
        std::swap(landmarks[0], landmarks[1]); // swap eyes
        std::swap(landmarks[3], landmarks[4]); // swap mouth corners
    }
};

inline FaceWithLandmarks operator *(const FaceWithLandmarks &src, double w)
{
    FaceWithLandmarks dst;
    dst.image = src.image * w;
    dst.landmarks = src.landmarks * w;
    return dst;
}

struct FaceWithLandmarksMean : public FaceWithLandmarks
{
    void updateMean(const FaceWithLandmarks &src)
    {
        // Create CV_32FC3 sample:
        cv::Mat image1;
        src.image.convertTo(image1, CV_32FC3, 1.0/255.0);
        updateMeanFloat(image1, src.landmarks);
    }
    
    int count = 0;

protected:
    
    void updateMeanFloat(const cv::Mat &image1, const std::array<cv::Point2f, 5> &landmarks1)
    {
        count++;
        if(image.empty())
        {
            image = image1.clone();
            landmarks = landmarks1;
        }
        else
        {
            image += (image1 - image) * (1.0 / static_cast<double>(count));
            landmarks = landmarks + ((landmarks1 - landmarks) * (1.0f / static_cast<float>(count)));
        }
    }
};

class FaceJitterer
{
public:
    
    using Landmarks = std::vector<cv::Point2f>;
    
    FaceJitterer(const FACE::Table &table, const JitterParams &params, const FaceSpecification &face);
    FaceWithLandmarks operator()(const cv::Mat &image, const Landmarks &landmarks, bool doJitter, bool doNoise);
    void setDoPreview(bool value) { m_doPreview = value; }
    bool getDoPreview() const { return m_doPreview; }    

protected:
    
    static cv::Point2f mean(const Landmarks &landmarks, const std::vector<int> &index);
    
    const FACE::Table &m_table;
    const JitterParams &m_params;
    const FaceSpecification &m_face;
    std::array<cv::Point2f, 2> m_eyes;
    bool m_doPreview = false;
    
    cv::RNG m_rng;
};

#endif // __facecrop_FaceJitterer_h__
