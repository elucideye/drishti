/*! -*-c++-*-
  @file   FaceJitterer
  @author David Hirvonen
  @brief  Face jitter utility to facilitate creation of training data.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_facecrop_FaceJitterer_h__
#define __drishti_facecrop_FaceJitterer_h__

#include "FaceSpecification.h"
#include "JitterParams.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "landmarks/FACE.h"

#include <opencv2/core.hpp>

#include <array>

using Landmarks = std::vector<cv::Point2f>;

// Overload some operators for the landmark points for readable arithmetic (not performance critical):
inline Landmarks operator*(const Landmarks& src, double w)
{
    Landmarks dst(src.size());
    for (int i = 0; i < src.size(); i++)
    {
        dst[i] = src[i] * w;
    }
    return dst;
}

inline Landmarks operator-(const Landmarks& a, const Landmarks& b)
{
    Landmarks dst(a.size());
    for (int i = 0; i < a.size(); i++)
    {
        dst[i] = a[i] - b[i];
    }
    return dst;
}

inline Landmarks operator+(const Landmarks& a, const Landmarks& b)
{
    Landmarks dst(a.size());
    for (int i = 0; i < a.size(); i++)
    {
        dst[i] = a[i] + b[i];
    }
    return dst;
}

struct FaceWithLandmarks
{
    cv::Mat image;
    Landmarks landmarks;
    Landmarks eyesNoseMouth; // {eyeR, eyeL, nose, mouthR, mouthL}
    std::string filename;

    FaceWithLandmarks& operator=(const FaceWithLandmarks& src)
    {
        image = src.image;
        landmarks = src.landmarks;
        return (*this);
    }

    FaceWithLandmarks& operator+=(const FaceWithLandmarks& src)
    {
        image += src.image;
        for (int i = 0; i < src.landmarks.size(); i++)
        {
            landmarks[i] += src.landmarks[i];
        }
        return (*this);
    }

    void flop()
    {
        switch (landmarks.size())
        {
            case 5:
            {
                // swap eyes
                std::swap(landmarks[0], landmarks[1]);
                // swap mouth corners
                std::swap(landmarks[3], landmarks[4]);
            }
            break;

            case 68:
            {
                // Contour
                std::swap(landmarks[0], landmarks[16]);
                std::swap(landmarks[1], landmarks[15]);
                std::swap(landmarks[2], landmarks[14]);
                std::swap(landmarks[3], landmarks[13]);
                std::swap(landmarks[4], landmarks[12]);
                std::swap(landmarks[5], landmarks[11]);
                std::swap(landmarks[6], landmarks[10]);
                std::swap(landmarks[7], landmarks[9]);
                std::swap(landmarks[8], landmarks[8]);

                // Eyebrow
                std::swap(landmarks[17], landmarks[26]);
                std::swap(landmarks[18], landmarks[25]);
                std::swap(landmarks[19], landmarks[24]);
                std::swap(landmarks[20], landmarks[23]);
                std::swap(landmarks[21], landmarks[22]);

                // Nose
                std::swap(landmarks[27], landmarks[27]);
                std::swap(landmarks[28], landmarks[28]);
                std::swap(landmarks[29], landmarks[29]);
                std::swap(landmarks[30], landmarks[30]);

                std::swap(landmarks[31], landmarks[35]);
                std::swap(landmarks[32], landmarks[34]);
                std::swap(landmarks[33], landmarks[33]);

                // Eye
                std::swap(landmarks[39], landmarks[42]);
                std::swap(landmarks[38], landmarks[43]);
                std::swap(landmarks[37], landmarks[44]);
                std::swap(landmarks[36], landmarks[45]);
                std::swap(landmarks[40], landmarks[47]);
                std::swap(landmarks[41], landmarks[46]);

                // Mouth
                std::swap(landmarks[48], landmarks[54]);
                std::swap(landmarks[49], landmarks[53]);
                std::swap(landmarks[50], landmarks[52]);
                std::swap(landmarks[51], landmarks[51]);

                std::swap(landmarks[59], landmarks[55]);
                std::swap(landmarks[58], landmarks[56]);
                std::swap(landmarks[57], landmarks[57]);

                std::swap(landmarks[60], landmarks[64]);
                std::swap(landmarks[61], landmarks[63]);
                std::swap(landmarks[62], landmarks[62]);

                std::swap(landmarks[67], landmarks[65]);
                std::swap(landmarks[66], landmarks[66]);
            }
            break;

            default:
                CV_Assert(landmarks.size() == 5 || landmarks.size() == 68);
        }
    }
};

inline FaceWithLandmarks operator*(const FaceWithLandmarks& src, double w)
{
    FaceWithLandmarks dst;
    dst.image = src.image * w;
    dst.landmarks = src.landmarks * w;
    return dst;
}

struct FaceWithLandmarksMean : public FaceWithLandmarks
{
    void updateMean(const FaceWithLandmarks& src)
    {
        // Create CV_32FC3 sample:
        cv::Mat image1;
        src.image.convertTo(image1, CV_32FC3, 1.0 / 255.0);
        updateMeanFloat(image1, src.landmarks);
    }

    int count = 0;

protected:
    void updateMeanFloat(const cv::Mat& image1, const std::vector<cv::Point2f>& landmarks1)
    {
        count++;
        if (image.empty())
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

    enum CropMode
    {
        kCrop,
        kMirror,
        kJitter
    };

    FaceJitterer(const FACE::Table& table, const FaceSpecification& face, const JitterParams& params);
    FaceWithLandmarks operator()(const cv::Mat& image, const Landmarks& landmarks, CropMode mode, bool doNoise);
    void setDoPreview(bool value) { m_doPreview = value; }
    bool getDoPreview() const { return m_doPreview; }

protected:
    static cv::Point2f mean(const Landmarks& landmarks, const std::vector<int>& index);

    const FACE::Table& m_table;
    const JitterParams& m_params;
    const FaceSpecification& m_face;
    std::array<cv::Point2f, 2> m_eyes;
    bool m_doPreview = false;

    cv::RNG m_rng;
};

#endif // __drishti_facecrop_FaceJitterer_h__
