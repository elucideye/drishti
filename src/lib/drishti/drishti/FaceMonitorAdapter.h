/**
  @file   FaceMonitorAdapter.h
  @author David Hirvonen
  @brief  Private utility class to map extern "C" callbacks to C++ API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

 */

#ifndef __drishti_drishti_FaceMonitorAdapter_h__
#define __drishti_drishti_FaceMonitorAdapter_h__ 1

#include "drishti/drishti_sdk.hpp"
#include "drishti/drishti_cv.hpp"
#include "drishti/EyeSegmenterImpl.hpp"

#include "drishti/face/Face.h"
#include "drishti/hci/FaceFinder.h"

_DRISHTI_SDK_BEGIN


// Maintain lightweight inline conversions in private header for internal use
inline drishti::sdk::Face convert(const drishti::face::FaceModel& model)
{
    drishti::sdk::Face f;

    if(model.eyeFullR.has && model.eyeFullL.has)
    {
        f.eyes.resize(2);
        f.eyes[0] = drishti::sdk::convert(*model.eyeFullR);
        f.eyes[1] = drishti::sdk::convert(*model.eyeFullL);
    }

    if(model.points.has)
    {
        f.landmarks = drishti::sdk::cvToDrishti(*model.points);
    }

    return f;
}

inline drishti::face::FaceModel convert(const drishti::sdk::Face& model)
{
    drishti::face::FaceModel f;

    f.eyeFullL = drishti::sdk::convert(model.eyes[0]);
    f.eyeFullR = drishti::sdk::convert(model.eyes[1]);
    f.points = drishti::sdk::drishtiToCv(model.landmarks);

    return f;
}

/**
 * FaceMonitorAdapter
 *
 * Provides an interface for converting drishti::hc::FaceMonitor output to the public extern "C" API
 */

struct FaceMonitorAdapter : public drishti::hci::FaceMonitor
{
public:
    using HighResolutionClock = std::chrono::high_resolution_clock;
    using TimePoint = HighResolutionClock::time_point; // <std::chrono::system_clock>;
    using Faces = std::vector<drishti::face::FaceModel>;    
    
    FaceMonitorAdapter(drishti_face_tracker_t& table, int n=std::numeric_limits<int>::max())
        : m_start(HighResolutionClock::now())
        , m_table(table)
        , m_n(n)
    {
    }

    ~FaceMonitorAdapter() = default;
    
    virtual Request request(const Faces& faces, const TimePoint& timeStamp)
    {
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(timeStamp - m_start).count();
        
        drishti_face_tracker_result_t result;
        convert(faces, result.faceModels);
        
        auto request = m_table.update(m_table.context, result, elapsed);
        
        return Request{request.n, request.getImage, request.getTexture};
    }
    
    virtual void grab(const std::vector<FaceImage>& frames, bool isInitialized)
    {
        // Populate public API buffer using public SDK wrapper types w/ shallow copy:
        drishti::sdk::Array<drishti_face_tracker_result_t, 64> results(frames.size());

        for (size_t i = 0; i < frames.size(); i++)
        {
            // Copy the full frame "face" image and metadata:
            convert(frames[i].image, results[i].image);
            convert(frames[i].faceModels, results[i].faceModels);
            
            // Copy the eye images and metadata:
            convert(frames[i].eyes, results[i].eyes);
            results[i].eyeModels.resize(2);
            for (int j = 0; j < 2; j++)
            {
                if (frames[i].eyeModels[j].eyelids.size())
                {
                    results[i].eyeModels[j] = drishti::sdk::convert(frames[i].eyeModels[j]);
                }
            }
        }

        m_table.callback(m_table.context, results);
    }

protected:
    
    static void convert(const Faces& facesIn, drishti::sdk::Array<drishti::sdk::Face, 2> &facesOut)
    {
        facesOut.resize(std::min(facesOut.limit(), facesIn.size()));
        for (std::size_t i = 0; i < facesOut.size(); i++)
        {
            facesOut[i] = drishti::sdk::convert(facesIn[i]);
        }
    }
    
    static drishti::sdk::Texture convert(const core::Texture &texture)
    {
        return {{texture.size.width, texture.size.height}, texture.texId };
    }
    
    static void convert(const core::ImageView &src, drishti_image_tex_t &dst)
    {
        dst.texture = convert(src.texture);
        if (!src.image.empty())
        {
            dst.image = cvToDrishti<cv::Vec4b, drishti::sdk::Vec4b>(src.image);
        }
    }
    
    TimePoint m_start; //! Timestmap for the start of tracking
    drishti_face_tracker_t m_table; //! Table of callbacks for face tracker output
    int m_n = 1; //! Number of frames to request for each callback
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_FaceMonitorAdapter_hpp__
