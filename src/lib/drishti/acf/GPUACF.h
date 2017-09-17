/*! -*-c++-*-
  @file   GPUACF.h
  @author David Hirvonen
  @brief  Implementation of OpenGL shader optimized Aggregated Channel Feature computation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_acf_GPUACF_h__
#define __drishti_acf_GPUACF_h__

#include "drishti/acf/drishti_acf.h"
#include "drishti/acf/ACF.h"
#include "drishti/core/convert.h" // for drishti::core::PlaneInfo

#include "ogles_gpgpu/common/proc/video.h"

#include <memory>
#include <array>

// clang-format off
namespace spdlog { class logger; }
// clang-format on

BEGIN_OGLES_GPGPU

class Flow2Pipeline;

// #### GPU #####

#define GPU_ACF_TRANSPOSE 1

struct ACF : public ogles_gpgpu::VideoSource
{
public:
    enum FeatureKind
    {
        kM012345,    // 7
        kLUVM012345, // 10
        kUnknown
    };

    using array_type = drishti::acf::Detector::Pyramid::array_type;
    using SizeVec = std::vector<Size2d>;
    using PlaneInfoVec = std::vector<drishti::core::PlaneInfo>;
    using ChannelSpecification = std::vector<std::pair<PlaneInfoVec, ProcInterface*>>;

    ACF(void* glContext, const Size2d& size, const SizeVec& scales, FeatureKind kind, int grayWidth = 0, int flowWidth = 0, bool debug = false);
    ~ACF();

    static void tryEnablePlatformOptimizations();

    void setUsePBO(bool flag);
    bool getUsePBO() const;
    
    void setLogger(std::shared_ptr<spdlog::logger>& logger);
    bool getChannelStatus();
    bool getFlowStatus();
    void setDoLuvTransfer(bool flag);
    void setDoAcfTrasfer(bool flag);

    // ACF base resolution to Grayscale image
    float getGrayscaleScale() const;

    // Scale of flow wrt inputimage
    float getFlowScale() const;

    Flow2Pipeline* getFlowProc();
    const std::array<int, 4>& getChannelOrder();

    virtual void preConfig();
    virtual void postConfig();
    static cv::Mat getImage(ProcInterface& proc);
    static cv::Mat getImage(ProcInterface& proc, cv::Mat& frame);

    virtual void operator()(const FrameInput& frame);
    virtual void operator()(const Size2d& size, void* pixelBuffer, bool useRawPixels, GLuint inputTexture = 0, GLenum inputPixFormat = DFLT_PIX_FORMAT);

    void release();
    void connect(std::shared_ptr<spdlog::logger>& logger);
    void setRotation(int degrees);

    static bool processImage(ProcInterface& proc, MemTransfer::FrameDelegate& delegate);
    int getChannelCount() const; // LUVM0123456
    std::vector<std::vector<Rect2d>> getCropRegions() const;
    std::vector<Rect2d> getChannelCropRegions(int level = 0) const;
    void prepare();
    void fill(drishti::acf::Detector::Pyramid& pyramid);
    void fill(drishti::acf::Detector::Pyramid& Pout, const drishti::acf::Detector::Pyramid& Pin);

    // GPU => CPU for ACF:
    cv::Mat getChannels();

    // GPU => CPU for grayscale:
    const cv::Mat& getGrayscale();

    // GPU => CPU for optical flow:
    const cv::Mat& getFlow();

    std::vector<cv::Mat> getFlowPyramid();

    ProcInterface* first();
    ProcInterface* getRgbSmoothProc();

    // Retrieve Luv image as planar 3 channel CV_32F
    const MatP& getLuvPlanar();

    // Retrieve Luv image in packed CV_8UC4 (RGBA) format
    const cv::Mat& getLuv();

    void beginTransfer();

protected:
    cv::Mat getChannelsImpl();

    std::array<int, 4> initChannelOrder();
    void initACF(const SizeVec& scales, FeatureKind kind, bool debug);
    void initLuvTransposeOutput();
    ChannelSpecification getACFChannelSpecification(MatP& acf) const;

    struct Impl;
    std::unique_ptr<Impl> impl;
};

ACF::FeatureKind getFeatureKind(const drishti::acf::Detector::Options::Pyramid::Chns& chns);

END_OGLES_GPGPU

#endif // __drishti_acf_GPUACF_h__
