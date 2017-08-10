#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "aglet/GLContext.h"

#include <gtest/gtest.h>

// NOTE: GL_BGRA is absent in Android NDK

// clang-format off
#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#else
#  define TEXTURE_FORMAT GL_BGRA
#endif
// clang-format on

#include "ogles_gpgpu/common/gl/memtransfer_optimized.h"
#include "ogles_gpgpu/common/proc/video.h"
#include "ogles_gpgpu/common/proc/grayscale.h"

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    auto code = RUN_ALL_TESTS();
    return code;
}

static cv::Mat getImage(ogles_gpgpu::ProcInterface& proc, cv::Mat& frame)
{
    if (dynamic_cast<ogles_gpgpu::MemTransferOptimized*>(proc.getMemTransferObj()))
    {
        ogles_gpgpu::MemTransfer::FrameDelegate delegate = [&](const ogles_gpgpu::Size2d& size, const void* pixels, size_t bytesPerRow) {
            frame = cv::Mat(size.height, size.width, CV_8UC4, (void*)pixels, bytesPerRow).clone();
        };
        proc.getResultData(delegate);
    }
    else
    {
        frame.create(proc.getOutFrameH(), proc.getOutFrameW(), CV_8UC4); // noop if preallocated
        proc.getResultData(frame.ptr());
    }
    return frame;
}

static cv::Mat getTestImage(int width, int height, int stripe, bool alpha)
{
    // Create a test image:
    cv::Mat test(480, 640, CV_8UC3, cv::Scalar::all(0));
    cv::Point center(test.cols / 2, test.rows / 2);
    for (int i = test.cols / 2; i > 0; i -= stripe)
    {
        cv::circle(test, center, i, cv::Scalar(rand() % 255, rand() % 255, rand() % 255), -1, 8);
    }

    if (alpha)
    {
        cv::cvtColor(test, test, cv::COLOR_BGR2BGRA); // add alpha
    }
    return test;
}

TEST(OGLESGPGPUTest, GrayScaleProc)
{
    auto context = aglet::GLContext::create(aglet::GLContext::kAuto);
    if (context)
    {
        cv::Mat test = getTestImage(640, 480, 10, true);
        glActiveTexture(GL_TEXTURE0);
        ogles_gpgpu::VideoSource video;
        ogles_gpgpu::GrayscaleProc gray;

        video.set(&gray);
        video({ { test.cols, test.rows }, test.ptr<void>(), true, 0, TEXTURE_FORMAT });

        cv::Mat result;
        getImage(gray, result);
        ASSERT_FALSE(result.empty());
    }
}
