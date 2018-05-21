/*!
  @file   FaceTrackerTest.cpp
  @author David Hirvonen
  @brief  Sample eye/face tracking using drishti.

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FaceTrackerTest.h"
#include "AsyncWorker.h"

#include <ogles_gpgpu/common/proc/disp.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <iomanip>

// clang-format off
namespace detail
{
    template <typename Value, typename... Arguments>
    std::unique_ptr<Value> make_unique(Arguments&&... arguments_for_constructor)
    {
        return std::unique_ptr<Value>(new Value(std::forward<Arguments>(arguments_for_constructor)...));
    }
}
// clang-format on

static void draw(cv::Mat& image, const drishti::sdk::Eye& eye);
static void draw(cv::Mat& image, const drishti::sdk::Face& face);

struct FaceTrackTest::Impl
{
    Impl(std::shared_ptr<spdlog::logger>& logger, const std::string& output)
        : logger(logger)
        , output(output)
        , counter(0)
    {
        worker.start();
    }

    ~Impl()
    {
        worker.stop();
    }

    // Preview methods {
    void initPreview(const cv::Size& size, GLenum textureFormat)
    {
        display = std::make_shared<ogles_gpgpu::Disp>();
        display->init(size.width, size.height, textureFormat);
        display->setOutputRenderOrientation(ogles_gpgpu::RenderOrientationFlipped);
    }

    void setPreviewGeometry(float tx, float ty, float sx, float sy)
    {
        display->setOffset(tx, ty);
        display->setDisplayResolution(sx, sy);
    }

    void updatePreview(std::uint32_t texture)
    {
        display->useTexture(texture);
        display->render(0);
    }
    // }

    std::shared_ptr<spdlog::logger> logger;
    std::string output;
    std::size_t counter;
    cv::Size size; // video resolution

    // Capture volume {
    std::chrono::high_resolution_clock::time_point captureTimestamp;
    double captureInterval = 0.0;
    struct Sphere
    {
        std::array<float, 3> center;
        float radius;
    } sphere = { { { 0.f, 0.f, 0.f } }, 0.f };
    // }

    AsyncWorker<std::function<void()>> worker;

    // This test class instantiates the ogles_gpgpu::Disp(lay) class in cases
    // where the user has provided a context w/ a visible and active OpenGL window,
    // and the display class will render directly to that screen.  The display
    // class is instantiated here in the pimpl class because the inputs textures are
    // available directly from the callbacks.   This could also be managed from a separate
    // listener registered to the face tracker, but simplicity it is added here.
    std::shared_ptr<ogles_gpgpu::Disp> display;
};

// See: https://github.com/elucideye/drishti/blob/master/src/lib/drishti/drishti/ut/test-FaceTracker.cpp
FaceTrackTest::FaceTrackTest(std::shared_ptr<spdlog::logger>& logger, const std::string& sOutput)
{
    m_impl = detail::make_unique<Impl>(logger, sOutput);
}

FaceTrackTest::~FaceTrackTest() = default;

void FaceTrackTest::initPreview(const cv::Size& size, GLenum textureFormat)
{
    m_impl->initPreview(size, textureFormat);
}

void FaceTrackTest::setPreviewGeometry(float tx, float ty, float sx, float sy)
{
    m_impl->setPreviewGeometry(tx, ty, sx, sy);
}

int FaceTrackTest::callback(drishti::sdk::Array<drishti_face_tracker_result_t, 64>& results)
{
    m_impl->logger->info("callback: Received results");

    if (results.size() > 0)
    {
        // Allocate a shared_ptr to store deep copies of the input data, so we can
        // pass this one around easily to separate worker threads, etc.
        auto stack = std::make_shared<StackType>(results.size());
        for (int i = 0; i < results.size(); i++)
        {
            (*stack)[i].result = results[i];

            // IMPORTANT: Here we make a deep copies of the input eye/frame images, since the requested
            // image is passed by a pointer that is only valid for the scope of the callback.  In some
            // cases the GPU->CPU transfer can be performed directly to pre-existing memory and
            // the public SDK layer can optimize for this by using teh allocator callback so that
            // memory can be allocated by the user/application layer.
            const auto& r = results[i];
            if (r.image.image.getRows() > 0 && r.image.image.getCols() > 0)
            {
                (*stack)[i].frame = drishti::sdk::drishtiToCv<drishti::sdk::Vec4b, cv::Vec4b>(r.image.image).clone();
            }
            if (r.eyes.image.getRows() > 0 && r.eyes.image.getCols() > 0)
            {
                (*stack)[i].eyes = drishti::sdk::drishtiToCv<drishti::sdk::Vec4b, cv::Vec4b>(r.eyes.image).clone();
            }
        }

        // Send the stack to the user's process method via the asynchronous
        // worker thread to avoid blocking in the main face tracker callback.
        m_impl->worker.post([this, stack] { this->process(*stack); });
    }

    return 0;
}

void FaceTrackTest::setCaptureSphere(const std::array<float, 3>& center, float radius, double seconds)
{
    m_impl->sphere = { center, radius };
    m_impl->captureInterval = seconds;
}

bool FaceTrackTest::shouldCapture(const drishti_face_tracker_result_t& faces)
{
    bool status = false;

    if (m_impl->sphere.radius > 0.f)
    {
        // Comnpute simple/global FPS
        const auto now = std::chrono::high_resolution_clock::now();
        const double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - m_impl->captureTimestamp).count();
        if (elapsed > m_impl->captureInterval)
        {
            for (const auto& f : faces.faceModels)
            {
                const cv::Scalar center(m_impl->sphere.center[0], m_impl->sphere.center[1], m_impl->sphere.center[2]);
                const float error = static_cast<float>(cv::norm(cv::Scalar(f.position[0], f.position[1], f.position[2]) - center));
                m_impl->logger->info("Error {}", error);
                if (error < m_impl->sphere.radius)
                {
                    status = true;
                    m_impl->captureTimestamp = now; // don't repeat captures too foten
                    break;
                }
            }
        }
    }

    return status;
}

// Here we would typically add some critiera required to trigger a full capture
// capture event.  We would do this selectively so that full frames are not
// retrieved at each stage of processing. For example, if we want to capture a s
// selfie image, we might monitor face positions over time to ensure that the
// subject is relatively centered in the image and that there is fairly low
// frame-to-frame motion.
drishti_request_t FaceTrackTest::trigger(const drishti_face_tracker_result_t& faces, double timestamp, std::uint32_t tex)
{
    m_impl->logger->info("trigger: Received results at time {}}", timestamp);

    if (m_impl->display)
    {
        m_impl->updatePreview(tex);
    }

    if (shouldCapture(faces))
    {
        // clang-format off
        return // Here we formulate the actual request, see drishti_request_t:
        {
            3,    // Retrieve the last N frames
            true, // Get frames in user memory
            true, // Get frames as texture ID's
            true, // Get full frame images
            true  // Get eye crop images
        };
        // clang-format on
    }

    return { 0 }; // otherwise request nothing!
}

int FaceTrackTest::allocator(const drishti_image_t& spec, drishti::sdk::Image4b& image)
{
    m_impl->logger->info("allocator: {} {}", spec.width, spec.height);
    return 0;
}

int FaceTrackTest::callbackFunc(void* context, drishti::sdk::Array<drishti_face_tracker_result_t, 64>& results)
{
    if (FaceTrackTest* ft = static_cast<FaceTrackTest*>(context))
    {
        return ft->callback(results);
    }
    return -1;
}

drishti_request_t FaceTrackTest::triggerFunc(void* context, const drishti_face_tracker_result_t& faces, double timestamp, std::uint32_t tex)
{
    if (FaceTrackTest* ft = static_cast<FaceTrackTest*>(context))
    {
        return ft->trigger(faces, timestamp, tex);
    }
    return { 0, false, false };
}

int FaceTrackTest::allocatorFunc(void* context, const drishti_image_t& spec, drishti::sdk::Image4b& image)
{
    if (FaceTrackTest* ft = static_cast<FaceTrackTest*>(context))
    {
        return ft->allocator(spec, image);
    }
    return -1;
}

void FaceTrackTest::setSizeHint(const cv::Size& size)
{
    m_impl->size = size;
}

void FaceTrackTest::process(StackType& stack)
{
    for (int i = 0; i < stack.size(); i++)
    {
        auto& s = stack[i];
        
        // Example: draw face models for frame
        //        for (int j = 0; j < s.result.faceModels.size(); j++)
        //        {
        //            const auto& f = s.result.faceModels[j];
        //            draw(s.frame, f);
        //        }
        
        { // Write the frame:
            std::stringstream ss;
            ss << m_impl->output << "/aframe_" << std::setw(4) << std::setfill('0') << m_impl->counter << "_" << i << ".png";
            cv::imwrite(ss.str(), s.frame);
        }
        
        // Example: draw eye models for nearest face
        //        for (int j = 0; j < s.result.eyeModels.size(); j++)
        //        {
        //            const auto &e = s.result.eyeModels[j];
        //            draw(s.eyes, e);
        //        }
        
        { // Write the eyes:
            std::stringstream ss;
            ss << m_impl->output << "/aeye_" << std::setw(4) << std::setfill('0') << m_impl->counter << "_" << i << ".png";
            cv::imwrite(ss.str(), s.eyes);
        }
    }
    m_impl->counter++;
}

// Utility {

void draw(cv::Mat& image, const drishti::sdk::Eye& eye)
{
    auto eyelids = drishti::sdk::drishtiToCv(eye.getEyelids());
    auto crease = drishti::sdk::drishtiToCv(eye.getCrease());
    auto pupil = drishti::sdk::drishtiToCv(eye.getPupil());
    auto iris = drishti::sdk::drishtiToCv(eye.getIris());

    cv::ellipse(image, iris, { 0, 255, 0 }, 1, 8);
    cv::ellipse(image, pupil, { 0, 255, 0 }, 1, 8);
    for (const auto& p : eyelids)
    {
        cv::circle(image, p, 2, { 0, 255, 0 }, -1, 8);
    }
    for (const auto& p : crease)
    {
        cv::circle(image, p, 2, { 0, 255, 0 }, -1, 8);
    }
}

void draw(cv::Mat& image, const drishti::sdk::Face& face)
{
    //    auto landmarks = drishti::sdk::drishtiToCv(face.landmarks);
    //    for (const auto &p : landmarks)
    //    {
    //        cv::circle(image, p, 2, {0,255,0}, -1, 8);
    //    }

    for (const auto& e : face.eyes)
    {
        draw(image, e);
    }
}

// }
