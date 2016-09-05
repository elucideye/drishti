/*!
  @file   finder/FaceFinder.cpp
  @author David Hirvonen
  @brief  Scene viewed by the camera represented by low level primitives: (corners, face, flow, etc.)

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FaceFinder.h"
#include "FetchResource.h"
#include "QtFaceDetectorFactory.h"
#include "gpu/FacePainter.h"
#include "gpu/FlashFilter.h"

#include "drishti/face/gpu/EyeFilter.h"
#include "drishti/eye/gpu/EllipsoPolarWarp.h"
#include "drishti/eye/gpu/EyeWarp.h"
#include "drishti/core/Logger.h"
#include "drishti/face/FaceDetectorAndTracker.h"
#include "drishti/face/FaceModelEstimator.h"
#include "drishti/face/FaceDetector.h"

#include "drishti/geometry/motion.h"

#include "ogles_gpgpu/common/proc/fifo.h"

#include <opencv2/highgui.hpp>

#include <vector>

#define DO_TRACKING 1
#define MODIFY_ACF 1

#define DO_CORNERS 0
#define DO_FLASH 0
#define DO_REGRESSION 1
#define DO_FLOW 1
#define DO_FLOW_QUIVER 0

#define DO_ACF_MODIFY 0

// Macbook (iSight)
#define REGRESSION_WIDTH 1024
#define DETECTION_WIDTH 512

using drishti::face::operator*;

static const char * sBar = "#################################################################";

template <typename T1, typename T2>
cv::Rect operator *(const cv::Rect_<T1> &roi, const T2 &scale)
{
    return cv::Rect_<T1>(roi.x * scale, roi.y * scale, roi.width * scale, roi.height * scale);
}

template <typename T1, typename T2>
std::vector<T1> operator *(const std::vector<T1> &src, const T2 &scale)
{
    auto dst = src;
    for(auto &r : dst)
    {
        r = r * scale;
    }
    return dst;
}

template <typename T1, typename T2>
cv::Size_<T1> operator*(const cv::Size_<T1> &size, const T2 &scale)
{
    return cv::Size_<T1>(T2(size.width) * scale, T2(size.height) * scale);
}

static cv::Size uprightSize(const cv::Size &size, int orientation);
static void extractCorners(const cv::Mat1b &corners, ScenePrimitives &scene, float flowScale = 1.f);
static void extractFlow(const cv::Mat4b &ayxb, const cv::Size &frameSize, ScenePrimitives &scene, float flowScale = 1.f);

// === utility ===

FaceFinder::FaceFinder(std::shared_ptr<drishti::face::FaceDetectorFactory> &factory, Config &args, void *glContext)
    : m_glContext(glContext)
    , m_hasInit(false)
    , m_outputOrientation(args.outputOrientation)
    , m_doRegression(DO_REGRESSION) // ########## DO REGRESSION #########
    , m_regressionWidth(REGRESSION_WIDTH)
    , m_doCorners(DO_CORNERS)
    , m_cornerWidth(512)
    , m_doFlow(DO_FLOW)  // ######## DO FLOW #########
    , m_flowWidth(128)
    , m_doFlash(DO_FLASH) // ###### DO FLASH ##########
    , m_flashWidth(128)
    , m_scenes(args.delay+1)
    , m_factory(factory)
    , m_sensor(args.sensor)
    , m_logger(args.logger)
    , m_threads(args.threads)
{
  
}

void FaceFinder::dump(std::vector<cv::Mat4b> &frames)
{
    if(m_fifo->getBufferCount() == m_fifo->getProcPasses().size())
    {
        auto &filters = m_fifo->getProcPasses();
        frames.resize(filters.size());
        for(int i = 0; i < filters.size(); i++)
        {
            frames[i].create( filters[i]->getOutFrameH(), filters[i]->getOutFrameW() );
            filters[i]->getResultData(frames[i].ptr<uint8_t>());
        }
    }
}

void FaceFinder::init(const FrameInput &frame)
{
    m_logger->info() << "FaceFinder::init()";

    m_logger->set_level(spdlog::level::err);

    const cv::Size inputSize(frame.size.width, frame.size.height);

    auto inputSizeUp = inputSize;
    bool hasTranspose = ((m_outputOrientation / 90) % 2);
    if(hasTranspose)
    {
        std::swap(inputSizeUp.width, inputSizeUp.height);
    }

    m_hasInit = true;

    createColormap();

    {
        // ### Fifo ###
        m_fifo = std::make_shared<ogles_gpgpu::FifoProc>(2);
        m_fifo->init(inputSize.width, inputSize.height, INT_MAX, false);
        m_fifo->createFBOTex(false);
    }

    {
        // ### ACF (Transpose) ###

        // 120 iphone
        // 1024 macbook
        // Detection width should be set based on device focal length:
        const int detectionWidth = DETECTION_WIDTH;
        m_scale = float(inputSizeUp.width) / float(detectionWidth);

        // ACF implementation uses reduce resolution transposed image:
        cv::Size detectionSize = inputSizeUp * (1.0f/m_scale);
        cv::Mat I(detectionSize.width, detectionSize.height, CV_32FC3, cv::Scalar::all(0));
        MatP Ip(I);
        m_detector->computePyramid(Ip, m_P);

        std::vector<ogles_gpgpu::Size2d> sizes;
        for(int i = 0; i < m_P.nScales; i++)
        {
            const auto size = m_P.data[i][0][0].size();
            sizes.emplace_back(size.width * 4, size.height * 4); // undo ACF binning x4
        }
        // CPU processing works with tranposed images for col-major storage assumption.
        // Undo that here:
        for(auto &s : sizes)
        {
            std::swap(s.width, s.height); // leave here to make T explicit
        }

        //const int cornerWidth = m_doCorners ? m_cornerWidth : 0;
        const int grayWidth = m_doRegression ? m_regressionWidth : 0;
        const int flowWidth = m_doFlow ? m_flowWidth : 0;
        const ogles_gpgpu::Size2d size(inputSizeUp.width,inputSizeUp.height);
        m_acf = std::make_shared<ogles_gpgpu::ACF>(m_glContext, size, sizes, grayWidth, flowWidth, false);
        m_acf->setRotation(m_outputOrientation);
    }

    {
        // ### Painter ###
        ogles_gpgpu::RenderOrientation outputOrientation = ogles_gpgpu::degreesToOrientation(360 - m_outputOrientation);
        m_rotater = std::make_shared<ogles_gpgpu::TransformProc>();
        m_rotater->setOutputRenderOrientation(outputOrientation);

        m_painter = std::make_shared<ogles_gpgpu::FacePainter>(0);
        m_painter->add(m_rotater.get());
        m_painter->prepare(inputSizeUp.width, inputSizeUp.height, GL_RGBA);
    }

    {
        // ### Flash ###
        m_flasher = std::make_shared<ogles_gpgpu::FlashFilter>();
        m_flasher->smoothProc.setOutputSize(48, 0);
        m_acf->rgbSmoothProc.add(&m_flasher->smoothProc);
    }

    ogles_gpgpu::Size2d eyesSize(480, 240);

#if DO_ELLIPSO_POLAR
    {
        // ### Ellipsopolar warper ####
        for(int i = 0; i < 2; i++)
        {
            m_ellipsoPolar[i] = std::make_shared<ogles_gpgpu::EllipsoPolarWarp>();
            m_ellipsoPolar[i]->setOutputSize(640, 240);
        }
    }
#endif // DO_ELLIPSO_POLAR

    {
        // ### Eye enhancer ###
        auto mode = ogles_gpgpu::EyeFilter::kLowPass;
        const float upper = 0.5;
        const float lower = 0.5;
        const float gain = 1.f;
        const float offset = 0.f;

        m_eyeFilter = std::make_shared<ogles_gpgpu::EyeFilter>(eyesSize, mode, upper, lower, gain, offset);
        m_eyeFilter->setAutoScaling(true);
        m_eyeFilter->setOutputSize(eyesSize.width, eyesSize.height);

#if DO_ELLIPSO_POLAR
        // Add a callback to retrieve updated eye models automatically:
        cv::Matx33f N = transformation::scale(0.5, 0.5) * transformation::translate(1.f, 1.f);
        for(int i = 0; i < 2; i++)
        {
            std::function<EyeWarp()> eyeDelegate = [&, N, i]()
            {
                auto eye = m_eyeFilter->getEyeWarps()[i];
                eye.eye = N * eye.H * eye.eye;
                return eye;
            };
            m_ellipsoPolar[i]->addEyeDelegate(eyeDelegate);
            m_eyeFilter->add(m_ellipsoPolar[i].get());
        }
#endif // DO_ELLIPSO_POLAR

        m_eyeFilter->prepare(inputSizeUp.width, inputSizeUp.height, (GLenum)GL_RGBA);
    }

}

// ogles_gpgpu::VideoSource can support list of subscribers
//
//       +=> FIFO[1][0] == ACF ====>
// VIDEO |       |
//       +=======+======== FLOW ===>

GLuint FaceFinder::operator()(const FrameInput &frame)
{
    m_logger->info() << "FaceFinder::operator() " << sBar;

    if(!m_hasInit)
    {
        m_hasInit = true;
        init2(*m_factory);
        init(frame);
    }

    assert(m_fifo->size() == m_scenes.size());

    m_frameIndex++; // increment frame index

    // Run GPU based processing on current thread and package results as a task for CPU
    // processing so that it will be available on the next frame.  This method will compute
    // ACF output using shaders on the GPU.
    ScenePrimitives scene(m_frameIndex);
    preprocess(frame, scene);

    GLuint inputTexId = m_acf->getInputTexId(), outputTexId = 0;

    inputTexId = m_acf->first()->getOutputTexId(); // override with the upright textures

    if(m_threads)
    {
        size_t inputIndex = m_fifo->getIn(); // index where inputTexture will be stored
        size_t outputIndex = m_fifo->getOut();
        outputTexId = m_fifo->getOutputFilter()->getOutputTexId();

        m_fifo->useTexture(inputTexId, 1); // uses inputIndex
        m_fifo->render();

        // Run CPU processing for this frame so that it will be available for the next frame
        m_scenes[inputIndex] = m_threads->process([scene,frame,this]()
        {
            ScenePrimitives sceneOut = scene;
            detect(frame, sceneOut);
            return sceneOut;
        });

        // Don't bother processing until FIFO is full:
        if(!m_fifo->isFull())
        {
            return m_acf->getInputTexId();
        }

        try
        {
            // Here we retrieve the CPU scene output for the previous frame,
            // which has been running on the job queue.  This adds one frame
            // of latency, but allows us to keep the GPU and CPU utilized.
            scene = m_scenes[outputIndex].get();
        }
        catch(...)
        {
            m_logger->error() << "Error with the output scene";
        }

        outputTexId = paint(scene, outputTexId);
    }
    else
    {
        detect(frame, scene);
        outputTexId = paint(scene, inputTexId);
    }

    return outputTexId;
}

GLuint FaceFinder::paint(const ScenePrimitives &scene, GLuint inputTexture)
{
    // Convert objects to line drawings
    m_painter->getLineDrawings().clear();

    if(scene.corners().size())
    {
        pointsToCrosses(scene.corners(), m_painter->getLineDrawings());
    }

    if(scene.faces().size())
    {
        facesToDrawings(scene.faces(), m_painter->getLineDrawings());
        for(const auto &f : scene.faces())
        {
            m_painter->addFace(f);
            m_eyeFilter->addFace(f);
        }

        // Configure eye enhancer
        m_eyeFilter->process(inputTexture, 1, GL_TEXTURE_2D);
        m_painter->setEyeTexture(m_eyeFilter->getOutputTexId(), m_eyeFilter->getOutFrameSize(), m_eyeFilter->getEyeWarps());

#if DO_ELLIPSO_POLAR
        //Draw the polar warp:
        for(int i = 0; i < 2; i++)
        {
            m_painter->setIrisTexture(i, m_ellipsoPolar[i]->getOutputTexId(), m_ellipsoPolar[i]->getOutFrameSize());
        }
#endif
    }
    else if(scene.objects().size())
    {
        rectanglesToDrawings(scene.objects() * m_scale, m_painter->getLineDrawings());
    }

    if(m_doFlow)
    {
        if(scene.flow().size())
        {
            flowToDrawings(scene.flow(), m_painter->getLineDrawings(), m_colors32FC3);
        }

        // Add the flow for debugging:
        m_painter->setFlowTexture(m_acf->flow.getOutputTexId(), m_acf->flow.getOutFrameSize());
    }

    if(m_doFlash)
    {
        m_painter->setFlashTexture(m_flasher->last()->getOutputTexId(), m_flasher->last()->getOutFrameSize());
    }

    m_painter->process(inputTexture, 1, GL_TEXTURE_2D);
    
    return m_rotater->getOutputTexId();
}

void FaceFinder::createColormap()
{
    // ##### Create a colormap ######
    cv::Mat1b colorsU8(1, 360);
    cv::Mat3b colorsU8C3;
    cv::Mat3f colors32FC3;
    for(int i = 0; i < 360; i++)
    {
        colorsU8(0,i) = uint8_t( 255.f * float(i)/(colorsU8.cols-1) + 0.5f );
    }

    cv::applyColorMap(colorsU8, colorsU8C3, cv::COLORMAP_HSV);
    colorsU8C3.convertTo(m_colors32FC3, CV_32FC3, 1.0/255.0);
}

/**
 * GPU preprocessing:
 * (1) FrameInpute -> texture -> ACF output image (P pyramid)
 * (2) Harris/Shi-Tomasi corners
 * (3) Resized grayscale image for face regression
 */

void FaceFinder::preprocess(const FrameInput &frame, ScenePrimitives &scene)
{
    m_logger->info() << "FaceFinder::preprocess()  " <<  int(frame.textureFormat) << sBar;

    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);

    (*m_acf)(frame);

    cv::Mat acf = m_acf->getChannels();
    assert(acf.type() == CV_8UC1);
    assert(acf.channels() == 1);

    if(m_acf->getChannelStatus())
    {
        auto &P = scene.m_P;
        P = std::make_shared<decltype(m_P)>();
        fill(*P);
    }

    auto thing = m_acf->getFlowPyramid();

#if DO_FLOW_QUIVER
    if(m_acf->getFlowStatus())
    {
        cv::Size frameSize = uprightSize({frame.size.width, frame.size.height}, m_outputOrientation);
        cv::Mat4b ayxb = m_acf->getFlow();
        extractFlow(ayxb, frameSize, scene, 1.0f / m_acf->getFlowScale());
    }
#endif

#define LOG_ACF 0
#if LOG_ACF
    {
        cv::Mat canvas;
        cv::cvtColor(acf, canvas, cv::COLOR_GRAY2BGR);
        for(const auto &i : crops) for(const auto &j : i)
                cv::rectangle(canvas, {j.x,j.y,j.width,j.height}, {0,255,0}, 1, 8);

        std::string home = getenv("HOME");
        home += "/Documents/";
        cv::imwrite(home + "acf.png", canvas);
        cv::imshow("acf", acf), cv::waitKey(10);
        //cv::imwrite("/tmp/data/acf.png", canvas);
    }
#endif

    // ### Grayscale image ###
    if(m_doRegression)
    {
        scene.image() = m_acf->getGrayscale();
    }
}

void FaceFinder::fill(drishti::acf::Detector::Pyramid &P)
{
    auto crops = m_acf->getCropRegions();
    assert(crops.size() > 1);

    P.pPyramid = m_P.pPyramid;
    P.nTypes = m_P.nTypes;
    P.nScales = m_P.nScales;
    P.info = m_P.info;
    P.lambdas = m_P.lambdas;
    P.scales = m_P.scales;
    P.scaleshw = m_P.scaleshw;

    P.rois.resize(crops.size());
    for(int i = 0; i < crops.size(); i++)
    {
        P.rois[i].resize(crops[i].size());
        for(int j = 0; j < crops[i].size(); j++)
        {
            const auto &r = crops[i][j];
            P.rois[i][j] = cv::Rect(r.x, r.y, r.width, r.height);
        }
    }
    m_acf->fill(P);
}

int FaceFinder::detect(const FrameInput &frame, ScenePrimitives &scene)
{
    m_logger->info() << "FaceFinder::detect() " << sBar;

    if(m_detector != nullptr && scene.m_P)
    {
        // Test GPU ACF detection
        // Fill in ACF Pyramid structure
        std::vector<double> scores;

        // *m_detector

        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - m_objects.first).count();
        if(elapsed > 0.1)
        {
            (*m_detector)(*scene.m_P, scene.objects(), &scores);
            m_objects = std::make_pair(std::chrono::high_resolution_clock::now(), scene.objects());
        }
        else
        {
            scene.objects() = m_objects.second;
        }

        if(m_doRegression && scene.objects().size())
        {
#define LOG_DETECTIONS 0
#if LOG_DETECTIONS
            cv::Mat frame(frame.size, CV_8UC4, frame.pixelBuffer);
            for(auto &d : objects)
            {
                cv::Rect2f df(d);
                cv::rectangle(frame, {df.tl()*scale, df.br()*scale}, {0,255,0}, 1, 8);
            }
            cv::imwrite("/tmp/data/detection.png", frame);
#endif

            auto &objects = scene.objects();
            std::vector<drishti::face::FaceModel> faces(objects.size());
            for(int i = 0; i < faces.size(); i++)
            {
                faces[i].roi = objects[i];
            }

            bool isDetection = true;

            cv::Mat1b gray = scene.image();

            const float Sdr = m_scale /* acf->full */ * m_acf->getGrayscaleScale() /* full->gray */;
            cv::Matx33f Hdr(Sdr,0,0,0,Sdr,0,0,0,1); //  = cv::Matx33f::eye();
            drishti::face::FaceDetector::PaddedImage Ib(gray, {{0,0}, gray.size()});

            m_faceDetector->setDoIrisRefinement(true);

            m_faceDetector->setFaceStagesHint(8);
            m_faceDetector->setFace2StagesHint(4);
            m_faceDetector->setEyelidStagesHint(4);
            m_faceDetector->setIrisStagesHint(10);
            m_faceDetector->setIrisStagesRepetitionFactor(1);

            m_faceDetector->refine(Ib, faces, Hdr, isDetection);

            //float iod = cv::norm(faces[0].eyeFullR->irisEllipse.center - faces[0].eyeFullL->irisEllipse.center);

            // Scale faces from regression to level 0
            // The configuration sizes used in the ACF stcked channe image
            // are all upright, but the output texture used for the display
            // is still in teh native (potentially rotated) coordinate system,
            // so we need to perform scaling wrt that.

            const float Srf = 1.0f / m_acf->getGrayscaleScale();
            cv::Matx33f H0(Srf,0,0,0,Srf,0,0,0,1);
            for(auto &f : faces)
            {
                f = H0 * f;
            }

            scene.faces() = faces;
        }
    }

    return 0;
}

// #### init2 ####

void FaceFinder::init2(drishti::face::FaceDetectorFactory &resources)
{
    m_logger->info() << "FaceFinder::init2() " << sBar;

    m_timerInfo.detectionTimeLogger = [this](double seconds)
    {
        this->m_timerInfo.detectionTime = seconds;
    };
    m_timerInfo.regressionTimeLogger = [this](double seconds)
    {
        this->m_timerInfo.regressionTime = seconds;
    };
    m_timerInfo.eyeRegressionTimeLogger = [this](double seconds)
    {
        this->m_timerInfo.eyeRegressionTime = seconds;
    };


#if DO_TRACKING
    auto faceDetectorAndTracker = std::make_shared<drishti::face::FaceDetectorAndTracker>(resources);
    faceDetectorAndTracker->setMaxTrackAge(2.0);
    m_faceDetector = faceDetectorAndTracker;
#else
    m_faceDetector = std::make_shared<drishti::face::FaceDetector>(resources);
#endif
    m_faceDetector->setDoNMS(true);
    m_faceDetector->setInits(1);

    // Get weak ref to underlying ACF detector
    m_detector = dynamic_cast<drishti::acf::Detector*>(m_faceDetector->getDetector());

#if DO_ACF_MODIFY
    if(m_detector)
    {
        // Perform modification
        drishti::acf::Detector::Modify dflt;
        dflt.cascThr = { "cascThr", -1.0 };
        dflt.cascCal = { "cascCal", +0.01 };
        m_detector->acfModify( dflt );
    }
#endif

    m_faceDetector->setDetectionTimeLogger(m_timerInfo.detectionTimeLogger);
    m_faceDetector->setRegressionTimeLogger(m_timerInfo.regressionTimeLogger);
    m_faceDetector->setEyeRegressionTimeLogger(m_timerInfo.eyeRegressionTimeLogger);

    {
        // FaceDetection mean:
        drishti::face::FaceModel faceDetectorMean = m_factory->getMeanFace();

        // We can change the regressor crop padding by doing a centered scaling of face features:
        if(0)
        {
            std::vector<cv::Point2f> centers
            {
                faceDetectorMean.getEyeLeftCenter(),
                faceDetectorMean.getEyeRightCenter(),
                *faceDetectorMean.noseTip
            };
            cv::Point2f center = drishti::core::centroid(centers);
            cv::Matx33f S(cv::Matx33f::diag({0.75, 0.75, 1.0}));
            cv::Matx33f T1(1,0,+center.x,0,1,+center.y,0,0,1);
            cv::Matx33f T2(1,0,-center.x,0,1,-center.y,0,0,1);
            cv::Matx33f H = T1 * S * T2;
            faceDetectorMean = H * faceDetectorMean;
        }

        m_faceDetector->setFaceDetectorMean(faceDetectorMean);
    }
}

// #### utilty: ####

static cv::Size uprightSize(const cv::Size &size, int orientation)
{
    cv::Size upSize = size;
    if((orientation / 90) % 2)
    {
        std::swap(upSize.width, upSize.height);
    }
    return upSize;
}

static void extractCorners(const cv::Mat1b &corners, ScenePrimitives &scene, float flowScale)
{
    // ### Extract corners first: ###
    std::vector<cv::Point> points;
    try
    {
        cv::findNonZero(corners, points);
    }
    catch(...) {}
    for(const auto &p : points)
    {
        scene.corners().emplace_back(flowScale * p.x, flowScale * p.y);
    }
}

static void extractFlow(const cv::Mat4b &ayxb, const cv::Size &frameSize, ScenePrimitives &scene, float flowScale)
{
    // Compute size of flow image for just the lowest pyramid level:
    cv::Size flowSize = cv::Size2f(frameSize) * (1.0f / flowScale);
    cv::Rect flowRoi({0,0}, ayxb.size());
    flowRoi &= cv::Rect({0,0}, flowSize);

    const int step = 2;
    scene.flow().reserve(scene.flow().size() + (ayxb.rows/step * ayxb.cols/step));

    for(int y = 0; y < ayxb.rows; y += step)
    {
        for(int x = 0; x < ayxb.cols; x += step)
        {
            // Extract flow:
            const cv::Vec4b &pixel = ayxb(y,x);
            cv::Point2f p(pixel[2], pixel[1]);
            cv::Point2f d = (p * (2.0f / 255.0f)) - cv::Point2f(1.0f, 1.0f);
            d *= 2.0; // additional scale for visualization
            scene.flow().emplace_back(cv::Vec4f(x, y, d.x, d.y) * flowScale);
        }
    }

    cv::Mat1b corners;
    cv::extractChannel(ayxb(flowRoi), corners, 0);
    extractCorners(corners, scene, flowScale);
}


