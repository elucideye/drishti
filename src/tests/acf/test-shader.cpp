/*!
  @file   test-shader.cpp
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  GPU ACF shader tests using a google test fixture.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

#include "QGLContext.h"
#include "core/Logger.h"
#include "core/drawing.h"
#include "acf/ACF.h"
#include "acf/MatP.h"
#include "acf/GPUACF.h"
#include "acf/gpu/gain.h"
#include "acf/gpu/triangle.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>
#include <memory>

#include "ogles_gpgpu/common/proc/box_opt.h"
#include "ogles_gpgpu/common/proc/gauss_opt.h"
#include "ogles_gpgpu/common/proc/flow.h"
#include "ogles_gpgpu/common/proc/ixyt.h"
#include "ogles_gpgpu/common/proc/fifo.h"
#include "ogles_gpgpu/common/proc/filter3x3.h"
#include "ogles_gpgpu/common/proc/iir.h"
#include "ogles_gpgpu/common/proc/lowpass.h"
#include "ogles_gpgpu/common/proc/highpass.h"
#include "ogles_gpgpu/common/proc/remap.h"
#include "ogles_gpgpu/common/proc/fir3.h"
#include "ogles_gpgpu/common/proc/median.h"

// https://code.google.com/p/googletest/wiki/Primer

const char* imageFilename;
const char* truthFilename;

#ifdef ANDROID
#  define DFLT_TEXTURE_FORMAT GL_RGBA
#else
#  define DFLT_TEXTURE_FORMAT GL_BGRA
#endif

#include <iostream>
#include <chrono>

//#define BEGIN_OGLES_GPGPU namespace ogles_gpgpu {
//#define END_OGLES_GPGPU }

#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }

#define TIMING 1

#ifdef TIMING
#define INIT_TIMER auto start = std::chrono::high_resolution_clock::now();
#define START_TIMER  start = std::chrono::high_resolution_clock::now();
#define STOP_TIMER(name)  std::cout << "RUNTIME of " << name << ": " << \
std::chrono::duration_cast<std::chrono::milliseconds>( \
std::chrono::high_resolution_clock::now()-start \
).count() << " ms " << std::endl;
#else
#define INIT_TIMER
#define START_TIMER
#define STOP_TIMER(name)
#endif

#include <opencv2/imgproc/imgproc.hpp>

BEGIN_EMPTY_NAMESPACE

class ACFTest : public ::testing::Test
{
protected:

    bool m_hasTranspose = false;

    std::function<cv::Mat(const cv::Mat &)> m_toUpright;
    std::function<cv::Mat(const cv::Mat &)> m_toTranspose;

    // Setup
    ACFTest()
    {
        m_logger = drishti::core::Logger::create("test-acf");

        // Load the ground truth data:

        image = loadImage(imageFilename);

#define TEST_ROTATION 1
#if TEST_ROTATION
        cv::Mat r;
        cv::flip(image.t(), r, 0);
        cv::swap(image, r);
        m_hasTranspose = true;

        m_toTranspose = [](const cv::Mat &src)
        {
            cv::Mat dst;
            cv::flip(src, dst, 0);
            return dst;
        };

        m_toUpright = [](const cv::Mat &src)
        {
            cv::Mat dst;
            cv::flip(src.t(), dst, 1);
            return dst;
        };
#endif

        // TODO: we need to load ground truth output for each shader
        // (some combinations could be tested, but that is probably excessive!)
        truth = loadImage(truthFilename);

        m_context = std::make_shared<QGLContext>();
    }

    // Cleanup
    virtual ~ACFTest()
    {
        drishti::core::Logger::drop("test-acf");
    }

    // Called after constructor for each test
    virtual void SetUp() {}

    // Called after destructor for each test
    virtual void TearDown() {}

    static cv::Mat loadImage(const std::string &filename)
    {
        assert(!filename.empty());
        cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);

        assert(!image.empty() && image.type() == CV_8UC3);

        cv::Mat tmp;
        cv::cvtColor(image, tmp, cv::COLOR_BGR2BGRA);
        cv::swap(image, tmp);
        return image;
    }

    std::shared_ptr<QGLContext> m_context;
    std::shared_ptr<spdlog::logger> m_logger;

    // Test images:
    cv::Mat image, truth;
};

struct SobelResult
{
    cv::Mat mag, theta, dx, dy;

    cv::Mat getAll()
    {
        cv::Mat channels[4], canvas;
        cv::normalize(mag, channels[0], 0, 1, cv::NORM_MINMAX, CV_32F);
        cv::normalize(theta, channels[1], 0, 1, cv::NORM_MINMAX, CV_32F);
        cv::normalize(dx, channels[2], 0, 1, cv::NORM_MINMAX, CV_32F);
        cv::normalize(dy, channels[3], 0, 1, cv::NORM_MINMAX, CV_32F);
        cv::vconcat(channels, 4, canvas);
        return canvas;
    }
};

static cv::Mat getImage(ogles_gpgpu::ProcInterface &proc)
{
    cv::Mat result(proc.getOutFrameH(), proc.getOutFrameW(), CV_8UC4);
    proc.getResultData(result.ptr());
    return result;
}

#define DISPLAY_OUTPUT 1

/*
 * Fixture tests
 */


static void _quiver(cv::Mat3b &canvas, const cv::Mat1f &dx, const cv::Mat1f &dy, int step, float scale, const cv::Mat1b &mask)
{
    assert(dx.size() == dy.size());

    float zoom = float(canvas.cols) / dx.cols;
    for(int y = 0; y < dx.rows; y += step)
    {
        for(int x = 0; x < dx.cols; x += step)
        {
            if(mask(y,x))
            {
                cv::Point2f v1(dx(y,x), dy(y,x));
                cv::Point2f p1(x, y);

                cv::Point2f p2 = p1 * zoom;
                cv::Point2f v2 = (v1 * zoom) * scale;

                cv::circle(canvas, p2, 1, CV_RGB(0,255,255), -1, CV_AA);
                cv::arrowedLine(canvas, p2, p2 + v2, CV_RGB(0,255,0), 1, CV_AA);
            }
        }
    }
}

TEST_F(ACFTest, fir3)
{
    ogles_gpgpu::VideoSource video;

    {
        const bool doRgb = false;

        ogles_gpgpu::NoopProc noopProc;
        ogles_gpgpu::GaussOptProc smoothProc(2);
        ogles_gpgpu::MedianProc medianProc;
        ogles_gpgpu::Fir3Proc fir3Proc(doRgb);
        ogles_gpgpu::FIFOPRoc fifoProc(3);

        noopProc.setOutputSize(0.125);

        fir3Proc.setWeights({-0.25, 0.5, -0.25}); // laplacian:

        //const float s = 0.333;
        //ogles_gpgpu::Vec3f f1(+0.50*s, -0.25*s, -0.25*s);
        //ogles_gpgpu::Vec3f f2(-0.25*s, +0.50*s, -0.25*s);
        //ogles_gpgpu::Vec3f f3(-0.25*s, -0.25*s, +0.50*s);
        //fir3Proc.setWeights(f3, f2, f1);

        video.set(&noopProc);
        noopProc.add(&smoothProc);
        smoothProc.add(&medianProc);
        medianProc.add(&fifoProc);

        //  (newest) RGB....RGB....RGB (oldest)
        fifoProc.addWithDelay(&fir3Proc, 0, 0);
        fifoProc.addWithDelay(&fir3Proc, 1, 1);
        fifoProc.addWithDelay(&fir3Proc, 2, 2);

        bool isFirst = true;

        {
            // Run from video:
            cv::VideoCapture source(0);
            cv::Mat frame;
            source >> frame;

            int frameIndex = 0, channelIndex = 0;
            while(true)
            {
                frameIndex++;

                source >> frame;
                cv::cvtColor(frame, frame, cv::COLOR_BGR2BGRA);

                std::stringstream ss;
                ss << frameIndex;
                cv::putText(frame, ss.str(), {20, 20}, CV_FONT_HERSHEY_COMPLEX, 1.0, {0,255,0});

                video({frame.cols, frame.rows}, frame.ptr(), true, 0, DFLT_TEXTURE_FORMAT);

                if(fifoProc.isFull())
                {
                    cv::Mat response = getImage( fir3Proc ); // fir3Proc );

                    std::vector<cv::Mat> channels;
                    cv::split(frame, channels);

                    int bin = frameIndex % 30;
                    if((bin < 6) && !(bin % 2))
                    {
                        if(bin == 0)
                        {
                            channelIndex = 0;
                        }
                        channels[(channelIndex % 3)].setTo(255);
                        channelIndex ++;
                    }

                    if(0)
                    {
                        cv::Mat images[3], canvas;
                        images[0] = getImage(*fifoProc[0]); // t-2
                        images[1] = getImage(*fifoProc[1]); // t-1
                        images[2] = getImage(*fifoProc[2]); // t
                        cv::hconcat(images, 3, canvas);
                        cv::imshow("canvas", canvas);
                        cv::waitKey(0);
                    }

                    cv::merge(channels, frame); // BGRA

                    //cv::hconcat(frame, response, response);
                    cv::imshow("frame", frame);
                    cv::imshow("response", response);
                    cv::waitKey(10);
                }
            }
        }
    }
}

TEST_F(ACFTest, flow_opt)
{
    ogles_gpgpu::VideoSource video;


#if 1
    const float scale = 2.0f;
    ogles_gpgpu::Flow2Pipeline flow;
#else
    const float scale = 1.0f;
    ogles_gpgpu::FlowPipeline flow;
#endif


    ogles_gpgpu::RemapProc remapProc;
    flow.getInputFilter()->add(&remapProc, 1);
    flow.getOutputFilter()->add(&remapProc, 0);

    cv::Mat image1;
    cv::resize(image, image1, {image.cols/8, image.rows/8});

    video.set(&flow);
    video({image1.cols, image1.rows}, image1.ptr(), true, 0, DFLT_TEXTURE_FORMAT);

    for(int a = 1; a < 10; a++)
    {
        float i = float(a)/10.0;

        // Generate known motion and compare with recovered flow:
        float theta = i;
        const float ct = std::cos(theta * M_PI/180.0);
        const float st = std::sin(theta * M_PI/180.0);
        const cv::Point2f t(i, i);
        const cv::Point2f c(image.cols/2, image.rows/2);
        const cv::Matx33f T1(1,0,c.x+t.x,0,1,c.y+t.y,0,0,1), T2(1,0,-c.x,0,1,-c.y,0,0,1);
        const cv::Matx33f R(+ct,-st,0,+st,+ct,0,0,0,1);
        const cv::Matx33f H = T1 * R * T2;

        // Create the ground truth flow field
        cv::Mat1f px(image1.size()), py(image1.size());
        cv::Mat1f fx(image1.size()), fy(image1.size()), ix(image1.size()), iy(image1.size());
        for(int y = 0; y < fy.rows; y++)
        {
            for(int x = 0; x < fy.cols; x++)
            {
                cv::Point3f q = H * cv::Point3f(x, y, 1.f);
                fx(y,x) = q.x/q.z; // absolute
                fy(y,x) = q.y/q.z;
                ix(y,x) = float(x) - fx(y,x); // relative
                iy(y,x) = float(y) - fy(y,x);
                px(y,x) = x;
                py(y,x) = y;
            }
        }

        cv::Mat gt;
        cv::hconcat(ix, iy, gt);

        cv::Mat image2;
        cv::remap(image1, image2, fx, fy, cv::INTER_LINEAR);
        video({image2.cols, image2.rows}, image2.ptr(), true, 0, DFLT_TEXTURE_FORMAT);

        cv::Mat flow8U = getImage(flow), flow32F;
        flow8U.convertTo(flow32F, CV_32FC4, ((scale*2.0)/255.0), -scale);

        std::vector<cv::Mat> channels;
        cv::split(flow32F, channels);
        cv::Mat1f dx = channels[2];
        cv::Mat1f dy = channels[1];
        cv::Mat1f strength;
        cv::magnitude(dx, dy, strength);
        cv::Mat1b mask = (strength > 0.1);

        auto mux = cv::mean(dx);
        auto muy = cv::mean(dy);
        std::cout << "EST: " << mux << " - " << muy << std::endl;

        cv::Mat ex = cv::abs(dx - ix);
        cv::Mat ey = cv::abs(dy - iy);

        mux = cv::mean(ix);
        muy = cv::mean(iy);
        std::cout << "GT: " << mux << " - " << muy << std::endl;

        cv::imshow("ex", ex);
        cv::imshow("ey", ey);

        // ==== DISPLAY =====

        cv::Mat3b canvas;
        cv::cvtColor(image, canvas, cv::COLOR_BGRA2BGR);
        _quiver(canvas, dx, dy, 8, 20.0, mask);
        cv::Mat xy, uv;
        cv::hconcat(channels[2], channels[1], xy);
        cv::hconcat(channels[0], channels[3], uv);
        cv::imshow("UV", uv);
        cv::imshow("quiver", canvas);
        cv::imshow("xy", (xy * 0.25) + 0.5);
        cv::imshow("truth", (gt * 0.25) + 0.5);

        {
            cv::Mat tmp = getImage(remapProc);
            cv::imshow("tmp", tmp);

            int j = 0;
            cv::Mat images[2] = { image1, tmp };
            do
            {
                cv::imshow("gpu-warp", images[j++%2]);
            }
            while(cv::waitKey(0) != int('q'));
        }

        cv::Mat imagea;
        cv::remap(image1, imagea, (-dx + px), (-dy + py), cv::INTER_LINEAR);
        {
            int j = 0;
            cv::Mat images[3] = { image1, image2, imagea };
            do
            {
                cv::imshow("image", images[j++%3]);
            }
            while(cv::waitKey(0) != int('q'));
        }

    }
}

TEST_F(ACFTest, iir)
{
    ogles_gpgpu::VideoSource video;

    {
        ogles_gpgpu::NoopProc noopProc;

#if 0
        ogles_gpgpu::LowPassFilterProc lpfProc(0.5);
#else
        ogles_gpgpu::HighPassFilterProc lpfProc(0.9);
#endif
        noopProc.add(&lpfProc);
        video.set(&noopProc);

        {
            // Run from video:
            cv::VideoCapture source(0);
            while(true)
            {
                cv::Mat canvas, frame;
                source >> canvas;
                cv::cvtColor(canvas, frame, cv::COLOR_BGR2BGRA);
                video({frame.cols, frame.rows}, frame.ptr(), true, 0, DFLT_TEXTURE_FORMAT);

                cv::Mat image = getImage(lpfProc);
                cv::imshow("image", image);
                cv::waitKey(10);
            }
        }
    }
}

TEST_F(ACFTest, gauss_opt)
{
    ogles_gpgpu::VideoSource video;

    if(1)
    {
        ogles_gpgpu::GaussOptProc gaussProc(4.0);
        video.set(&gaussProc);

        video({image.cols, image.rows}, image.ptr(), true, 0, DFLT_PIX_FORMAT);
        cv::Mat smooth = getImage(gaussProc);

        cv::Mat images[2] = { image, smooth };
        int i = 0;
        do
        {
            cv::imshow("smooth", images[i++%2]);
        }
        while(cv::waitKey(0) != int('q'));
    }
}

#if 0
TEST_F(ACFTest, gauss_opt)
{
    cv::Mat smooth5, smooth7;
    {
        ogles_gpgpu::GaussProc gaussProc(ogles_gpgpu::GaussProcPass::k5Tap, false);
        ogles_gpgpu::VideoSource video;
        video.set(&gaussProc);
        video({image.cols, image.rows}, image.ptr(), true, 0, DFLT_TEXTURE_FORMAT);
        gaussProc.getMemTransferObj()->setOutputPixelFormat(DFLT_TEXTURE_FORMAT);
        smooth5 = getImage(gaussProc);
    }

    {
        ogles_gpgpu::GaussProc gaussProc(ogles_gpgpu::GaussProcPass::k7Tap, false);
        ogles_gpgpu::VideoSource video;
        video.set(&gaussProc);
        video({image.cols, image.rows}, image.ptr(), true, 0, DFLT_TEXTURE_FORMAT);
        gaussProc.getMemTransferObj()->setOutputPixelFormat(DFLT_TEXTURE_FORMAT);
        smooth7 = getImage(gaussProc);
    }

    cv::imshow("diff", smooth7-smooth5);

    int i = 0;
    cv::Mat clip[] = { smooth5, smooth7 };
    do
    {
        cv::imshow("smooth", clip[i++%2]);
    }
    while(cv::waitKey(0) != int('q'));
}
#endif

/*

TEST_F(ACFTest, gauss_opt)
{
    ogles_gpgpu::VideoSource video;
    ogles_gpgpu::NoopProc noop;
    ogles_gpgpu::GaussOptProc gaussOptProc(7, false);

    video.set(&noop);
    noop.add(&gaussOptProc);

    video({image.cols, image.rows}, image.ptr(), true, 0, DFLT_TEXTURE_FORMAT);

    gaussOptProc.getMemTransferObj()->setOutputPixelFormat(DFLT_TEXTURE_FORMAT);
    cv::Mat after = getImage(gaussOptProc);

#if DISPLAY_OUTPUT
    cv::hconcat(image, after, after);
    cv::imshow("before_after", after);
#endif
}
*/

// GradProc delivers:
// clamp(mag, 0.0, 1.0), theta/3.14159, clamp(dx, 0.0, 1.0), clamp(dy, 0.0, 1.0))

static void printStats(const cv::Mat &I, const std::string &tag)
{
    cv::Vec2d vals;
    cv::minMaxLoc(I, &vals[0], &vals[1]);
    std::cout << tag << " mean = " << cv::mean(I) << " min = " << vals[0] << " max = " << vals[1] << std::endl;
}

static void normshow(const std::string &tag, const cv::Mat &I)
{
    printStats(I, tag);
    cv::Mat canvas;
    cv::normalize(I, canvas, 0, 1, cv::NORM_MINMAX, CV_32F);
    cv::imshow(tag, canvas);
}

static void compareLUV(const cv::Mat &LUVp, const cv::Mat &LUV_)
{
    assert(LUV_.channels() >= 3);

    cv::Mat LUVpu8;
    LUVp.convertTo(LUVpu8, CV_8UC3, 255.0);

    cv::Mat L_, U_, V_;
    cv::extractChannel(LUV_, L_, 0);
    cv::extractChannel(LUV_, U_, 1);
    cv::extractChannel(LUV_, V_, 2);

    std::vector<cv::Mat> vLUVh_ = { L_, U_, V_};
    cv::Mat LUVh_;
    cv::hconcat(vLUVh_, LUVh_);
    cv::imshow("LUVp", LUVpu8);
    cv::imshow("LUV_shader", LUVh_);
}

#define DO_TESTING 0

TEST_F(ACFTest, acf)
{
    const char *classifier = "/Users/dhirvonen/devel/drishtisdk//assets/all_tight_face_48x48_v2Detector.mat";

    // #### CPU ####
    cv::Mat I;
    cv::cvtColor(image, I, cv::COLOR_BGR2RGB);
    I.convertTo(I, CV_32FC3, (1.0/255.0));
    CV_Assert(I.type() == CV_32FC3);

    MatP Ip( m_toTranspose(I) );
    drishti::acf::Detector detector(classifier);
    detector.setIsTranspose(true);

#if  DO_TESTING
    {
        // Test CPU detection
        std::vector<double> scores;
        std::vector<cv::Rect> objects;
        detector(Ip, objects, &scores);

        cv::Mat canvas = image.clone();
        for(auto &r : objects)
        {
            cv::rectangle(canvas, r, {0,255,0}, 1, 8);
        }
        cv::imshow("acf_cpu_detection", canvas);
        //cv::waitKey(0);
    }
#endif

    // Pull out the ACF intermediate results from the logger:
    cv::Mat M, Mnorm, O, L, U, V, H;
    std::function<int(const cv::Mat &, const std::string &)> logger = [&](const cv::Mat &I, const std::string &tag) -> int
    {
        if(tag.find("L:") != std::string::npos)
        {
            L = I.t();
        }
        else if(tag.find("U:") != std::string::npos)
        {
            U = I.t();
        }
        else if(tag.find("V:") != std::string::npos)
        {
            V = I.t();
        }
        else if(tag.find("Mnorm:") != std::string::npos)
        {
            Mnorm = I.t();
        }
        else if(tag.find("M:") != std::string::npos)
        {
            M = I.t();
        }
        else if(tag.find("O:") != std::string::npos)
        {
            O = I.t();
        }
        else if(tag.find("H:") != std::string::npos)
        {
            H = I.t();
        }
        return 0;
    };
    detector.setLogger(logger);

    drishti::acf::Detector::Pyramid Pcpu;

    std::vector<double> scales;
    std::vector<cv::Size2d> scaleshw;
    std::vector<ogles_gpgpu::Size2d> sizes;
    {
        INIT_TIMER;
        detector.computePyramid(Ip, Pcpu);
        scales = Pcpu.scales;
        scaleshw = Pcpu.scaleshw;
        for(int i = 0; i < Pcpu.nScales; i++)
        {
            const auto size = Pcpu.data[i][0][0].size();
            sizes.emplace_back(size.width * 4, size.height * 4);
        }

        {
            for(auto &s : sizes)
            {
                std::swap(s.width, s.height);
            }
        }

        STOP_TIMER("acf_cpu");
    }

    {
        INIT_TIMER;
        MatP Ich;
        detector.computeChannels(Ip, Ich, logger);
        STOP_TIMER("acf_cpu");

        cv::Mat d = Ich.base().clone(), canvas;
        canvas = d.t();
        cv::imshow("acf_cpu_native", d);
        cv::imshow("acf_cpu", canvas);
    }

    // #### GPU #####
    ogles_gpgpu::Size2d inputSize(image.cols, image.rows);
    const bool doGrayscale = false;
    const bool doCorners = false;
    ogles_gpgpu::ACF video(nullptr, inputSize, sizes, doGrayscale, doCorners, false); // alias

    video.rgb2luvProc.setOutputRenderOrientation(ogles_gpgpu::RenderOrientationDiagonalFlipped);

    using TagTime = std::pair<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>>;
    std::vector<TagTime> times;
    std::function<void(const std::string &)> timer = [&](const std::string &name)
    {
        times.emplace_back(name, std::chrono::high_resolution_clock::now());
    };

    cv::Mat acf;
    {
        // #### GPU #####
        cv::Mat input = image;

#define WARP_UP_GPU 1
#if WARP_UP_GPU
        for(int i = 0; i < 10; i++)
        {
            video({input.cols, input.rows}, input.ptr(), true, 0, DFLT_TEXTURE_FORMAT);
        }
#endif

        video.setLogger(timer);
        video({input.cols, input.rows}, input.ptr(), true, 0, DFLT_TEXTURE_FORMAT);
        acf = video.getChannels();
    }

    cv::imshow("acf", acf);
    cv::waitKey(0);
    //cv::imwrite("/tmp/acf.png", acf);

    auto crops = video.getCropRegions();


    {
        // Draw crop regions:
        cv::Mat canvas;
        cv::cvtColor(acf, canvas, cv::COLOR_GRAY2BGR);
        for(auto &l : crops)
        {
            for(auto &r : l)
            {
                //cv::imshow("region", canvas(r));
                cv::rectangle(canvas, {r.x, r.y, r.width, r.height}, {0,255,0}, 1, 8);
            }
        }
        cv::imshow("canvas", canvas);
        cv::waitKey(0);
    }

    drishti::acf::Detector::Pyramid Pgpu;
    Pgpu.pPyramid = detector.opts.pPyramid;
    Pgpu.nTypes = video.getChannelCount();
    Pgpu.nScales = scales.size();
    Pgpu.scales = scales;
    Pgpu.scaleshw = scaleshw;
    Pgpu.rois.resize(crops.size());
    for(int i = 0; i < crops.size(); i++)
    {
        Pgpu.rois[i].resize(crops[i].size());
        for(int j = 0; j < crops[i].size(); j++)
        {
            const auto &r = crops[i][j];
            Pgpu.rois[i][j] = cv::Rect(r.x, r.y, r.width, r.height);
        }
    }
    video.fill(Pgpu);

#define COMPARE_CPU_GPU_ORIENTATION 0
#if COMPARE_CPU_GPU_ORIENTATION
    {

        {
            // Draw crop regions:
            cv::Mat canvas;
            cv::cvtColor(acf, canvas, cv::COLOR_GRAY2BGR);
            for(auto &l : Pgpu.rois)
            {
                for(auto &r : l)
                {
                    //cv::imshow("region", canvas(r));
                    cv::rectangle(canvas, r, {0,255,0}, 1, 8);
                }
            }
            cv::imshow("canvas", canvas);
        }

        // ##### MO ######
        cv::Mat XOMY = video.getImage(video.gradProc);;
        cv::Mat Ogpu, Ocpu = O * (1.0/M_PI);
        cv::extractChannel(XOMY, Ogpu, 1);

        cv::Mat h;
        cv::Mat ha = video.getImage(video.gradHistProcAOut);
        cv::Mat hb = video.getImage(video.gradHistProcBOut);
        std::vector<cv::Mat> vha, vhb;
        cv::split(ha, vha);
        cv::swap(vha[0], vha[2]);
        cv::split(hb, vhb);
        cv::swap(vhb[0], vhb[2]);
        std::copy(vhb.begin(), vhb.end(), std::back_inserter(vha));
        cv::vconcat(vha, h);
        cv::normalize(h, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::imshow("h", h);

        cv::Vec2d val;
        cv::minMaxLoc(Ocpu, &val[0], &val[1]);
        std::cout << "min/max: " << val << std::endl;

        cv::imshow("Ogpu", Ogpu);
        cv::imshow("Ocup", Ocpu);
        cv::waitKey(0);
    }
#endif


#define COMPARE_CPU_GPU_CHANNELS 0
#if COMPARE_CPU_GPU_CHANNELS
    // Compare Pcpu <-> Pgpu
    for(int i = 0; i < Pgpu.nScales; i++)
    {
        cv::Mat Ccpu, Cgpu;
        cv::hconcat(Pgpu.data[i][0].get(), Cgpu);
        cv::vconcat(Pcpu.data[i][0].get(), Ccpu);
        Ccpu = Ccpu.t();
        cv::imshow("Ccpu", Ccpu);
        cv::imshow("Cgpu", Cgpu);
        cv::waitKey(0);
    }
#endif

    {
        // Test GPU ACF detection
        // Fill in ACF Pyramid structure

        {
            // Perform modification
            drishti::acf::Detector::Modify dflt;
            dflt.cascThr = { "cascThr", -1.0 };
            dflt.cascCal = { "cascCal", +0.002 };
            detector.acfModify(dflt);
        }

        std::vector<double> scores;
        std::vector<cv::Rect> objects;
        detector(Pgpu, objects, &scores);

        cv::Mat canvas =  m_toUpright(image);
        for(auto &r : objects)
        {
            cv::rectangle(canvas, r, {0,255,0}, 1, 8);
        }
        cv::imshow("acf_gpu_detection", canvas);
    }

    cv::waitKey(0);

    double total = 0.0;
    for(int i = 1; i < times.size(); i++)
    {
        double elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(times[i].second - times[i-1].second).count();
        total += elapsed;
        if(times[i].first.find(" end") != std::string::npos)
        {
            std::cout << times[i].first << " " << std::fixed << elapsed << " (" << total << ")" << std::endl;
        }
    }

    //acf = video.getChannels();
    //std::cout << "acf.size() = " << acf.size() << std::endl;
    //cv::imshow("acf", acf);

#if 0
    {
        // ##### MO ######
        cv::Mat XOMY = video.getImage(video.gradProc);;
        std::vector<cv::Mat> MOXY;
        cv::split(XOMY, MOXY);
        cv::imshow("O", MOXY[1]);
        cv::waitKey(0);
    }
#endif


#if 0
    cv::Mat pyramid = getImage(video.pyramidProc);
    cv::imshow("pyramid", pyramid);

    // ##### LUV #######
    cv::Mat LUVp, LUV, VUL = getImage(video.smoothProc);
    cv::cvtColor(VUL, LUV, cv::COLOR_BGRA2RGBA);
    std::vector<cv::Mat> vLUV { L, U, V };
    cv::hconcat(vLUV, LUVp);
    compareLUV(LUVp, LUV);

    // #### MOXY ####
    cv::Mat XOMY;
    std::vector<cv::Mat> MOXY;

    // ## M ##
    cv::Mat _M, _O;
    cv::normalize(M, _M, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::hconcat(MOXY[0], _M, _M);
    cv::imshow("M_shader_acf", _M);

    // ## O ##
    cv::normalize(O, _O, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::hconcat(MOXY[1], _O, _O);
    cv::imshow("O_shader_acf", _O);

    // ## Mnorm ##
    XOMY = getImage(video.normProcOut);

    cv::Mat _Mnorm, tmp;
    cv::normalize(Mnorm, _Mnorm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::split(XOMY, MOXY);
    std::swap(MOXY[0], MOXY[2]);
    cv::normalize(MOXY[0], tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("Mnorm_acf", _Mnorm);
    cv::imshow("Mnorm_shader", tmp);

    // ## H ##
    cv::Mat ha = getImage(video.gradHistProcAOut);
    cv::Mat hb = getImage(video.gradHistProcBOut);
    std::vector<cv::Mat> vha, vhb;
    cv::split(ha, vha);
    cv::swap(vha[0], vha[2]);
    cv::split(hb, vhb);
    cv::swap(vhb[0], vhb[2]);
    std::copy(vhb.begin(), vhb.begin()+2, std::back_inserter(vha));
    cv::Mat h;
    cv::vconcat(vha, h);
    cv::normalize(h, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    cv::Mat _H;
    cv::normalize(H, _H, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("H_acf", _H);
    cv::imshow("H_shader", tmp);
#endif

    cv::waitKey(0);
}

TEST_F(ACFTest, luv)
{
    ogles_gpgpu::VideoSource video;
    ogles_gpgpu::Rgb2LuvProc rgb2luvProc;

    rgb2luvProc.setOutputSize(0.25); // 1/4 binning

    cv::Mat input = image;

    video.set(&rgb2luvProc);
    video({input.cols, input.rows}, input.ptr(), true, 0, DFLT_TEXTURE_FORMAT);

    rgb2luvProc.getMemTransferObj()->setOutputPixelFormat(DFLT_TEXTURE_FORMAT);
    cv::Mat result = getImage(rgb2luvProc);

#if DISPLAY_OUTPUT
    std::vector<cv::Mat> channels;
    cv::split(result, channels);
    cv::swap(channels[0], channels[2]);
    cv::hconcat(channels, result);
    cv::imshow("luv", result);
    cv::waitKey(0);
#endif
}

END_EMPTY_NAMESPACE
