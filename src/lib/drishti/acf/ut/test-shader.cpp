/*!
  @file   test-shader.cpp
  @author David Hirvonen
  @brief  CPU ACF shader tests using a google test fixture.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file has various tests for comparing GPU ACF output with the
  reference CPU ACF output.  This is a WIP and there is currently 
  liberal use of cv::imshow() for visualization, etc.  This needs to
  be automated and reasonable tolerances on GPU vs CPU discrepancies 
  need to be established.

*/

// https://code.google.com/p/googletest/wiki/Primer

#define DRISHTI_ACF_TEST_DISPLAY_OUTPUT 0
#define DRISHTI_ACF_TEST_COMPARE_CPU_GPU_ORIENTATION 0
#define DRISHTI_ACF_TEST_COMPARE_CPU_GPU_CHANNELS 0

#if DRISHTI_BUILD_OGLES_GPGPU && DRISHTI_BUILD_QT
// TODO: (needs more work to be 1)
#  define DO_ACF_GPU_TEST 0
#else
#  define DO_ACF_GPU_TEST 0
#endif

#define DRISHTI_ACF_TEST_BOOST 1
#define DRISHTI_ACF_TEST_CEREAL 1

#if DRISHTI_ACF_TEST_CEREAL
// http://uscilab.github.io/cereal/serialization_archives.html
#  include <cereal/archives/portable_binary.hpp>
#  include <cereal/types/vector.hpp>
#endif

// #!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!
// #!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!
// #!#!#!#!#!#!#!#!#!#!#!#!#!#!# Work in progress !#!#!#!#!#!#!#!#!#!#!#!#!
// #!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!
// #!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!

#include <gtest/gtest.h>

#include "drishti/core/drawing.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/acf/ACF.h"
#include "drishti/acf/MatP.h"
#include "drishti/core/Logger.h"
#include "drishti/geometry/Primitives.h"

#if DO_ACF_GPU_TEST
#  include "drishti/acf/GPUACF.h"
#endif

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <memory>

const char* imageFilename;
const char* truthFilename;
const char* modelFilename;
const char* outputDirectory;

#ifdef ANDROID
#  define DFLT_TEXTURE_FORMAT GL_RGBA
#else
#  define DFLT_TEXTURE_FORMAT GL_BGRA
#endif

#include <iostream>
#include <chrono>

#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

BEGIN_EMPTY_NAMESPACE

struct WaitKey
{
    WaitKey() {}
    ~WaitKey()
    {
#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
        cv::waitKey(0);
#endif
    }
};

class ACFTest : public ::testing::Test
{
protected:

    bool m_hasTranspose = false;

    // Setup
    ACFTest()
    {
        m_logger = drishti::core::Logger::create("test-acf");

        // Load the ground truth data:

        image = loadImage(imageFilename);
        if(m_hasTranspose)
        {
            image = image.t();
        }

        // TODO: we need to load ground truth output for each shader
        // (some combinations could be tested, but that is probably excessive!)
        //truth = loadImage(truthFilename);

#if DO_ACF_GPU_TEST
        m_context = std::make_shared<QGLContext>();
#endif
    }

    // Cleanup
    virtual ~ACFTest()
    {
        drishti::core::Logger::drop("test-drishti-acf");
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

#if DO_ACF_GPU_TEST
    std::shared_ptr<QGLContext> m_context;
#endif
    
    std::shared_ptr<spdlog::logger> m_logger;

    // Test images:
    cv::Mat image, truth;
};


#if DO_ACF_GPU_TEST
static cv::Mat getImage(ogles_gpgpu::ProcInterface &proc)
{
    cv::Mat result(proc.getOutFrameH(), proc.getOutFrameW(), CV_8UC4);
    proc.getResultData(result.ptr());
    return result;
}
#endif // DO_ACF_GPU_TEST

// This is a WIP, currently we test the basic CPU detection functionality
// with a sample image.  Given the complexity of the GPU implementation,
// more tests will need to be added for lower level channel computation,
// such as CPU vs GPU error bounds.  For now the simple detection success
// test and a simple placeholder assert(true) test wil be added at the
// end of the test.

// http://stackoverflow.com/a/32647694
bool isEqual(const cv::Mat& a, const cv::Mat& b)
{
    cv::Mat temp;
    cv::bitwise_xor(a,b,temp); //It vectorizes well with SSE/NEON
    return !(cv::countNonZero(temp) );
}

bool isEqual(const drishti::acf::Detector &a, const drishti::acf::Detector &b)
{
    if(!isEqual(a.clf.fids, b.clf.fids))
    {
        std::cout << cv::Mat1b(a.clf.fids == b.clf.fids) << std::endl;

        cv::Mat tmp;
        cv::hconcat(a.clf.fids, b.clf.fids, tmp);
        std::cout << tmp << std::endl;
    }
    
    if(!isEqual(a.clf.child, b.clf.child))
    {
        std::cout << cv::Mat1b(a.clf.child == b.clf.child) << std::endl;
        
        cv::Mat tmp;
        cv::hconcat(a.clf.fids, b.clf.fids, tmp);
        std::cout << tmp << std::endl;
    }
    
    if(!isEqual(a.clf.depth, b.clf.depth))
    {
        std::cout << cv::Mat1b(a.clf.depth == b.clf.depth) << std::endl;
        
        cv::Mat tmp;
        cv::hconcat(a.clf.fids, b.clf.fids, tmp);
        std::cout << tmp << std::endl;
    }
    
    return // The float -> uint16_t -> float will not be an exact match
        isEqual(a.clf.fids, b.clf.fids) &&
        isEqual(a.clf.child, b.clf.child) &&
        isEqual(a.clf.depth, b.clf.depth);
        //isEqual(a.clf.thrs, b.clf.thrs) &&
        //isEqual(a.clf.hs, b.clf.hs) &&
        //isEqual(a.clf.weights, b.clf.weights) &&
}

#if DRISHTI_ACF_TEST_BOOST
TEST_F(ACFTest, ACFSerializeBoost)
{
    // Load from cvmat
    drishti::acf::Detector detector(modelFilename), detector2;
    
    std::string filename = outputDirectory;
    filename += "/acf.pba.z";
    
    // Write to pba.z
    save_pba_z(filename, detector);
    
    // Load from pba.z
    load_pba_z(filename, detector2);
    
    ASSERT_TRUE(isEqual(detector, detector2));
}
#endif // DRISHTI_ACF_TEST_BOOST

#if DRISHTI_ACF_TEST_CEREAL
TEST_F(ACFTest, ACFSerializeCereal)
{
    // Load from cvmat
    drishti::acf::Detector detector(modelFilename), detector2;
    
    std::string filename = outputDirectory;
    filename += "/acf.pba.c";
    
    { // save
        std::ofstream file(filename, std::ios::binary);
        if(file)
        {
            cereal::PortableBinaryOutputArchive3 ar(file);
            ar(detector);
        }
    }
    
    { // load
        std::ifstream file(filename, std::ios::binary);
        if(file)
        {
            cereal::PortableBinaryInputArchive3 ar(file);
            ar(detector2);
        }
    }
    
    ASSERT_TRUE(isEqual(detector, detector2));
}
#endif // DRISHTI_ACF_TEST_CEREAL

TEST_F(ACFTest, ACFDetection)
{
    const char *classifier = modelFilename;
    
    WaitKey waitKey;
    
    drishti::acf::Detector detector(classifier);
    
    // #### CPU ####
    cv::Mat I;
    cv::cvtColor(image, I, cv::COLOR_BGR2RGB);
    I.convertTo(I, CV_32FC3, (1.0/255.0));
    CV_Assert(I.type() == CV_32FC3);
    
    {  // Test CPU detection w/ cv::Mat
        std::vector<double> scores;
        std::vector<cv::Rect> objects;
        detector.setIsTranspose(m_hasTranspose);
        detector(I, objects, &scores);
        
#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
        cv::Mat canvas = image.clone();
        for(auto &r : objects)
        {
            cv::rectangle(canvas, r, {0,255,0}, 1, 8);
        }
        cv::imshow("acf_cpu_detection", canvas);
#endif
        
        ASSERT_GT(objects.size(), 0); // Very weak test!!!
    }

    // Be sure to construct planar images in transpose:
    MatP Ip( m_hasTranspose ? I : I.t() );

    {  // Test CPU detection w/ MatP planar format
        std::vector<double> scores;
        std::vector<cv::Rect> objects;
        
        detector.setIsTranspose(true);
        detector(Ip, objects, &scores);

#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
        cv::Mat canvas = m_hasTranspose ? image : image.t();
        for(auto &r : objects)
        {
            cv::rectangle(canvas, r, {0,255,0}, 1, 8);
        }
        cv::imshow("acf_cpu_detection", canvas);
#endif
        
        ASSERT_GT(objects.size(), 0); // Very weak test!!!
    }

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
    
#if DO_ACF_GPU_TEST
    std::vector<ogles_gpgpu::Size2d> sizes;
#endif
    
    {
        detector.computePyramid(Ip, Pcpu);
        scales = Pcpu.scales;
        scaleshw = Pcpu.scaleshw;
        
#if DO_ACF_GPU_TEST
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
#endif
        
    }

    {
        MatP Ich;
        detector.computeChannels(Ip, Ich, logger);
        cv::Mat d = Ich.base().clone(), canvas;
        canvas = d.t();
        //cv::imshow("acf_cpu_native", d);
        //cv::imshow("acf_cpu", canvas);
    }

#if DO_ACF_GPU_TEST
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

    auto crops = video.getCropRegions();
    
#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
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
    }
#endif // DRISHTI_ACF_TEST_DISPLAY_OUTPUT

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

#if DRISHTI_ACF_TEST_COMPARE_CPU_GPU_ORIENTATION && DRISHTI_ACF_TEST_DISPLAY_OUTPUT
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
    }
#endif // COMPARE_CPU_GPU_ORIENTATION

#if DRISHTI_ACF_TEST_COMPARE_CPU_GPU_CHANNELS && DRISHTI_ACF_TEST_DISPLAY_OUTPUT
    // Compare Pcpu <-> Pgpu
    for(int i = 0; i < Pgpu.nScales; i++)
    {
        cv::Mat Ccpu, Cgpu;
        cv::hconcat(Pgpu.data[i][0].get(), Cgpu);
        cv::vconcat(Pcpu.data[i][0].get(), Ccpu);
        Ccpu = Ccpu.t();
        cv::imshow("Ccpu", Ccpu);
        cv::imshow("Cgpu", Cgpu);
    }
#endif // COMPARE_CPU_GPU_CHANNELS

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

#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
        cv::Mat canvas =  image.clone();
        for(auto &r : objects)
        {
            cv::rectangle(canvas, r, {0,255,0}, 1, 8);
        }
        cv::imshow("acf_gpu_detection", canvas);
#endif // DRISHTI_ACF_TEST_DISPLAY_OUTPUT
    }

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

#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
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

#endif // DRISHTI_ACF_TEST_DISPLAY_OUTPUT
#endif // DO_ACF_GPU_TEST (fin)
    
    ASSERT_TRUE(true); // Weak test (placeholder), make sure we reach here
}

END_EMPTY_NAMESPACE
