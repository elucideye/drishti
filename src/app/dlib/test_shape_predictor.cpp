// This example program shows how to use dlib's implementation of the paper:
//    One Millisecond Face Alignment with an Ensemble of Regression Trees by
//    Vahid Kazemi and Josephine Sullivan, CVPR 2014
//
// http://www.csc.kth.se/~vahidk/papers/KazemiCVPR14.pdf
//

#include "drishti/core/drishti_stdlib_string.h" // FIRST!

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dlib/data_io.h>
#include <dlib/image_transforms/assign_image.h>
#include <dlib/statistics/statistics.h>
#include <dlib/image_processing/shape_predictor.h>
#include <dlib/opencv/cv_image.h>
#include <dlib/vectorstream.h>
#include <dlib/serialize.h>
#include <dlib/data_io/load_image_dataset.h>
#include <dlib/opencv.h>

#include "drishti/core/Line.h"
#include "drishti/core/string_utils.h"
#include "drishti/ml/shape_predictor_archive.h"
#include "drishti/testlib/drishti_cli.h"

#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cv_cereal.h"

#include "cxxopts.hpp"

#include <iostream>

std::vector<std::vector<double>>
get_interocular_distances(const std::vector<std::vector<dlib::full_object_detection>>& objects);

cv::Point cv_point(const dlib::point& p)
{
    return cv::Point(p.x(), p.y());
}

cv::Rect cv_rect(const dlib::rectangle& r)
{
    return cv::Rect(cv_point(r.tl_corner()), cv_point(r.br_corner()));
}

dlib::rectangle dlib_rect(const cv::Rect& r)
{
    return dlib::rectangle(r.x, r.y, r.br().x, r.br().y);
}

dlib::point dlib_point(const cv::Point& p)
{
    return dlib::point(p.x, p.y);
}

int mine(int argc, char* argv[])
{
    const auto argumentCount = argc;

    bool doPreview = false;
    bool doThreads = false;
    bool doVerbose = false;
    bool doHelp = false;
    std::string sInput;
    std::string sModel;

    cxxopts::Options options("train_shape_predictor", "Command line interface for dlib shape_predictor training");

    // clang-format off
    options.add_options()
        ( "input", "Input filename list", cxxopts::value<std::string>(sInput))
        ( "model", "Model filename", cxxopts::value<std::string>(sModel))
        ( "preview", "Use preview window", cxxopts::value<bool>(doPreview))
        ( "threads", "Use worker threads when possible", cxxopts::value<bool>(doThreads))
        ( "verbose", "Print verbose diagnostics", cxxopts::value<bool>(doVerbose))
        ( "help", "Print the help message", cxxopts::value<bool>(doHelp));
    // clang-format on    
    options.parse(argc, argv);

    if((argumentCount <= 1) || options.count("help"))
    {
        std::cout << options.help({""}) << std::endl;
        return 0;
    }
    
    if(sModel.empty())
    {
        std::cerr << "Must specify output *.dat model file."  << std::endl;
        exit(1);
    }

    if(sInput.empty())
    {
        std::cerr << "Must specify valid input file list."  << std::endl;
        exit(1);
    }
    
    const auto filenames = drishti::cli::expand(sInput);

    // Default deserialization:
    //dlib::shape_predictor sp;
    //dlib::deserialize( model.c_str() ) >> sp;

    drishti::ml::shape_predictor sp;
    load_cpb(sModel, sp);
    int ellipse_count = sp.m_ellipse_count;
    
#if 0
    for(auto &f : filenames)
    {
        cv::Mat crop = cv::imread(f, cv::IMREAD_GRAYSCALE);

        auto img = dlib::cv_image<uint8_t>(crop);
        dlib::full_object_detection shape = sp(img, dlib::rectangle(0,0,crop.cols,crop.rows));

        cv::Mat canvas;
        cv::cvtColor(crop, canvas, cv::COLOR_GRAY2BGR);

        int count = shape.num_parts();
        count -= (ellipse_count * 5);
        std::vector<cv::Point> points;
        for(int j = 0; j < count; j++)
        {
            points.push_back(cv_point(shape.part(j)));
            cv::circle(canvas, points.back(), 4, {0,255,0}, -1, 8);
        }

        std::vector<cv::RotatedRect> ellipses;
        for(int j = count; j < shape.num_parts(); j+=5)
        {
            cv::RotatedRect e1;
            e1.center.x =    shape.part(j+0).x();
            e1.center.y =    shape.part(j+1).x();
            e1.size.width =  shape.part(j+2).x();
            e1.size.height = shape.part(j+3).x();
            e1.angle =       shape.part(j+4).x();
            ellipses.push_back(e1);

            std::cout << e1.center << " " << e1.size << " " << e1.angle << std::endl;

            cv::ellipse(canvas, e1, {0,255,0}, 1, 8);
        }


        cv::imshow("canvas", canvas);
        cv::waitKey(0);
    }

#else
    std::vector<std::vector<dlib::full_object_detection> > faces_train;
    dlib::array<dlib::array2d<uint8_t>> images_train;
    dlib::image_dataset_file source(sInput);
    source.skip_empty_images();
    load_image_dataset(images_train, faces_train, source);

    double elapsed = 0.0;
    for(int i = 0; i < images_train.size(); i++)
    {
        for(const auto &face : faces_train[i])
        {
            int64 tic = cv::getTickCount();
            dlib::full_object_detection shape = sp(images_train[i], face.get_rect());
            int64 toc = cv::getTickCount();
            elapsed += (toc - tic) / cv::getTickFrequency();

            cv::Mat image;
            if(doPreview)
            {
                image = dlib::toMat( images_train[i] );
                cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
                cv::rectangle(image, cv_rect(face.get_rect()), {0,255,0}, 1, 8);
            }

            std::vector<cv::Point> points;
            for(int j = 0; j < shape.num_parts(); j++)
            {
                points.push_back(cv_point(shape.part(j)));

                if(doPreview)
                {
                    cv::circle(image, points.back(), 2, {0,255,0}, -1, 8);
                }
            }

            if(doPreview)
            {
                cv::imshow("image", image), cv::waitKey(0);
            }
        }
    }

    std::cout << "elapsed: " << elapsed << std::endl;
#endif

    return 0;
}

int main(int argc, char *argv[])
{
    try
    {
        mine(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cout << "\nexception thrown!" << std::endl;
        std::cout << e.what() << std::endl;
    }
}
