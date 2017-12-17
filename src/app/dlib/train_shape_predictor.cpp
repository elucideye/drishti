// This example program shows how to use dlib's implementation of the paper:
//    One Millisecond Face Alignment with an Ensemble of Regression Trees by
//    Vahid Kazemi and Josephine Sullivan, CVPR 2014
//
// http://www.csc.kth.se/~vahidk/papers/KazemiCVPR14.pdf
//
// Changelog:
//  - support regression in shape space for simple size reduction
//  - support line indexed features (more pose invariant in some cases (per RCPR))
//  - support normalized pixel differences (can improve regression in some cases)
//  - support trailing ellipse parameters (embedded in point list)
//    for simultaneous contour and ellipse model regression

#include "drishti/core/drishti_stdlib_string.h" // FIRST !!!

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

#include "drishti/core/string_utils.h"
#include "drishti/core/Line.h"
#include "drishti/ml/shape_predictor_archive.h"
#include "drishti/ml/shape_predictor_trainer.h"
#include "drishti/geometry/Ellipse.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cv_cereal.h"

#include "RecipeIO.h"

#include "cxxopts.hpp"

#include <iostream>

//#define _SP dlib
#define _SP drishti::ml

// _cascade_depth = 10;
// _tree_depth = 4;
// _num_trees_per_cascade_level = 500;
// _nu = 0.1;
// _oversampling_amount = 20;
// _feature_pool_size = 400;
// _lambda = 0.1;
// _num_test_splits = 20;
// _feature_pool_region_padding = 0;

static std::vector<std::vector<double>>
get_interocular_distances(const std::vector<std::vector<dlib::full_object_detection>>& objects);

using DlibImageArray = dlib::array<dlib::array2d<uint8_t>>;
using DlibObjectSet = std::vector<std::vector<dlib::full_object_detection>>;

static void reduce_images(DlibImageArray& images_train, DlibObjectSet& faces_train, int ellipse_count, int width);
static void dump_thumbs(DlibImageArray& images_train, DlibObjectSet& faces_train, const std::string& dir, int ellipse_count);

static std::vector<int> parse_dimensions(const std::string& str);
static dlib::rectangle parse_roi(const std::string& str);

static void view_images(DlibImageArray& images_train, DlibObjectSet& faces_train)
{
    for (int i = 0; i < images_train.size(); i++)
    {
        auto& src = images_train[i];
        cv::Mat image(src.nr(), src.nc(), CV_8UC1, (void*)&src[0][0], src.width_step());
        cv::Mat canvas = image.clone();

        for (int j = 0; j < faces_train[i].size(); j++)
        {
            // reduce roi:
            for (int k = 0; k < faces_train[i][j].num_parts(); k++)
            {
                auto& part = faces_train[i][j].part(k);
                cv::Point2f p(part.x(), part.y());
                cv::circle(canvas, p, 4, { 255, 255, 255 }, -1, 8);
                cv::imwrite("/tmp/foo.png", canvas);
            }
        }
    }
}

int gauze_main(int argc, char* argv[])
{
    auto logger = drishti::core::Logger::create("train_shape_predictor");

    const auto argumentCount = argc;

    bool do_help = false;
    bool do_threads = false;
    bool do_thumbs = false;
    bool do_verbose = false;
    bool do_silent = false;

    drishti::dlib::Recipe recipe;

    std::string sModel;
    std::string sRoi;
    std::string sTest;
    std::string sTrain;
    std::string sRecipe;
    std::string sRecipeOut;
    std::string sOutput;
    
    cxxopts::Options options("train_shape_predictor", "Command line interface for dlib shape_predictor training");

    // clang-format off
    options.add_options()
        ( "output", "Output directory for intermediate results", cxxopts::value<std::string>(sOutput))
        ( "train", "training file", cxxopts::value<std::string>(sTrain))
        ( "test", "testing file", cxxopts::value<std::string>(sTest))
        ( "model", "Model file", cxxopts::value<std::string>(sModel))
        ( "thumbs", "dump thumbnails of width n", cxxopts::value<bool>(do_thumbs))        
        ( "roi", "exclusion roi (x,y,w,h) normalized cs", cxxopts::value<std::string>(sRoi))

        // Regression parameters:
        ( "recipe", "Cascaded pose regression training recipe", cxxopts::value<std::string>(sRecipe))
        ( "boilerplate", "Output boilerplate recipe file", cxxopts::value<std::string>(sRecipeOut))        
        
        ( "threads", "Use worker threads when possible", cxxopts::value<bool>(do_threads))
        ( "verbose", "Print verbose diagnostics", cxxopts::value<bool>(do_verbose))
        ( "silent", "Disable logging entirely", cxxopts::value<bool>(do_silent))
        ( "help", "Print the help message", cxxopts::value<bool>(do_help));
    // clang-format on    

    options.parse(argc, argv);

    if(do_silent)
    {
        logger->set_level(spdlog::level::off);
    }    

    if((argumentCount <= 1) || options.count("help"))
    {
        logger->info(options.help({""}));
        return 0;
    }

    if(!sRecipeOut.empty())
    {
        recipe.dimensions = { 4, 8, 12, 16, 20, 24 };
        saveJSON(sRecipeOut, recipe);
        return 0;
    }
    
    if(sRecipe.empty())
    {
        logger->error("Must specify valid training recipe.");
        return 1;
    }
    
    loadJSON(sRecipe, recipe);
    
    if(sTrain.empty())
    {
        logger->error("Must specify valid XML training file.");
        return 1;
    }

    if(sModel.empty())
    {
        logger->error("Must specify output *.dat model file.");
        return 1;
    }

    if(do_verbose)
    {
        logger->info("cascade_depth: {}", recipe.cascades);
        logger->info("tree_depth: {}", recipe.depth);
        logger->info("num_trees_per_cascade_level: {}", recipe.trees_per_level);
        logger->info("nu: {}", recipe.nu);
        logger->info("oversampling_amount: {}", recipe.oversampling);
        logger->info("feature_pool_size: {}", recipe.features);
        logger->info("lambda: {}", recipe.lambda);
        logger->info("num_test_splits: {}", recipe.splits);
        logger->info("feature_pool_region_padding: {}", recipe.padding);
        logger->info("use npd: {}", recipe.npd);
        logger->info("affine: {}", recipe.do_affine);
        logger->info("interpolated: {}", recipe.do_interpolate);
    }

    dlib::array<dlib::array2d<uint8_t>> images_train, images_test;
    std::vector<std::vector<dlib::full_object_detection> > faces_train, faces_test;

    dlib::image_dataset_file source(sTrain);
    source.skip_empty_images();
    load_image_dataset(images_train, faces_train, source);
    
    if(faces_train.empty())
    {
        logger->error("No shapes specified for training");
        return 1;
    }

    if(do_thumbs)
    {
        dump_thumbs(images_train, faces_train, sOutput, recipe.ellipse_count);
        return 0;
    }

    // Here we optionally downsample:
    if(recipe.width > 0)
    {
        reduce_images(images_train, faces_train, recipe.ellipse_count, recipe.width);
    }

    dlib::drectangle roi;
    if(!sRoi.empty())
    {
        roi = parse_roi(sRoi);
    }

    // Now make the object responsible for training the model.
    _SP::shape_predictor_trainer trainer;
    trainer.set_cascade_depth(recipe.cascades);
    trainer.set_tree_depth(recipe.depth);
    trainer.set_num_trees_per_cascade_level(recipe.trees_per_level);
    trainer.set_nu(recipe.nu);                            // regularization (smaller nu == more regularization)
    trainer.set_oversampling_amount(recipe.oversampling); // amount of oversampling for training data
    trainer.set_feature_pool_size(recipe.features);
    trainer.set_lambda(recipe.lambda);                    // feature separation (not learning rate)
    trainer.set_num_test_splits(recipe.splits);
    trainer.set_feature_pool_region_padding(recipe.padding);

    // new parameters
    if(recipe.dimensions.size())
    {
        trainer.set_dimensions(recipe.dimensions);
        trainer.set_cascade_depth(recipe.dimensions.size());
    }
    trainer.set_ellipse_count(recipe.ellipse_count);
    trainer.set_do_npd(recipe.npd);
    trainer.set_do_affine(recipe.do_affine);
    trainer.set_roi(roi);
    trainer.set_do_line_indexed(recipe.do_interpolate);
    
    trainer.set_num_threads(8);
    
    if(do_verbose)
    {
        trainer.be_verbose();
    }

    int max_dim = faces_train[0][0].num_parts() * 2;
    for(const auto &dim : recipe.dimensions)
    {
        CV_Assert(0 < dim && dim <= max_dim);
    }

    if(do_verbose)
    {
        logger->info("Begin training...");
    }

    std::map<int, float> weights;
    for(const auto &w : recipe.weights)
    {
        weights[std::stoi(w.first)] = w.second;
    }
    
    //_SP::shape_predictor sp;
    _SP::shape_predictor sp = trainer.train(images_train, faces_train, weights);
    if(do_verbose)
    {
        logger->info("Done training...");
    }

    // Finally, we save the model to disk so we can use it later.
    //dlib::serialize( sModel.c_str() ) << sp;

    if(do_verbose)
    {
        logger->info("Saving to ...{}", sModel);
    }
    
    save_cpb(sModel, sp);
    sp.populate_f16(); // populate half precision leaf nodes

    if(do_verbose)
    {
        logger->info("Done saving ...{}", sModel);
    }

    auto train_iod = get_interocular_distances(faces_train);
    double training_error = test_shape_predictor(sp, images_train, faces_train, train_iod);
    
    if(do_verbose)
    {
        logger->info("Mean training error: {}", training_error);
    }

    if(!sTest.empty())
    {
        load_image_dataset(images_test, faces_test, sTest);
        
        auto test_iod = get_interocular_distances(faces_test);
        float test_error = test_shape_predictor(sp, images_test, faces_test, test_iod);
        
        if(do_verbose)
        {
            logger->info("Mean testing error: {}", test_error);
        }
    }

    return 0;
}

int main(int argc, char *argv[])
{
    try
    {
        gauze_main(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception thrown:" << e.what();
    }
}

// ----------------------------------------------------------------------------------------

static double interocular_distance (const dlib::full_object_detection& det)
{
    double length = 0;
    for(int i = 0; i < det.num_parts(); i++)
        for(int j = i+1; j < det.num_parts(); j++)
        {
            length = std::max(double(dlib::length(det.part(i) - det.part(j))), length);
        }

    return length;
}

static std::vector<std::vector<double> > get_interocular_distances (const DlibObjectSet& objects)
{
    std::vector<std::vector<double> > temp(objects.size());
    for (unsigned long i = 0; i < objects.size(); ++i)
    {
        for (unsigned long j = 0; j < objects[i].size(); ++j)
        {
            temp[i].push_back(interocular_distance(objects[i][j]));
        }
    }
    return temp;
}

static void dump_thumbs(DlibImageArray &images_train, DlibObjectSet &faces_train, const std::string &dir, int ellipse_count)
{
    static std::vector< cv::Vec3b > rainbow // RGB (inuitive)
    {
        {255,0,0},     // red
        {255,127,0},   // orange
        {255,255,0},   // yellow
        {0,255,0},     // green
        {0,0,255},     // blue
        {75, 0, 130},  // indigo
        {139, 0, 255}, // violet
        {127,127,127}, // white
    };
    for(auto &c : rainbow)
    {
        std::swap(c[0], c[2]); // RGB -> BGR
    }

    for(int i = 0; i < images_train.size(); i++)
    {
        auto &src = images_train[i];
        cv::Mat image(src.nr(), src.nc(), CV_8UC1, (void *)&src[0][0], src.width_step()), small;

        for(int j = 0; j < faces_train[i].size(); j++)
        {
            auto &roiIn = faces_train[i][j].get_rect();
            cv::Rect roi(roiIn.left(), roiIn.top(), roiIn.width(), roiIn.height());

            cv::Mat canvas;
            try
            {
                if(image.channels() != 3)
                {
                    cv::cvtColor(image(roi), canvas, cv::COLOR_GRAY2BGR);
                }
                else
                {
                    canvas = image(roi).clone();
                }
            }
            catch(...)
            {
                std::cout << "BAD ROI: " <<  i << " " << j << std::endl;
                continue;
            }

            int end =faces_train[i][j].num_parts() - (ellipse_count * 5);
            for(int k = 0; k < end; k++)
            {
                auto &part = faces_train[i][j].part(k);
                cv::Point p(part.x(), part.y());
                cv::circle(canvas, p - roi.tl(), 3, rainbow[k%rainbow.size()], -1, 8);
            }

            std::stringstream ss;
            ss << dir << "/" << "roi_" <<  i << "_" << j << ".png";
            cv::imwrite(ss.str(), canvas);
        }
    }
}

static void reduce_int(long &value, float scale)
{
    value = static_cast<long>(value * scale + 0.5f);
}

static void reduce_int(dlib::rectangle &roi, float scale)
{
    reduce_int(roi.top(), scale);
    reduce_int(roi.left(), scale);
    reduce_int(roi.right(), scale);
    reduce_int(roi.bottom(), scale);
}

static void reduce_images(DlibImageArray &images_train, DlibObjectSet &faces_train, int ellipse_count, int width)
{
    for(int i = 0; i < images_train.size(); i++)
    {
        auto &src = images_train[i];
        cv::Mat image(src.nr(), src.nc(), CV_8UC1, (void *)&src[0][0], src.width_step()), small;

        const float scale = float(width) / image.cols;
        cv::resize(image, small, {}, scale, scale, cv::INTER_LANCZOS4);
        dlib::assign_image(src, dlib::cv_image<uint8_t>(small));

        for(int j = 0; j < faces_train[i].size(); j++)
        {
            // reduce roi:
            reduce_int(faces_train[i][j].get_rect(), scale);

            const int end = faces_train[i][j].num_parts() - (ellipse_count * 5);
            for(int k = 0; k < end; k++)
            {
                auto &part = faces_train[i][j].part(k);
                reduce_int(part.x(), scale);
                reduce_int(part.y(), scale);
                CV_Assert(part.x() < 1000 && part.y() < 1000);
            }

            // Handle the ellipse data (optional):
            for(int k = end; k < faces_train[i][j].num_parts(); k+= 5)
            {
                auto & obj = faces_train[i][j];
                obj.part(k+0).x() *= scale;
                obj.part(k+1).x() *= scale;
                obj.part(k+2).x() *= scale;
                obj.part(k+3).x() *= scale;
            }
        }
    }
}

static dlib::rectangle parse_roi(const std::string &str)
{
    std::vector<std::string> tokens;
    drishti::core::tokenize(str, tokens);
    float l = std::stof(tokens[0]);
    float t = std::stof(tokens[1]);
    float r = std::stof(tokens[2]);
    float b = std::stof(tokens[3]);
    return dlib::drectangle(l, t, r-l, b-t);
}
