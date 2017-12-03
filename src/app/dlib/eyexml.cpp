/*! -*-c++-*-
 @file   eyexml.cpp
 @author David Hirvonen
 @brief  Convert drishti eye model xml files to dlib shape_predictor "parts" file.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "drishti/eye/Eye.h"
#include "drishti/eye/EyeIO.h"
#include "drishti/core/Logger.h"
#include "drishti/core/drishti_string_hash.h"
#include "drishti/core/string_utils.h"
#include "drishti/testlib/drishti_cli.h"

#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/geometry/motion.h"

#include <cereal/archives/json.hpp>

#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"

#include <opencv2/highgui.hpp>
#include <boost/filesystem.hpp>

#include "cxxopts.hpp"

#include <fstream>
#include <iostream>
#include <string>

namespace bfs = boost::filesystem;

using string_hash::operator"" _hash;

/*
 
 <?xml version='1.0' encoding='ISO-8859-1'?>
 <?xml-stylesheet type='text/xsl' href='image_metadata_stylesheet.xsl'?>
 <dataset>
    <name>Training faces</name>
    <comment>Custom face landmark images.</comment>
    <images>
        <image file='/Some/long/path/file0.png'>
            <box top='0' left='0' width='200' height='150'>
                <part name='0000' x='49' y='82'/>
                <part name='0001' x='60' y='72'/>
                <part name='0002' x='74' y='66'/>
 ...
                <part name='0036' x='16' y='0'/>
            </box>
        </image>
    </images> 
</dataset>

 */

// http://stackoverflow.com/a/22926477
template <typename T>
inline T ntoh_any(T t)
{
    static const unsigned char int_bytes[sizeof(int)] = { 0xFF };
    static const int msb_0xFF = 0xFF << (sizeof(int) - 1) * CHAR_BIT;
    static bool host_is_big_endian = (*(reinterpret_cast<const int*>(int_bytes)) & msb_0xFF) != 0;
    if (host_is_big_endian)
    {
        return t;
    }

    unsigned char* ptr = reinterpret_cast<unsigned char*>(&t);
    std::reverse(ptr, ptr + sizeof(t));
    return t;
}

static cv::Size read_png_size(const std::string& filename)
{
    cv::Size size; // 0
    std::ifstream in(filename);
    if (in)
    {
        unsigned int width, height;
        in.seekg(16);
        in.read((char*)&width, 4);
        in.read((char*)&height, 4);
        size = { static_cast<int>(ntoh_any(width)), static_cast<int>(ntoh_any(height)) };
    }
    return size;
}

// TODO: share w/ train_cpr
static void loadJSON(const std::string& filename, drishti::eye::EyeModel& eye)
{
    std::ifstream is(filename);
    if (is)
    {
        cereal::JSONInputArchive ia(is);
        typedef decltype(ia) Archive;
        ia(GENERIC_NVP("eye", eye));
    }
}

// TODO: share w/ train_cpr
static void load(const std::string& sEye, drishti::eye::EyeModel& eye)
{
    std::string sExt;
    if (sEye.find(".eye.xml") != std::string::npos)
    {
        sExt = ".eye.xml";
    }
    else
    {
        sExt = bfs::path(sEye).extension().string();
    }

    switch (string_hash::hash(sExt))
    {
        case ".eye.xml"_hash:
            eye.read(sEye);
            break;

        case ".json"_hash:
            loadJSON(sEye, eye);
            break;
    }
}

class DlibDocument
{
public:
    DlibDocument()
    {
        m_spec = drishti::eye::EyeModelSpecification::create();
    }

    ~DlibDocument()
    {
    }

    void start()
    {
        static const char* sStyleSheet = "xml-stylesheet type='text/xsl' href='image_metadata_stylesheet.xsl'";
        static const char* sComment = "eyelids, crease, iris, pupil";
        static const char* sName = "Drishti eye model";

        auto* decl = doc.allocate_node(rapidxml::node_declaration);
        decl->append_attribute(doc.allocate_attribute("version", "1.0"));
        decl->append_attribute(doc.allocate_attribute("encoding", "ISO-8859-1"));
        doc.append_node(decl);

        auto* style = doc.allocate_node(rapidxml::node_pi, sStyleSheet);
        doc.append_node(style);

        // <dataset>
        dataset = doc.allocate_node(rapidxml::node_element, "dataset");

        {
            //<name>Training faces</name>
            auto* name = doc.allocate_node(rapidxml::node_element, "name", sName);
            dataset->append_node(name);

            //<comment>Custom face landmark images.</comment>
            auto* comment = doc.allocate_node(rapidxml::node_element, "comment", sComment);
            dataset->append_node(comment);

            {
                // <images>
                images = doc.allocate_node(rapidxml::node_element, "images");
            }
        }
    }

    void finish()
    {
        dataset->append_node(images);
        doc.append_node(dataset);
    }

    void addEye(const std::string& sEye, const std::string& sImage)
    {
        drishti::eye::EyeModel eye;
        load(sEye, eye);

        if (eye.eyelids.size())
        {
            // Parse image dimensions
            addEye(eye, sImage);
        }
    }
    
    cv::Mat draw_with_padding(const cv::Mat &image, const drishti::eye::EyeModel &eyeIn)
    {
        static const int border = 200;
        
        cv::Mat canvas, bigger;
        cv::resize(image, bigger, image.size() * 4);
        cv::copyMakeBorder(bigger, canvas, border, border, border, border, cv::BORDER_CONSTANT);
        
        const cv::Matx33f H = transformation::translate(border, border) * transformation::scale(4.f, 4.f);
        const auto eye = H * eyeIn;
        const auto points = drishti::eye::eyeToShape(eye, m_spec);
        
        for(int i = 0; i < 16; i++)
        {
            cv::line(canvas, points[(i+0)%16], points[(i+1)%16], {255,255,255}, 1, 8);
        }
        
        for(int i = 16; i < 24; i++)
        {
            cv::line(canvas, points[i+0], points[i+1], {255,255,255}, 1, 8);
        }
        
        cv::ellipse(canvas, eye.irisEllipse, {255,255,255}, 1, 8);
        cv::ellipse(canvas, eye.pupilEllipse, {255,255,255}, 1, 8);
        
        const float scale1 = 1.35f;
        const float scale2 = scale1 * 1.1f;
        cv::Matx33f S1 = transformation::scale(scale1, scale1, eye.irisEllipse.center);
        cv::Matx33f S2 = transformation::scale(scale2, scale2, eye.irisEllipse.center);
        
        for(int i = 0; i < points.size() - 10; i++)
        {
            const cv::Point2f p2 = points[i];
            const cv::Point3f p3(p2.x, p2.y, 1.f), q3 = S1 * p3, r3 = S2 * p3;
            const cv::Point q2(q3.x, q3.y), r2(r3.x, r3.y);
            cv::circle(canvas, p2, 6, {0,255,0}, -1, 8);
            cv::line(canvas, p2, q2, {0,255,0}, 1, 8);
            
            std::stringstream ss;
            ss << i;
            
            const int fontFace = CV_FONT_HERSHEY_PLAIN;
            const double fontScale = 2.0;
            int thickness = 3, baseline = 0;
            const cv::Size textSize = cv::getTextSize(ss.str(), fontFace, fontScale, thickness, &baseline);
            baseline += thickness;
            
            cv::Point2f bl = (q2 + cv::Point(0, thickness)), tr = (q2 + cv::Point(textSize.width, -textSize.height)), diag = (tr - bl) * 0.5f;
            cv::putText(canvas, ss.str(), r2 - cv::Point(diag.x, diag.y), fontFace, fontScale, {127,255,127}, thickness);
        }

        return canvas;

    }

    void addEye(drishti::eye::EyeModel& eye, const std::string& filename)
    {
        cv::Size size = read_png_size(filename);

        if (size.area() > 0)
        {
            m_eyeCount++; // only write if # eyes > 0

            // TODO: deserialize spec from command line
            auto points = drishti::eye::eyeToShape(eye, m_spec);

            {
                // Example visualization via annotation scheme drawing
                //std::string base = drishti::core::basename(filename);
                //cv::Mat canvas = draw_with_padding(cv::imread(filename), eye);
                //cv::imwrite("/tmp/eyes/" + base + ".jpg", canvas);
            }
            
            // ###########
            // ### XML ###
            // ###########

            // <image file='/Some/long/path/file0.png'>
            auto* image = doc.allocate_node(rapidxml::node_element, "image");
            image->append_attribute(doc.allocate_attribute("file", filename.c_str()));

            {
                // <box top='0' left='0' width='200' height='150'>
                const auto sWidth = doc.allocate_string(std::to_string(size.width).c_str());
                const auto sHeight = doc.allocate_string(std::to_string(size.height).c_str());

                auto* box = doc.allocate_node(rapidxml::node_element, "box");
                box->append_attribute(doc.allocate_attribute("top", "0"));
                box->append_attribute(doc.allocate_attribute("left", "0"));
                box->append_attribute(doc.allocate_attribute("width", sWidth));
                box->append_attribute(doc.allocate_attribute("height", sHeight));

                for (int j = 0; j < points.size(); j++)
                {
                    // <part name='0000' x='49' y='82'/>
                    const int x = static_cast<int>(points[j].x + 0.5f);
                    const int y = static_cast<int>(points[j].y + 0.5f);

                    // Important: dlib requires fixed width part names
                    std::stringstream ss;
                    ss << std::setfill('0') << std::setw(4) << j;

                    const auto sName = doc.allocate_string(ss.str().c_str());
                    const auto sX = doc.allocate_string(std::to_string(x).c_str());
                    const auto sY = doc.allocate_string(std::to_string(y).c_str());

                    auto* part = doc.allocate_node(rapidxml::node_element, "part");
                    part->append_attribute(doc.allocate_attribute("name", sName));
                    part->append_attribute(doc.allocate_attribute("x", sX));
                    part->append_attribute(doc.allocate_attribute("y", sY));
                    box->append_node(part);
                }
                image->append_node(box);
            }
            images->append_node(image);
        }
    }

    void write(std::ofstream& os)
    {
        if (m_eyeCount > 0)
        {
            os << doc;
        }
    }

    int m_eyeCount = 0;
    drishti::eye::EyeModelSpecification m_spec;
    rapidxml::xml_node<>* dataset = nullptr;
    rapidxml::xml_node<>* images = nullptr;
    rapidxml::xml_document<> doc;
};

static int drishtiEyeToDlib(const std::vector<std::string>& filenames, std::string& sOutput, const std::string& ext = ".eye.xml")
{
    std::ofstream os(sOutput);
    if (os)
    {
        DlibDocument doc;

        doc.start();
        for (const auto& sImage : filenames)
        {
            bfs::path sEye(sImage);
            sEye.replace_extension(ext);
            doc.addEye(sEye.string(), sImage);
        }
        doc.finish();
        doc.write(os);
    }

    return 0;
}

int gauze_main(int argc, char** argv)
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("eyexml");

    // ############################
    // ### Command line parsing ###
    // ############################

    std::string sInput, sOutput, sExtension = ".eye.xml";

    cxxopts::Options options("eyexml", "Convert eye files to xml");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("e,extension", "Extension", cxxopts::value<std::string>(sExtension))
        ("h,help", "Print help message");
    // clang-format on

    options.parse(argc, argv);

    if ((argumentCount <= 1) || options.count("help"))
    {
        logger->info("{}", options.help({ "" }));
        return 0;
    }

    auto filenames = drishti::cli::expand(sInput);

    return drishtiEyeToDlib(filenames, sOutput, sExtension);
}

int main(int argc, char** argv)
{
    try
    {
        return gauze_main(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}
