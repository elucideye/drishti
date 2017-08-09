/*!
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
#include "drishti/testlib/drishti_cli.h"

#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cv_cereal.h"

#include "landmarks/DlibXML.h"

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

class DlibDocumentEye : public DlibDocument
{
public:
    DlibDocumentEye()
    {
        m_spec = drishti::eye::EyeModelSpecification::create();
    }
    void addEye(const cv::Rect &roi, const std::string& sEye, const std::string& sImage)
    {
        drishti::eye::EyeModel eye;
        load(sEye, eye);

        if (eye.eyelids.size())
        {
            const auto points = drishti::eye::eyeToShape(eye, m_spec);
            
            // Parse image dimensions
            addPoints(roi, points, sImage);
        }
    }

protected:

    drishti::eye::EyeModelSpecification m_spec;
};

static int drishtiEyeToDlib(const std::vector<std::string>& filenames, std::string& sOutput, const std::string& ext = ".eye.xml")
{
    std::ofstream os(sOutput);
    if (os)
    {
        DlibDocumentEye doc;
        doc.start();
        for (const auto& sImage : filenames)
        {
            cv::Size size = read_png_size(sImage);
            if(size.area())
            {
                bfs::path sEye(sImage);
                sEye.replace_extension(ext);
                doc.addEye(cv::Rect({0,0}, size), sEye.string(), sImage);
            }
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
