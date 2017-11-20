/*! -*-c++-*-
  @file   fddbcrop.cpp
  @author David Hirvonen
  @brief  FDDB dataset parsing.

  \copyright Copyright 2015-2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/ml/BoundingBox.h"

#include "FDDB.h"
#include "cxxopts.hpp"

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h"
#endif
// clang-format on

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace bfs = boost::filesystem;

int main(int argc, char* argv[])
{
    const auto argumentCount = argc;

    std::string sInput;
    std::string sOutput;
    std::string sTextDir;
    std::string sFaceDir;
    bool doVerbose = false;

    cxxopts::Options options("drishti-fddb", "Command line parser for fddb images");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("t,text", "Text directory", cxxopts::value<std::string>(sTextDir))
        ("f,face", "Face directory", cxxopts::value<std::string>(sFaceDir))
        ("v,verbose", "Verbose output", cxxopts::value<bool>(doVerbose))        
        ("h,help", "Print help message");
    // clang-format on    

    options.parse(argc, argv);

    if((argumentCount <= 1) || options.count("help"))
    {
        std::cout << options.help({""}) << std::endl;
        return 0;
    }    

    // The master input list looks like this:
    //
    // 2002/08/07/big/img_1358
    // 134.525 81.1928 1.50212 165.22 159.212 1
    //
    // 2002/07/22/big/img_306
    // 62.5585 42.6637 1.49185 162.583 93.6411 1
    //
    // 2002/08/13/big/img_619
    // 54.885 35.9912 1.49674 137.485 156.363 1
    // 58.1053 41.7296 1.32333 304.696 157.209 1
    // 54.3857 41.4509 -1.51792 439.195 56.9354 1
        
    // We want to dump each image entry to its own file:
    // 2002/08/07/big/img_1358 => 2002_08_07_big_img_1358

    std::vector<FDDB::record> records = parseFDDB(sInput);
    
    for(auto &r : records)
    {
        bfs::path imageFilename, textFilename;
        std::string flatFilename;
        
        {
            bfs::path file(r.filename);
            textFilename = (bfs::path(sTextDir) / file).replace_extension(".txt");
            imageFilename = (bfs::path(sFaceDir) / file).replace_extension(".jpg");
            
            flatFilename = r.filename;
            std::replace(flatFilename.begin(), flatFilename.end(), '/', '_');
            std::cout << r.filename << " = " << flatFilename << std::endl;
        }
        
        // lbl  - a string label describing object type (eg: 'pedestrian')
        // bb   - [l t w h]: bb indicating predicted object extent
        // occ  - 0/1 value indicating if bb is occluded
        // bbv  - [l t w h]: bb indicating visible region (may be [0 0 0 0])
        // ign  - 0/1 value indicating bb was marked as ignore
        // ang  - [0-360] orientation of bb in degrees
        
        drishti::ml::BoundingBoxSet objectSet;
        
        for(auto &l : r.ellipses)
        {
            const auto &e = l.first;
            cv::RotatedRect ellipse( cv::Point2d(e[3],e[4]), cv::Size2d(e[0]*2.0,e[1]*2.0), e[2]*180.0/M_PI );
            
            float r = ((ellipse.size.width + ellipse.size.height) * 0.25f) * 0.75f;
            cv::Point2f v(std::cos(e[2]), std::sin(e[2])), c = ellipse.center + (v * ((0<v.y)-(v.y<0)) * ellipse.size.height * 0.2 /* 0.06125f */ );
            cv::Rect roi(cv::Point(c) - cv::Point(r,r), cv::Point(c) + cv::Point(r,r));
            
            drishti::ml::BoundingBox object("face", roi, 0.0, roi, false, /* ellipse.angle */ 0.0);
            object.ellipse = ellipse;
            objectSet.objects.push_back(object);
        }
        
        std::cout << imageFilename << std::endl;
        cv::Mat image = cv::imread(imageFilename.string()); // currently need to read in full image for dimensions
        
        if(image.empty())
        {
            continue;
        }
        
        { // All pairs occlusion tests to set the occ fields:
            auto &objects = objectSet.objects;
            cv::Mat1f overlap(objects.size(), objects.size(), 0.f);
            for(int i = 0; i < objects.size(); i++)
			{
                for(int j = (i+1); j < objects.size(); j++)
				{
                    overlap(i,j) = (objects[i].bb & objects[j].bb).size().area();
				}
			}

            overlap = (overlap + overlap.t());
            for(int i = 0; i < overlap.rows; i++)
            {
                int area = objects[i].bb.area();
                int occlusion = cv::sum(overlap.row(i))[0];
                objects[i].ign = int( (objects[i].bb & cv::Rect({0,0},image.size())).area() != objects[i].bb.area() );
                objects[i].occ = double(std::min(occlusion,area)) / area;
            }

#if defined(DRISHTI_USE_IMSHOW)
            if(doVerbose)
            {
                for(const auto &o : objects)
                {
                    int visible = 1 - int((o.occ > 0.1) || o.ign);
                    cv::Scalar color(0,255*int(visible),255*(1-visible));
                    cv::ellipse(image, o.ellipse, color, 2, 8);
                    cv::rectangle(image, o.bb, color, 2, 8);
                    
                    std::stringstream text;
                    text << o.occ << " " << o.bb.width;
                    cv::putText(image, text.str(), o.bb.tl(), CV_FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar::all(255));
                }

                glfw::imshow("image", image);
                glfw::waitKey(0);
                glfw::destroyAllWindows();
            }
#endif
            
            if(image.channels() == 3)
            {
                {// Dump the object set in a file for training:
                    std::cout << textFilename.string() << std::endl;
                    std::ofstream stream( sOutput + "/" + flatFilename + ".txt" );
                    if(stream)
                    {
                        stream << objectSet;
                    }
                }
                
                // Dump face crops:
                for(int i = 0; i < objectSet.objects.size(); i++)
                {
                    if((objects[i].occ < 0.1) && !objects[i].ign)
                    {
                        std::stringstream ext;
                        ext << "_" << i << ".jpg";
            
                        bfs::path cropFilename = bfs::path(sFaceDir) / bfs::path(r.filename + ext.str());
                        std::cout << cropFilename.string() << std::endl;
                        //cv::imwrite( cropFilename.string(), image(objects[i].bb) );
                    }
                }
            }
        }
    }
}
