/*!
  @file   Shape.cpp
  @author David Hirvonen
  @brief  Implementation of simple (de)serializable shapes/contours.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "core/Shape.h"
#include "core/Line.h"
#include "core/string_utils.h"

#include <unsupported/Eigen/Splines>

#include <fstream>
#include <string>

DRISHTI_CORE_BEGIN

const char * sXML = R"text(<?xml version='1.0' encoding='ISO-8859-1'?>
<?xml-stylesheet type='text/xsl' href='image_metadata_stylesheet.xsl'?>
<dataset>
<name>Training faces</name>
<comment>Custom face landmark images.</comment>
<images>
)text";

std::string getXMLHeader()
{
    std::string header = sXML;
    return header;
}

std::string getXMLFooter()
{
    const char * sXML = "    </images>\n</dataset>\n";
    return std::string(sXML);
}

void Shape::write(cv::FileStorage& fs) const
{
    fs << "{";
    fs << "roi" << roi;
    fs << "points";
    fs << "[";
    for(const auto &p : contour)
    {
        cv::Point3f tmp(p.p.x, p.p.y, p.knot);
        fs << tmp;
    }
    fs << "]";
    fs << "}";
}

void Shape::read(const cv::FileNode& node)
{
    std::vector<int> tmp;
    node["roi"] >> tmp; roi = {tmp[0], tmp[1], tmp[2], tmp[3]};
    
    cv::FileNode n = node["points"];
    if(n.type() == cv::FileNode::SEQ)
    {
        ControlPoint p;
        for(int i = 0; i < n.size(); i++)
        {
            cv::Point3f tmp;
            n[i] >> tmp;
            contour.push_back( ControlPoint(cv::Point2f(tmp.x, tmp.y), tmp.z ) );
        }
    }
};

void Shape::read(const std::string &filename)
{
    //std::cout << filename << std::endl;
    cv::FileStorage storage(filename, cv::FileStorage::READ);
    if(storage.isOpened())
    {
        auto node = storage["contour"];
        if(!node.empty())
        {
            read(node);
        }
        else
        {
            read(storage.root());
        }
    }
};

void Shape::write(const std::string &filename) const
{
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);
    if(storage.isOpened())
    {
        write(storage["contour"]);
    }
};

void fitSpline(const PointVec &controlPoints, PointVec &interpolatedPoints, int count, bool closed)
{
    if(controlPoints.size() > 1)
    {
        typedef Eigen::Spline<double,2> Spline2d;
        typedef Spline2d::PointType PointType;
        typedef Spline2d::ControlPointVectorType ControlPointVectorType;
        
        ControlPointVectorType points(2,controlPoints.size() + int(closed));
        for(int i = 0; i < controlPoints.size() + int(closed); i++)
        {
            points(0, i) = controlPoints[i % controlPoints.size()].x;
            points(1, i) = controlPoints[i % controlPoints.size()].y;
        }
        
        const Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points,3);
        interpolatedPoints.clear();
        count += int(closed);
        for (int i = 0; i < count; i++)
        {
            float u = float(i) / count;
            PointType p = spline( u );
            interpolatedPoints.emplace_back( p(0,0), p(1,0) );
        }
    }
}

static void upsample(const Eigen::Spline<double,2> &spline, int extent, float k0, float k1, PointVec &interpolatedPoints)
{
    const float arcSpan = k1 - k0;
    
    // Denseley sample the spline to create an arc:
    std::vector<cv::Point2f> arc(1000);
    {
        const float tic = arcSpan / float(arc.size());
        for(int j = 0; j < arc.size(); j++)
        {
            const float u = k0 + float(j) * tic;
            const auto p = spline(u);
            arc[j] = cv::Point2f(p(0,0), p(1,0));
        }
        
        float arcLength = 0.f;
        std::vector<float> arcLengths(arc.size()-1);
        std::vector<float> arcLengthsAccum(arcLengths.size());
        for(int j = 0; j < arcLengths.size(); j++)
        {
            const float segmentLength = cv::norm(arc[j+1] - arc[j+0]);
            arcLength += segmentLength;
            arcLengths[j] = segmentLength;
            arcLengthsAccum[j] = arcLength;
        }
        
        interpolatedPoints.push_back(arc[0]);
        
        float spacing = arcLength / float(extent); // desired spacing
        for(int j = 0; j < extent; j++)
        {
            float arcLengthTarget = float(j) * spacing;
            for(int k = 1; k < arcLengthsAccum.size(); k++)
            {
                float v0 = arcLengthsAccum[k-1];
                float v1 = arcLengthsAccum[k+0];
                if(v0 <= arcLengthTarget && arcLengthTarget < v1)
                {
                    interpolatedPoints.push_back(arc[k]);
                    break;
                }
            }
        }
    }
}


void upsample(const PointVec &controlPoints, PointVec &interpolatedPoints, int factor, bool closed)
{
    if(controlPoints.size() > 1)
    {
        typedef Eigen::Spline<double,2> Spline2d;
        typedef Spline2d::PointType PointType;
        typedef Spline2d::ControlPointVectorType ControlPointVectorType;
        
        int length = int(controlPoints.size()) + int(closed);
        ControlPointVectorType points(2, length);
        for(int i = 0; i < length; i++)
        {
            points(0, i) = controlPoints[i % controlPoints.size()].x;
            points(1, i) = controlPoints[i % controlPoints.size()].y;
        }

        Spline2d::KnotVectorType knots;
        Eigen::ChordLengths(points, knots);
        const Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points,3,knots);
        
        interpolatedPoints.clear();

        if(closed)
        {
            int half = int(controlPoints.size())/2;
            int extent = half * factor; // length of each upper/lower half
            for(int i = 0; i < 2; i++)
            {
                const float k0 = knots((i+0)*half);
                const float k1 = knots((i+1)*half);
                upsample(spline, extent, k0, k1, interpolatedPoints);
            }
        }
        else
        {
            int extent = int(controlPoints.size() * factor) - 1;
            upsample(spline, extent, 0, 1, interpolatedPoints);
        }
    }
}

std::vector< std::vector<int> > readSpecification(const std::string &filename)
{
    std::vector<std::vector<int>> specification;
    
    std::vector<std::string> lines;
    std::ifstream file(filename);
    std::copy(std::istream_iterator<drishti::core::Line>(file), std::istream_iterator<drishti::core::Line>(), std::back_inserter(lines));
    
    specification.resize(lines.size());
    for(int i = 0; i < lines.size(); i++)
    {
        std::vector<std::string> tokens;
        drishti::core::tokenize(lines[i], tokens);
        for(int j = 0; j < tokens.size(); j++)
        {
#if 1
            int value;
            std::stringstream ss(tokens[j]);
            ss >> value;
            specification[i].push_back(value);
#else
            specification[i].push_back( std::stoi(tokens[j]) );
#endif
        }
    }
    
    // Specification is:
    for(int i = 0; i < specification.size(); i++)
    {
        for(int j = 0; j < specification[i].size(); j++)
        {
            std::cout << specification[i][j] << " " ;
        }
        
        std::cout << std::endl;
    }
    
    return specification;
}


// These OpenCV functions must be in global namespace
void write(cv::FileStorage& fs, const std::string&, const drishti::core::Shape& x) { x.write(fs); }
void read(const cv::FileNode& node, drishti::core::Shape& x, const drishti::core::Shape & default_value)
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

std::vector<cv::Vec3b> makeRainbow()
{
    static std::vector< cv::Vec3b > rainbow
    {
        {0,0,255},     // red
        {0,127,255},   // orange
        {0,255,255},   // yellow
        {0,255,0},     // green
        {255,0,0},     // blue
        {130, 0, 75},  // indigo
        {255, 0, 139}, // violet
        {127,127,127}, // white
    };
    return rainbow;
}

DRISHTI_CORE_END

