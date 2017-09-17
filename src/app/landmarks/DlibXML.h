/*! -*-c++-*-
 @file   DlibXML.h
 @author David Hirvonen
 @brief  Convert points to landmark format used by dlib shape_predictor.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#ifndef __drishti_landmarks_DlibXML_h__
#define __drishti_landmarks_DlibXML_h__

#include "drishti/core/Logger.h"
#include "drishti/core/drishti_string_hash.h"
#include "drishti/testlib/drishti_cli.h"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cv_cereal.h"

#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"

#include <fstream>
#include <iostream>
#include <string>

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

inline cv::Size read_png_size(const std::string& filename)
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

/*
 std::ofstream os(sOutput);
 if (os)
 {
     DlibDocument doc;

     doc.start();
     for (const auto& sImage : filenames)
     {
         auto points = load(sImage);
         doc.addPoints(points, sImage);
     }
     doc.finish();
     doc.write(os);
 }
*/

class DlibDocument
{
public:
    DlibDocument() = default;
    ~DlibDocument() = default;

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

    void addPoints(const cv::Rect& roi, const std::vector<cv::Point2f>& points, const std::string& filename)
    {
        m_count++;

        // TODO: deserialize spec from command line

        // ###########
        // ### XML ###
        // ###########

        // <image file='/Some/long/path/file0.png'>
        auto* image = doc.allocate_node(rapidxml::node_element, "image");
        image->append_attribute(doc.allocate_attribute("file", filename.c_str()));

        {
            // <box top='0' left='0' width='200' height='150'>
            const auto sTop = doc.allocate_string(std::to_string(roi.y).c_str());
            const auto sLeft = doc.allocate_string(std::to_string(roi.x).c_str());
            const auto sWidth = doc.allocate_string(std::to_string(roi.width).c_str());
            const auto sHeight = doc.allocate_string(std::to_string(roi.height).c_str());

            auto* box = doc.allocate_node(rapidxml::node_element, "box");
            box->append_attribute(doc.allocate_attribute("top", sTop));
            box->append_attribute(doc.allocate_attribute("left", sLeft));
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

    void write(std::ofstream& os)
    {
        if (m_count > 0)
        {
            os << doc;
        }
    }

    int m_count = 0;
    rapidxml::xml_node<>* dataset = nullptr;
    rapidxml::xml_node<>* images = nullptr;
    rapidxml::xml_document<> doc;
};

#endif // __drishti_landmarks_DlibXML_h__
