/*!
  @file   ACFObject.h
  @author David Hirvonen (C++ implementation)
  @brief  Delcaration of an ACF object class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __DRISHTI__ACFOBJECT__
#define __DRISHTI__ACFOBJECT__

#include "drishti/acf/drishti_acf.h"

#include <string>
#include <iostream>

DRISHTI_ACF_NAMESPACE_BEGIN

// ((((( FACE ROI )))))

// lbl  - a string label describing object type (eg: 'pedestrian')
// bb   - [l t w h]: bb indicating predicted object extent
// occ  - 0/1 value indicating if bb is occluded
// bbv  - [l t w h]: bb indicating visible region (may be [0 0 0 0])
// ign  - 0/1 value indicating bb was marked as ignore
// ang  - [0-360] orientation of bb in degrees

// lbl: ''
// bb: [0 0 0 0]
// occ: 0
// bbv: [0 0 0 0]
// ign: 0
// ang: 0

struct ACFObject
{
    ACFObject(const  std::string &lbl, const cv::Rect &bb, float occ, const cv::Rect &bbV, bool ign, double ang)
        : lbl(lbl)
        , bb(bb)
        , occ(occ)
        , bbV(bbV)
        , ign(ign)
        , ang(ang) {}

    std::string lbl;
    cv::Rect bb;
    double occ = 0.0;
    cv::Rect bbV;
    bool ign = false;
    double ang = 0.0;

    cv::RotatedRect ellipse; // NOTE: preserve FDDG ellipse -- not in the output

    friend std::ostream & operator<<(std::ostream &os, const ACFObject &src);
};

struct ACFObjectSet
{
    ACFObjectSet() {}
    friend std::ostream & operator<<(std::ostream &os, const ACFObjectSet &src);

    std::vector<ACFObject> objects;
};

// ((((((((( IO )))))))))

inline std::ostream & operator<<(std::ostream &os, const ACFObject &src)
{
    static const char space = ' ';
    os << src.lbl << space;
    os << src.bb.x << ' ' << src.bb.y << ' ' << src.bb.width << ' ' << src.bb.height << space;
    os << src.occ << space; // occluded
    os << src.bbV.x << ' ' << src.bbV.y << ' ' << src.bbV.width << ' ' << src.bbV.height << space; // visible bounding box
    os << src.ign << space; // ignore
    os << src.ang << space; // angle
    return os;
}

inline std::ostream & operator<<(std::ostream &os, const ACFObjectSet &src)
{
    static const std::string header = "% bbGt version=3\n";
    os << header;
    for(int i = 0; i < src.objects.size(); i++)
    {
        os << src.objects[i] << std::endl;
    }

    return os;
}


DRISHTI_ACF_NAMESPACE_END

#endif /* defined(__DRISHTI__ACFOBJECT__) */
