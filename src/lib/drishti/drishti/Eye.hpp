/*!
  @file   Eye.hpp
  @author David Hirvonen
  @brief  Top level API eye model declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the eye model structure used
  to report results for the top level SDK.
*/

#ifndef __drishti_drishti_Eye_hpp__
#define __drishti_drishti_Eye_hpp__

#include "drishti/drishti_sdk.hpp"
#include "drishti/Image.hpp"

#include <vector>
#include <iostream>

_DRISHTI_SDK_BEGIN

/*
 * Eye type
 */

class DRISHTI_EXPORTS Eye
{
public:
    struct Ellipse
    {
        Vec2f center = {0.f,0.f};
        Size2f size = {0.f,0.f};
        float angle = 0.f;
    };

    Eye();
    Eye(const Eye &src);

    void setIris(const Ellipse &src)
    {
        iris = src;
    }
    void setPupil(const Ellipse &src)
    {
        pupil = src;
    }
    void setEyelids(const std::vector<Vec2f>& src)
    {
        eyelids = src;
    }
    void setCrease(const std::vector<Vec2f>& src)
    {
        crease = src;
    }
    void setCorners(const Vec2f &inner, const Vec2f &outer)
    {
        innerCorner = inner;
        outerCorner = outer;
    }
    void setRoi(const Recti &src)
    {
        roi = src;
    }

    const Ellipse& getIris() const
    {
        return iris;
    }
    Ellipse& getIris()
    {
        return iris;
    }

    const Ellipse& getPupil() const
    {
        return pupil;
    }
    Ellipse& getPupil()
    {
        return pupil;
    }

    const std::vector<Vec2f> &getEyelids() const
    {
        return eyelids;
    }
    std::vector<Vec2f> &getEyelids()
    {
        return eyelids;
    }

    const std::vector<Vec2f> &getCrease() const
    {
        return crease;
    }
    std::vector<Vec2f> &getCrease()
    {
        return crease;
    }

    const Vec2f & getInnerCorner() const
    {
        return innerCorner;
    }
    Vec2f & getInnerCorner()
    {
        return innerCorner;
    }

    const Vec2f & getOuterCorner() const
    {
        return outerCorner;
    }
    Vec2f & getOuterCorner()
    {
        return outerCorner;
    }

    const Recti & getRoi() const
    {
        return roi;
    }
    Recti & getRoi()
    {
        return roi;
    }

protected:

    Ellipse iris;
    Ellipse pupil;
    std::vector<Vec2f> eyelids;
    std::vector<Vec2f> crease;
    Vec2f innerCorner;
    Vec2f outerCorner;
    Recti roi;
};

struct DRISHTI_EXPORTS EyeStream
{
    enum Format { XML, JSON };
    EyeStream(const Format &format) : format(format) {}
    std::string ext() const;
    Format format = XML;
};

struct DRISHTI_EXPORTS EyeOStream : public EyeStream
{
    EyeOStream(const Eye &eye, Format format) : EyeStream(format), eye(eye) {}
    const Eye &eye;
};

struct DRISHTI_EXPORTS EyeIStream : public EyeStream
{
    EyeIStream(Eye &eye, Format format) : EyeStream(format), eye(eye) {}
    Eye &eye;
};

DRISHTI_EXPORTS std::ostream& operator<<(std::ostream &os, const EyeOStream &eye);
DRISHTI_EXPORTS std::istream& operator>>(std::istream &is, EyeIStream &eye);

enum EyeRegions
{
    kScleraRegion  = 1,
    kIrisRegion    = 2,
    kPupilRegion   = 4
};

void DRISHTI_EXPORTS createMask(Image1b &mask, const Eye &eye, int components=kIrisRegion);

_DRISHTI_SDK_END

#endif // __drishti_drishti_Eye_hpp__

