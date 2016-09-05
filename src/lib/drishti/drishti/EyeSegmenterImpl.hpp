/**
  @file   EyeSegmenterImpl.hpp
  @author David Hirvonen
  @brief  Top level API eye model declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the private implementation structure
  used for the top level SDK -- this provides a cleaner API, minimizes dependencies
  and improve build times.
*/

#ifndef __drishtisdk__DrishtiEyeSegmenterImpl__
#define __drishtisdk__DrishtiEyeSegmenterImpl__

#include "drishti/EyeSegmenter.hpp"
#include "drishti/core/Logger.h"

// *INDENT-OFF*
namespace drishti { namespace eye { class EyeModelEstimator; } };
// *INDENT-ON*

_DRISHTI_SDK_BEGIN

class DRISHTI_EXPORTS EyeSegmenter::Impl
{
public:
    Impl(bool doLoad=false);
    Impl(const std::string &filename);
    Impl(std::istream &is);
    ~Impl();
    int operator()(const Image3b &image, Eye &eye, bool isRight);

    Eye getMeanEye(int width) const;

    void setEyelidInits(int count);
    int getEyelidInits() const;

    void setIrisInits(int count);
    int getIrisInits() const;

    static int getMinWidth();

    static float getRequiredAspectRatio();

    void setOptimizationLevel(int level);

protected:

    void init(std::istream &is);

    std::unique_ptr<eye::EyeModelEstimator> m_eme;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

_DRISHTI_SDK_END

#endif /* defined(__drishtisdk__DrishtiEyeSegmenterImpl__) */
