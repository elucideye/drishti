/*!
  @file   XGBooster.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Internal declaration of the XGBoost C++ interface class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishtisdk__XGBooster__
#define __drishtisdk__XGBooster__

#include "ml/drishti_ml.h"

#include <opencv2/core.hpp>

// Probably want to hide this stuff
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/export.hpp>

#include "core/Logger.h"

#include <memory>

template <typename T>
using MatrixType = std::vector<std::vector<T>>;

_DRISHTI_ML_BEGIN

class XGBooster
{
public:

    struct Recipe
    {
        int numberOfTrees = 512;
        int maxDepth = 5;
        double dataSubsample = 0.5;
        double learningRate = 0.1;
        double featureSubsample = 0.1;
        bool regression = true;

        friend class boost::serialization::access;
        template<class Archive> void serialize(Archive & ar, const unsigned int version);
    };

    class Impl;
    XGBooster();
    XGBooster(const Recipe &recipe);
    float operator()(const std::vector<float> &features);
    void train(const MatrixType<float> &features, const std::vector<float> &values, const MatrixType<uint8_t> &mask= {});

    void read(const std::string &filename);
    void write(const std::string &filename) const;

    // Boost serialization:
    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive & ar, const unsigned int version);

    void setStreamLogger(std::shared_ptr<spdlog::logger> &logger);

protected:

    std::shared_ptr<Impl> m_impl;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

_DRISHTI_ML_END

BOOST_CLASS_EXPORT_KEY(drishti::ml::XGBooster);
BOOST_CLASS_EXPORT_KEY(drishti::ml::XGBooster::Impl);

#endif /* defined(__drishtisdk__XGBooster__) */
