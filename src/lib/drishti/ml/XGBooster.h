/*! -*-c++-*-
  @file   XGBooster.h
  @author David Hirvonen
  @brief  Internal declaration of the XGBoost C++ interface class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_ml_XGBooster_h__
#define __drishti_ml_XGBooster_h__

#include "drishti/ml/drishti_ml.h"
#include "drishti/core/Logger.h"

#include <opencv2/core.hpp>

#include <memory>

template <typename T>
using MatrixType = std::vector<std::vector<T>>;

DRISHTI_ML_NAMESPACE_BEGIN

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

        template <class Archive>
        void serialize(Archive& ar, const unsigned int version);
    };

    class Impl;
    XGBooster();
    XGBooster(const Recipe& recipe);
    ~XGBooster();
    float operator()(const std::vector<float>& features);
    void train(const MatrixType<float>& features, const std::vector<float>& values, const MatrixType<uint8_t>& mask = {});

    void read(const std::string& filename);
    void write(const std::string& filename) const;

    // Boost serialization:
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version);

    void setStreamLogger(std::shared_ptr<spdlog::logger>& logger);

protected:
    std::unique_ptr<Impl> m_impl;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

DRISHTI_ML_NAMESPACE_END

#endif // __drishti_ml_XGBooster_h__
