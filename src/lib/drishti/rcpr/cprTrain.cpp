/*!
  @file   cprTrain.cpp
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @brief  Implementation of core CPR training routines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#if !DRISHTI_BUILD_MIN_SIZE

#define DRISHTI_CPR_DO_FEATURE_DEBUG 1
#define DRISHTI_CPR_DO_PREVIEW_GT 0
#define DRISHTI_CPR_DO_PREVIEW_JITTER 0

#include "drishti/rcpr/CPR.h"
#include "drishti/ml/PCA.h"
#include "drishti/geometry/Ellipse.h"
#include "drishti/core/Parallel.h"
#include "drishti/core/timing.h"

// clang-format off
#if DRISHTI_SERIALIZE_WITH_BOOST
#  include "drishti/core/drishti_serialization_boost.h"
#endif
// clang-format on

typedef std::vector<float> T_VECTOR;
typedef std::vector<T_VECTOR> T_MATRIX;

// clang-format off
#if DRISHTI_CPR_DO_FEATURE_DEBUG || DRISHTI_CPR_DO_PREVIEW_GT || DRISHTI_CPR_DO_PREVIEW_JITTER
#  include <opencv2/highgui.hpp>
#  include <opencv2/imgproc.hpp>
#endif
// clang-format on

#include <algorithm>
#include <numeric>
#include <mutex>

DRISHTI_RCPR_NAMESPACE_BEGIN

template <typename T>
using MatrixType = std::vector<std::vector<T>>;

using BoostVec = std::vector<std::shared_ptr<ml::XGBooster>>;

using IntVec = std::vector<int>;

static cv::Mat
draw(BoostVec& gbdt, const EllipseVec& pCur, const ImageMaskPairVec& Is, const IntVec& imgIds, const PointVec& xs)
{
    std::vector<int> features;

    std::sort(features.begin(), features.end());
    std::unique(features.begin(), features.end());

    std::vector<cv::Mat> canvases;
    std::vector<std::vector<cv::Mat>> drawings;
    cv::RNG rng;
    rng.state = 100;

    canvases.resize(2);
    drawings.resize(2);

    cv::Size maxSize(0, 0);
    for (int k = 0; k < 2; k++)
    {
        for (int j = 0; j < 6; j++)
        {
            // visualize the features:
            int i = rng.uniform(0, int(pCur.size()) - 1);
            const cv::Mat& I = Is[imgIds[i]].getImage();

            cv::Mat canvas;
            cv::cvtColor(I, canvas, cv::COLOR_GRAY2BGR);

            CV_Assert(Is[imgIds[i]].getMask().size() == Is[imgIds[i]].getImage().size());
            cv::Mat tmp(canvas.size(), canvas.type(), cv::Scalar(255, 0, 0));
            canvas.copyTo(tmp, Is[imgIds[i]].getMask());

            //cv::Mat It_ = canvas.t();
            const float scale = M_SQRT2;
            drawFeatures(canvas, xs, pCur[i], features, scale, DRISHTI_CPR_TRANSPOSE);
            //drawFeatures(It_, xs, pCur[i], features, scale, 1-DRISHTI_CPR_TRANSPOSE);
            //cv::imshow("canvas", canvas); cv::imshow("It_", It_.t()), cv::waitKey(0);

            cv::RotatedRect e = phiToEllipse(pCur[i], DRISHTI_CPR_TRANSPOSE) * scale;

            cv::ellipse(canvas, e, { 255, 0, 255 }, 1, 8);
            cv::resize(tmp, tmp, {}, scale, scale);
            cv::ellipse(tmp, e, { 255, 0, 255 }, 1, 8);

            drishti::geometry::Ellipse e2(e);
            cv::line(tmp, e.center, e2.getMajorAxisPos(), { 0, 255, 0 }, 1, 8);
            cv::vconcat(canvas, tmp, canvas);

            // Update the max size
            maxSize.width = std::max(maxSize.width, canvas.cols);
            maxSize.height = std::max(maxSize.height, canvas.rows);

            drawings[k].push_back(canvas);
        }
    }

    for (int k = 0; k < 2; k++)
    {
        for (auto& i : drawings[k])
        {
            cv::Mat canvas;
            int padX = maxSize.width - i.cols;
            int padY = maxSize.height - i.rows;
            cv::copyMakeBorder(i, canvas, 0, padY, 0, padX, cv::BORDER_CONSTANT);
            std::swap(i, canvas);
        }

#if DRISHTI_CPR_TRANSPOSE
        cv::hconcat(drawings[k], canvases[k]);
#else
        cv::vconcat(drawings[k], canvases[k]);
        canvases[k] = canvases[k].t();
#endif
    }
    cv::Mat canvas;
    cv::vconcat(canvases, canvas);
    return canvas;
};

#if DRISHTI_CPR_DO_PREVIEW_GT || DRISHTI_CPR_DO_PREVIEW_JITTER

static void
debug_current_and_ground_truth(const cv::Mat& Is, const Vector1d& pCur, const Vector1d& pGtIn, const std::string& name)
{
    cv::Mat canvas;
    cv::cvtColor(Is, canvas, cv::COLOR_GRAY2BGR);
    cv::RotatedRect eCur = phiToEllipse(pCur, DRISHTI_CPR_TRANSPOSE);
    cv::RotatedRect eGt = phiToEllipse(pGtIn, DRISHTI_CPR_TRANSPOSE);

    drishti::geometry::Ellipse xCur(eCur);
    drishti::geometry::Ellipse xGt(eGt);

    cv::ellipse(canvas, eCur, { 0, 255, 0 }, 1, 8);
    cv::line(canvas, eCur.center, xCur.getMajorAxisPos(), { 0, 255, 0 }, 2, 8);

    cv::ellipse(canvas, eGt, { 255, 0, 0 }, 1, 8);
    cv::line(canvas, eGt.center, xGt.getMajorAxisPos(), { 0, 255, 0 }, 2, 8);

    cv::imshow(name, canvas);
}

#endif

// % augment data amount
// pCur=repmat(pStar,N*L,1);
// imgIds=repmat(1:N,[1 L]);
// for m=2:L, is=(1:N)+(m-1)*N;
//   pTmp = poseGt('inverse',model,pGt(randperm(N),:));
//   pTmp = poseGt('compose',model,pTmp,pCur(is,:));
//   pCur(is,:) = poseGt('compose',model,pGt,pTmp);
// end;
// pGt=repmat(pGt,[L 1]); N1=N; N=N*L;

static void
transform(const CPR::Model& model, const HVec& Hs, const EllipseVec& pGtIn, EllipseVec& pGtIn_, Vector1d& pStar_)
{
    pGtIn_.resize(Hs.size());
    for (int i = 0; i < pGtIn.size(); i++)
    {
        RealType asp = pGtIn[i].back();
        pGtIn_[i] = phisFrHs(Hs[i] * phisToHs(pGtIn[i]));
        pGtIn_[i].push_back(asp);
    }
    pStar_ = compPhiStar(model, pGtIn_);
}

template <typename T>
std::vector<T> operator*(const cv::Matx33f& H, std::vector<T>& phi)
{
    std::vector<T> phi2;
    phi2 = phisFrHs(H * phisToHs(phi));
    phi2.push_back(phi.back());
    return phi2;
}

// Hs can be used to specify some normalization estimate.
// For eye estimation this could be procrustes normalization of the eyelid, such that the mean iris could be estimated
// in the normalized eye coordinate system.

int CPR::cprTrain(const ImageMaskPairVec& Is, const EllipseVec& pGtIn, const HVec& Hs, const CprPrm& cprPrm, bool doJitter)
{
    this->cprPrm = cprPrm; // store these

    auto ftrPrm = *(cprPrm.ftrPrm);
    const auto& model = *(cprPrm.model);

    const int N = int(Is.size());
    const int T = *(cprPrm.T);
    const int L = (*cprPrm.L);

    cv::RNG rng;

    for (auto& i : Is)
    {
        CV_Assert(i.getImage().size() == i.getMask().size());
    }

    trainingLog.clear();

    // Ground truth in normalized coordinate system if transformations are given:
    Vector1d pStar_;
    EllipseVec pGtIn_;
    if (Hs.size())
    {
        transform(*(cprPrm.model), Hs, pGtIn, pGtIn_, pStar_);
    }

    // Compute mean model:
    Vector1d pStar = compPhiStar(*(cprPrm.model), pGtIn); // need to normalize

    // Create vector to store current estimate for each pose:
    EllipseVec pCur(N * L, (pStar_.size() ? pStar_ : pStar)); // repmat of pStar

    // If we have normalization params, then map back to image coordinate system:
    if (Hs.size())
    {
        for (int i = 0; i < pCur.size(); i++)
        {
            int j = (i % N);
            pCur[i] = Hs[j].inv() * pCur[i];

#if DRISHTI_CPR_DO_PREVIEW_GT
            debug_current_and_ground_truth(Is[j].getImage(), pCur[i], pGtIn[j], "gt");
            cv::waitKey(0);
#endif
        }
    }

    EllipseVec pGt(N * L);
    std::vector<int> imgIds(N * L);
    std::iota(imgIds.begin(), imgIds.end(), 0);
    std::copy(pGtIn.begin(), pGtIn.end(), pGt.begin());

    Vector1d pTmp;
    for (int i = N; i < (N * L); i++)
    {
        int j = (i % N);
        int k = rng.uniform(0, N - 1);
        imgIds[i] = j;
        pGt[i] = pGt[j];

        if (doJitter && Hs.size())
        {
            CV_Assert(Hs.size() == pGtIn.size());
            Vector1d random = Hs[j].inv() * pGtIn_[k];

            RealType alpha = rng.uniform(0., 1.);
            pCur[i] = (random * alpha) + (pCur[i] * (RealType(1) - alpha));
        }
        else
        {
            // Compute inverse of phis0 so that phis0+phis1=phis1+phis0=identity.
            pTmp = inverse(model, pGt[k]);
            pTmp = compose(model, pTmp, pCur[i]);
            pCur[i] = compose(model, pGt[j], pTmp);
        }

#if DRISHTI_CPR_DO_PREVIEW_JITTER
        cv::imshow("other", Is[k].getImage());
        debug_current_and_ground_truth(Is[imgIds[i]].getImage(), pCur[i], pGt[i], "jitter");
        cv::waitKey(0);
#endif
    }

    for (auto& p : pCur)
    {
        p[2] = pStar[2]; // fix angle and aspect ratio to that of the mean
        p[4] = pStar[4];
    }

    const int R = int(pCur.front().size());

    // #### Store the features and the learned trees ####
    std::vector<acf::Field<RegModel::Regs>> regs;

    CV_Assert(T == cprPrm.cascadeRecipes.size());

    // Loop and gradually improve pCur
    for (int t = 0; t < T; t++)
    {
        const auto& recipe = cprPrm.cascadeRecipes[t];

        ftrPrm.radius = double(recipe.featureRadius);
        ftrPrm.F = double(recipe.featurePoolSize); // TODO revisit

        MatrixType<uint8_t> mask(pCur.size());
        cv::Mat features(int(pCur.size()), ftrPrm.F, CV_64F);
        cv::Mat values(int(pCur.size()), R, CV_64F);

        // Generate shared features 1x per stage
        CPR::RegModel::Regs::FtrData ftrData;
        ftrsGen({}, ftrPrm, ftrData, cprPrm.cascadeRecipes[t].lambda);

        // For each model:
        for (int i = 0; i < pCur.size(); i++)
        {
            //% get target value for pose
            Vector1d tar;
            tar = inverse({}, pCur[i]); // pCur starts as pStar (mean model)
            tar = compose({}, tar, pGt[i]);

            //% generate and compute pose indexed features
            CPR::FeaturesResult ftrResult;
            featuresComp({}, pCur[i], Is[imgIds[i]], ftrData, ftrResult, recipe.useNPD);

            // TODO: make decision about format, might be better to stick w/ std::vector<std::vector<T>>
            mask[i] = ftrResult.ftrMask;
            cv::Mat(ftrResult.ftrs).reshape(1, 1).copyTo(features.row(i));
            cv::Mat(tar).reshape(1, 1).copyTo(values.row(i));
        }

        features.convertTo(features, CV_32F);
        values.convertTo(values, CV_32F);
        T_MATRIX features_(features.rows);
        for (int i = 0; i < features.rows; i++)
        {
            // Fill in the feature matrix
            T_VECTOR row = features.row(i);
            features_[i] = row;
        }

        const size_t N = pCur.size();
        std::vector<int> phiIndexToRegressor(R, -1);
        std::vector<int> regressorToPhiIndex; // full list of regressors we will estimate this stage
        MatrixType<int> phiSets;
        if (recipe.paramIndex.size())
        {
            regressorToPhiIndex = recipe.paramIndex;
            phiSets = { recipe.paramIndex };
        }
        else
        {
            regressorToPhiIndex = { 0, 1, 2, 3, 4 };
            phiSets = { { 0 }, { 1 }, { 2 }, { 3 }, { 4 }, { 0, 1 } };
        }
        // Reverse map from phi dimension to corresponding regressor
        for (int i = 0; i < regressorToPhiIndex.size(); i++)
        {
            phiIndexToRegressor[regressorToPhiIndex[i]] = i;
        }

        std::vector<std::shared_ptr<ml::XGBooster>> xgbdt(regressorToPhiIndex.size());

        //% train independent 1D regressors for each r, keep best
        //std::vector<double> losses(R, 0.0);
        //std::vector<EllipseVec> pTmps(R + 1, pCur); // add 1 for (x,y)
        //std::vector<T_VECTOR> predictions(R);

        MatrixType<float> predictions(regressorToPhiIndex.size());

        {
            // Estimate regressors
            std::mutex mutex;
            std::function<void(int)> trainRegressor = [&](int i) {
                cv::Mat tmp = values.col(regressorToPhiIndex[i]);
                tmp = tmp.t();

                std::vector<std::vector<float>> data = features_;
                std::vector<float> target = tmp;

                ml::XGBooster::Recipe params;
                params.learningRate = recipe.learningRate;
                params.dataSubsample = recipe.dataSampleRatio;
                params.maxDepth = recipe.maxDepth;
                params.featureSubsample = float(recipe.featureSampleSize) / recipe.featurePoolSize;

                xgbdt[i] = std::make_shared<ml::XGBooster>(params);
                xgbdt[i]->train(data, target, recipe.doMask ? mask : MatrixType<uint8_t>());

                predictions[i].resize(data.size());
                for (int j = 0; j < data.size(); j++)
                {
                    predictions[i][j] = (*xgbdt[i])(data[j]);
                }

                // Now we compose the models, and find parameter producing lowest error
                m_streamLogger->info("done training stage {} param {}", t, i);
            };

            core::ParallelHomogeneousLambda harness(trainRegressor);
            cv::parallel_for_({ 0, int(regressorToPhiIndex.size()) }, harness);
        }

#if DRISHTI_CPR_DO_FEATURE_DEBUG
        if (m_viewer)
        {
            cv::Mat canvas = draw(xgbdt, pCur, Is, imgIds, *ftrData.xs);
            m_viewer("features", canvas);
        }
#endif
        std::vector<double> losses(phiSets.size(), 0.0);
        std::vector<EllipseVec> pTmps(phiSets.size(), pCur);

        {
            // Now search over combinations for best loss:

            // i : index to vector of pose dimension indices
            // j : index to # of training samples
            // k : index over pose dimensions indices of phiSets[i]
            std::function<void(int)> computeLoss = [&](int i) {
                const auto& dims = phiSets[i];
                double loss = 0.0;
                Vector1d del = identity(model);
                for (int j = 0; j < N; j++)
                {
                    for (int k = 0; k < dims.size(); k++)
                    {
                        del[dims[k]] = predictions[phiIndexToRegressor[dims[k]]][j];
                    }

                    pTmps[i][j] = compose(model, pCur[j], del);
                    loss += dist(model, pTmps[i][j], pGt[j]);
                }
                losses[i] = (loss / double(N));
            };
            core::ParallelHomogeneousLambda harness(computeLoss);
            cv::parallel_for_({ 0, int(phiSets.size()) }, harness);
        }

        for (int j = 0; j < losses.size(); j++)
        {
            m_streamLogger->info("loss: {}", losses[j]);
        }

        int best = 0;
        for (int j = best + 1; j < losses.size(); j++)
        {
            if (losses[j] < losses[best])
            {
                best = j;
            }
        }

        std::stringstream ss;
        for (auto& i : phiSets[best])
        {
            ss << i << " ";
        }

        m_streamLogger->info("Best loss {} losses = {}", ss.str(), losses[best]);
        pCur = pTmps[best]; // update current estimate based on best parmeter

        trainingLog.resize(trainingLog.size() + 1);
        trainingLog.back().loss = losses[best];

        //% Stop if loss did not decrease:
        RegModel::Regs reg;
        reg.ftrData = { "ftrData", ftrData };
        reg.r = { "r", best };

        for (int i = 0; i < phiSets[best].size(); i++)
        {
            reg.xgbdt.emplace_back(phiSets[best][i], xgbdt[i]);
        }

        regs.emplace_back("reg", reg, true);
    }

    this->regModel->model = { "model", model };
    this->regModel->pStar = { "pStar", pStar };
    this->regModel->T = { "T", T };
    this->regModel->regs = { "regs", regs };
    this->regModel->pStar_ = { "pStar_", pStar_ }; // New store normalized version

    // this->regModel->pca = pca;

    return 0;
}

// Uiltity

DRISHTI_RCPR_NAMESPACE_END

#endif // !DRISHTI_BUILD_MIN_SIZE
