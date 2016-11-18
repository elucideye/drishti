/*!
  @file   CPRIO.cpp
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @brief  Implementation of (de)serialization routines for Cascaded Pose Regression class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/rcpr/CPR.h"

#if !DRISHTI_CPR_DO_LEAN

DRISHTI_RCPR_NAMESPACE_BEGIN

// This is the original matlab serialization code

int CPR::deserialize(const std::string &filename)
{
    m_isMat = filename.rfind(".mat") != std::string::npos;
    if(m_isMat)
    {
        return deserialize(filename.c_str());
    }
    return 0;
}

static void ParseNode(acf::ParserNode< acf::Field<rcpr::CPR::Model>> &model)
{
    // Parse model parts:
    auto && parts_ = model.create("parts", (*model)->parts );
    parts_.parse<decltype((*parts_)->prn)>("prn", (*parts_)->prn);
    parts_.parse<decltype((*parts_)->lks)>("lks", (*parts_)->lks);
    parts_.parse<decltype((*parts_)->mus)>("mus", (*parts_)->mus);
    parts_.parse<decltype((*parts_)->sigs)>("sigs", (*parts_)->sigs);
    parts_.parse<decltype((*parts_)->wts)>("wts", (*parts_)->wts);

    // Currently nothing else in model struct
}

int CPR::deserialize(const char *filename)
{
    MatlabIO matio;
    acf::ParserNode<CPR> root_(filename, *this);

    {
        // Parse the cpr params
        auto && cprPrm_ = root_.create("cprPrm", cprPrm);
        cprPrm_.parse<acf::Field<double>>( "T", (*cprPrm_)->T );
        cprPrm_.parse<acf::Field<double>>( "L", (*cprPrm_)->L );

        {
            // Parse the model:
            auto && model_ = cprPrm_.create("model", (*cprPrm_)->model );
            ParseNode(model_);
        }
        {
            // Parse the feature parameters:
            auto && ftrPrm_ = cprPrm_.create("ftrPrm", (*cprPrm_)->ftrPrm );
            {
                ftrPrm_.parse<decltype((*ftrPrm_)->type)>("type", (*ftrPrm_)->type);
                ftrPrm_.parse<decltype((*ftrPrm_)->type)>("F", (*ftrPrm_)->F);
                ftrPrm_.parse<decltype((*ftrPrm_)->type)>("radius", (*ftrPrm_)->radius);
            }
        }

        {
            // Parse teh fern parameters:
            auto && fernPrm_ = cprPrm_.create("fernPrm", (*cprPrm_)->fernPrm);
            {
                fernPrm_.parse<decltype((*fernPrm_)->thrr)>("thrr", (*fernPrm_)->thrr);
                fernPrm_.parse<decltype((*fernPrm_)->reg)>("reg", (*fernPrm_)->reg);
                fernPrm_.parse<decltype((*fernPrm_)->S)>("S", (*fernPrm_)->S);
                fernPrm_.parse<decltype((*fernPrm_)->M)>("M", (*fernPrm_)->M);
                fernPrm_.parse<decltype((*fernPrm_)->R)>("R", (*fernPrm_)->R);
                fernPrm_.parse<decltype((*fernPrm_)->eta)>("eta", (*fernPrm_)->eta);
            }
        }
    }

    {
        auto && regModel_ = root_.create("regModel", regModel);
        {
            {
                // Parse the model: (currently repeated)
                auto && model_ = regModel_.create("model", (*regModel_)->model);
                ParseNode(model_);
            }

            {
                // vector<RegModel>

                // TODO: Add Cell<ParserNode<T>> or something similar for matlab cell types, need to think through:
                auto && variables = regModel_.m_matio.find< acf::VecVecContainer >( regModel_.m_variables, "regs" );
                (*regModel_)->regs->resize( variables.size() );
                (*regModel_)->regs.set("reg[]");
                (*regModel_)->regs.mark(true);

                auto &regs = (*regModel_)->regs;
                for(int i = 0; i < variables.size(); i++)
                {
                    auto &reg = (*regs)[i];
                    acf::ParserNode<acf::Field<RegModel::Regs>> reg_("reg", reg, variables[i]);

                    {
                        auto && ferns_ = reg_.create<decltype((*reg_)->ferns)>("ferns", (*reg_)->ferns);
                        ferns_.parse<decltype((*ferns_)->fids)>("fids", (*ferns_)->fids);
                        ferns_.parse<decltype((*ferns_)->thrs)>("thrs", (*ferns_)->thrs);
                        ferns_.parse<decltype((*ferns_)->ysFern)>("ysFern", (*ferns_)->ysFern);
                    }

                    {
                        auto && ftrData_ = reg_.create<decltype((*reg_)->ftrData)>("ftrData", (*reg_)->ftrData);
                        ftrData_.parse<decltype((*ftrData_)->type)>("type", (*ftrData_)->type);
                        ftrData_.parse<decltype((*ftrData_)->F)>("F", (*ftrData_)->F);
                        ftrData_.parse<decltype((*ftrData_)->F)>("nChn", (*ftrData_)->nChn);
                        ftrData_.parse<decltype((*ftrData_)->xs)>("xs", (*ftrData_)->xs);
                        ftrData_.parse<decltype((*ftrData_)->pids)>("pids", (*ftrData_)->pids);
                    }
                    reg_.parse<decltype((*reg_)->r)>("r", (*reg_)->r);
                }
            }

            regModel_.parse<decltype((*regModel_)->pStar)>("pStar", (*regModel_)->pStar);
            regModel_.parse<decltype((*regModel_)->pDstr)>("pDstr", (*regModel_)->pDstr);
            regModel_.parse<decltype((*regModel_)->T)>("T", (*regModel_)->T);
        }
    }

    return 0;
}

DRISHTI_RCPR_NAMESPACE_END

#endif

DRISHTI_RCPR_NAMESPACE_BEGIN

// Move to CPRIO

// Boost serialization:
template<class Archive>
void CPR::Model::Parts::serialize(Archive & ar, const unsigned int version)
{
    ar & prn;
    ar & lks;
    ar & joint;
    ar & mus;
    ar & sigs;
    ar & wts;
}

template<class Archive>
void CPR::Model::serialize(Archive & ar, const unsigned int version)
{
    ar & parts;
}

template<class Archive>
void CPR::CprPrm::FtrPrm::serialize(Archive & ar, const unsigned int version)
{
    ar & type;
    ar & F;
    ar & radius;
    ar & nChn;
}

template<class Archive>
void CPR::CprPrm::FernPrm::serialize(Archive & ar, const unsigned int version)
{
    ar & thrr;
    ar & reg;
    ar & S;
    ar & M;
    ar & R;
    ar & eta;
}

template<class Archive>
void CPR::CprPrm::Recipe::serialize(Archive & ar, const unsigned int version)
{
    ar & maxLeafNodes;
    ar & maxDepth;
    ar & treesPerLevel;
    ar & featurePoolSize;
    ar & featureSampleSize;
    ar & featureRadius;
    ar & learningRate;
    ar & dataSampleRatio;
    ar & useNPD;
    ar & doMask;
    ar & paramIndex;
    ar & lambda;
}

template<class Archive>
void CPR::CprPrm::serialize(Archive & ar, const unsigned int version)
{
    ar & model;
    ar & T;
    ar & L;
    ar & ftrPrm;
#if !DRISHTI_CPR_DO_LEAN
    ar & fernPrm;
#endif
    ar & verbose;
    
    // New experimental features
    ar & cascadeRecipes;
}

template<class Archive>
void CPR::RegModel::Regs::FtrData::serialize(Archive & ar, const unsigned int version)
{
    ar & type;
    ar & F;
    ar & nChn;
    
#if DRISHTI_CPR_DO_HALF_FLOAT
    std::vector<PointHalf> xs_;
    if(Archive::is_loading::value)
    {
        ar & xs_;
        copy(xs_, (*xs));
    }
    else
    {
        copy((*xs), xs_);
        ar & xs_;
    }
#else
    ar & xs;
#endif
    
    ar & pids;
}

template<class Archive>
void CPR::RegModel::Regs::serialize(Archive & ar, const unsigned int version)
{
#if !DRISHTI_CPR_DO_LEAN
    ar & ferns;
#endif
    
    ar & ftrData;
    ar & r;
    ar & xgbdt;
}

template<class Archive>
void CPR::RegModel::serialize(Archive & ar, const unsigned int version)
{
    ar & model;
    ar & pStar;
    ar & pDstr;
    ar & T;
    ar & regs;
    
    if(version >= 1)
    {
        ar & pStar_;
    }
}

template<class Archive>
void CPR::serialize(Archive & ar, const unsigned int version)
{
    boost::serialization::void_cast_register<drishti::rcpr::CPR, drishti::ml::ShapeEstimator>();
    ar & cprPrm;
    ar & regModel;
}

// explicit instantiation:

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
template void CPR::Model::Parts::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::Model::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::CprPrm::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::RegModel::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::serialize<OArchive>(OArchive &ar, const unsigned int);
#endif

typedef portable_binary_iarchive IArchive;
template void CPR::Model::Parts::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::Model::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::CprPrm::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::RegModel::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::serialize<IArchive>(IArchive &ar, const unsigned int);

#if DRISHTI_USE_TEXT_ARCHIVES

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

typedef boost::archive::text_oarchive OArchiveTXT;
template void CPR::Model::Parts::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::Model::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);

typedef boost::archive::text_iarchive IArchiveTXT;
template void CPR::Model::Parts::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::Model::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);

#endif // DRISHTI_USE_TEXT_ARCHIVES

DRISHTI_RCPR_NAMESPACE_END

BOOST_CLASS_EXPORT_IMPLEMENT(drishti::rcpr::CPR);
