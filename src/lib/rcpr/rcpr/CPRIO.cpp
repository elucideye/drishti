/*!
  @file   CPRIO.cpp
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @brief  Implementation of (de)serialization routines for Cascaded Pose Regression class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "rcpr/CPR.h"

// This is the original matlab serialization code

#if !DO_LEAN_CPR

DRISHTI_RCPR_BEGIN

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

DRISHTI_RCPR_BEGIN

#endif


