/*!
  @file   acfModify.cpp
  @author David Hirvonen (C++ implementation)
  @author P. Dollár (original matlab code)
  @brief  Routines to modify ACF detection classification.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/acf/ACF.h"
#include <cmath>

//function detector = acfModify( detector, varargin )
// Modify aggregate channel features object detector.
//
// Takes an object detector trained by acfTrain() and modifies it. Only
// certain modifications are allowed to the detector and the detector should
// never be modified directly (this may cause the detector to be invalid and
// cause segmentation faults). Any valid modification to a detector after it
// is trained should be performed using acfModify().
//
// The parameters 'nPerOct', 'nOctUp', 'nApprox', 'lambdas', 'pad', 'minDs'
// modify the channel feature pyramid created (see help of chnsPyramid.m for
// more details) and primarily control the scales used. The parameters
// 'pNms', 'stride', 'cascThr' and 'cascCal' modify the detector behavior
// (see help of acfTrain.m for more details). Finally, 'rescale' can be
// used to rescale the trained detector (this change is irreversible).
//
// USAGE
//  detector = acfModify( detector, pModify )
//
// INPUTS
//  detector   - detector trained via acfTrain
//  pModify    - parameters (struct or name/value pairs)
//   .nPerOct    - [] number of scales per octave
//   .nOctUp     - [] number of upsampled octaves to compute
//   .nApprox    - [] number of approx. scales to use
//   .lambdas    - [] coefficients for power law scaling (see BMVC10)
//   .pad        - [] amount to pad channels (along T/B and L/R)
//   .minDs      - [] minimum image size for channel computation
//   .pNms       - [] params for non-maximal suppression (see bbNms.m)
//   .stride     - [] spatial stride between detection windows
//   .cascThr    - [] constant cascade threshold (affects speed/accuracy)
//   .cascCal    - [] cascade calibration (affects speed/accuracy)
//   .rescale    - [] rescale entire detector by given ratio
//
// OUTPUTS
//  detector   - modified object detector
//
// EXAMPLE
//
// See also chnsPyramid, bbNms, acfTrain, acfDetect
//
// Piotr's Image&Video Toolbox      Version 3.20
// Copyright 2013 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
// Please email me if you find bugs, or have suggestions or questions!
// Licensed under the Simplified BSD License [see external/bsd.txt]

// get parameters (and copy to detector and pPyramid structs)

DRISHTI_ACF_NAMESPACE_BEGIN

void Detector::Modify::merge(const Modify &src, int mode)
{
    nPerOct.merge(src.nPerOct, mode);
    nOctUp.merge(src.nOctUp, mode);
    nApprox.merge(src.nApprox, mode);
    lambdas.merge(src.lambdas, mode);
    pad.merge(src.pad, mode);
    minDs.merge(src.minDs, mode);
    pNms.merge(src.pNms, mode);
    stride.merge(src.stride, mode);
    cascThr.merge(src.cascThr, mode);
    cascCal.merge(src.cascCal, mode);
    rescale.merge(src.rescale, mode);
}

// Modify detector in place:
int Detector::acfModify( const Detector::Modify &pIn )
{
    auto &p = opts.pPyramid;

    // Since this is a subset
    Modify dflt;
    dflt.nPerOct = {"nPerOct",p->nPerOct};
    dflt.nOctUp = {"nOctUp",p->nOctUp};
    dflt.nApprox = {"nApprox",p->nApprox};
    dflt.lambdas = {"lambdas", p->lambdas};
    dflt.pad = {"pad", p->pad};
    dflt.minDs = {"minDs",p->minDs};
    dflt.pNms = {"pNms",opts.pNms};
    dflt.stride = {"stride", opts.stride};
    dflt.cascThr = {"cascThr", opts.cascThr};
    dflt.cascCal = {"cascCal", 0.0 };
    dflt.rescale = {"rescale", 1.0 };

    int checkExtra = 1;
    Detector::Modify params = pIn;
    params.merge(dflt, checkExtra);

    //std::cout << params << std::endl;

    {
        // Note: In this case the original matlab code used the getPrmDflt with the
        // merged output explicitly assigned to a select/sparse set of field in the
        // original detector.  This isn't supported by the current OPTIONS.merge(dflt)
        // structure approximation that is used in the ACF lib, so we treat this as
        // a special case and perform the merge inline below.
        opts.pPyramid->nPerOct.merge(params.nPerOct, checkExtra);
        opts.pPyramid->nOctUp.merge(params.nOctUp, checkExtra);
        opts.pPyramid->nApprox.merge(params.nApprox, checkExtra);
        opts.pPyramid->lambdas.merge(params.lambdas, checkExtra);
        opts.pPyramid->pad.merge(params.pad, checkExtra);
        opts.pPyramid->minDs.merge(params.minDs, checkExtra);

        opts.pNms.merge(params.pNms, checkExtra);
        opts.stride.merge(params.stride, checkExtra);
        opts.cascThr.merge(params.cascThr, checkExtra);
        opts.cascCal.merge(params.cascCal, checkExtra);
    }

    // TODO: need to update

    // Finalize pPyramid and opts:
    p->complete = { "complete", 0 };
    p->pChns->complete = { "complete", 0 };

    Detector::Pyramid pyramid;
    chnsPyramid({}, &p.get(), pyramid, true); // want pyramid.pPyramid

    p->complete = 1;
    p->pChns->complete = 1;

    double shrink = p->pChns->shrink;
    opts.stride = std::max(1.0, /*std::*/round(double(opts.stride)/shrink)) * shrink;
    opts.pPyramid = pyramid.pPyramid;

    // calibrate and rescale detector:
    clf.hs += (*params.cascCal);

    if(dflt.rescale != 1.0)
    {
        // detector=detectorRescale(detector,rescale);
        CV_Assert(false);
    }

    return 0;
}

DRISHTI_ACF_NAMESPACE_END
