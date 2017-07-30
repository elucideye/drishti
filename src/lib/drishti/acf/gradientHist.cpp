/*!
  @file   gradientHist.cpp
  @author David Hirvonen (C++ implementation)
  @author P. Dollár (original matlab code)
  @brief  Computation of gradient histogram.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// function H = gradientHist( M, O, varargin )
//
// Compute oriented gradient histograms.
//
// For each binSize x binSize region in an image I, computes a histogram of
// gradients, with each gradient quantized by its angle and weighed by its
// magnitude. If I has dimensions [hxw], the size of the computed feature
// vector H is floor([h/binSize w/binSize nOrients]).
//
// This function implements the gradient histogram features described in:
//   P. Dollár, Z. Tu, P. Perona and S. Belongie
//   "Integral Channel Features", BMVC 2009.
// These features in turn generalize the HOG features introduced in:
//   N. Dalal and B. Triggs, "Histograms of Oriented
//   Gradients for Human Detection," CVPR 2005.
// Setting parameters appropriately gives almost identical features to the
// original HOG or updated FHOG features, see hog.m and fhog.m for details.
//
// The input to the function are the gradient magnitude M and orientation O
// at each image location. See gradientMag.m for computing M and O from I.
//
// The first step in computing the gradient histogram is simply quantizing
// the magnitude M into nOrients [hxw] orientation channels according to the
// gradient orientation. The magnitude at each location is placed into the
// two nearest orientation bins using linear interpolation if softBin >= 0
// or simply to the nearest orientation bin if softBin < 0. Next, spatial
// binning is performed by summing the pixels in each binSize x binSize
// region of each [hxw] orientation channel. If "softBin" is odd each pixel
// can contribute to multiple spatial bins (using bilinear interpolation),
// otherwise each pixel contributes to a single spatial bin. The result of
// these steps is a floor([h/binSize w/binSize nOrients]) feature map
// representing the gradient histograms in each image region.
//
// Parameter settings of particular interest:
//  binSize=1: simply quantize the gradient magnitude into nOrients channels
//  softBin=1, useHog=1, clip=.2: original HOG features (see hog.m)
//  softBin=-1; useHog=2, clip=.2: FHOG features (see fhog.m)
//  softBin=0, useHog=0: channels used in Dollar's BMVC09 paper
//
// This code requires SSE2 to compile and run (most modern Intel and AMD
// processors support SSE2). Please see: http://en.wikipedia.org/wiki/SSE2.
//
// USAGE
//  H = gradientHist( M, O, [binSize,nOrients,softBin,useHog,clipHog,full] )
//
// INPUTS
//  M        - [hxw] gradient magnitude at each location (see gradientMag.m)
//  O        - [hxw] gradient orientation in range defined by param flag
//  binSize  - [8] spatial bin size
//  nOrients - [9] number of orientation bins
//  softBin  - [1] set soft binning (odd: spatial=soft, >=0: orient=soft)
//  useHog   - [0] 1: compute HOG (see hog.m), 2: compute FHOG (see fhog.m)
//  clipHog  - [.2] value at which to clip hog histogram bins
//  full     - [false] if true expects angles in [0,2*pi) else in [0,pi)
//
// OUTPUTS
//  H        - [w/binSize x h/binSize x nOrients] gradient histograms
//
// EXAMPLE
//  I=rgbConvert(imread('peppers.png'),'gray'); [M,O]=gradientMag(I);
//  H1=gradientHist(M,O,2,6,0); figure(1); montage2(H1);
//  H2=gradientHist(M,O,2,6,1); figure(2); montage2(H2);
//
// See also gradientMag, gradient2, hog, fhog
//
// Piotr's Image&Video Toolbox      Version 3.23
// Copyright 2013 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
// Please email me if you find bugs, or have suggestions or questions!
// Licensed under the Simplified BSD License [see external/bsd.txt]

#include "drishti/acf/ACF.h"
#include "drishti/acf/drishti_acf.h"
#include <opencv2/highgui/highgui.hpp>

void gradQuantize(float* O, float* M, int* O0, int* O1, float* M0, float* M1, int nb, int n, float norm, int nOrients, bool full, bool interpolate);
void gradHist(float* M, float* O, float* H, int h, int w, int bin, int nOrients, int softBin, bool full);

void gradHist(const cv::Mat& M, const cv::Mat& O, MatP& H, int bin, int nOrients, int softBin, bool full)
{
    H.create({ M.cols / bin, M.rows / bin }, M.depth(), nOrients);
    H.setTo<float>(0.f);

    float* m = const_cast<float*>(M.ptr<float>());
    float* o = const_cast<float*>(O.ptr<float>());

    int w = M.cols;
    int h = M.rows;
    std::swap(w, h);

    gradHist(m, o, H.ptr<float>(), h, w, bin, nOrients, softBin, full);
}

DRISHTI_ACF_NAMESPACE_BEGIN

int Detector::gradientHist(const cv::Mat& M, const cv::Mat& O, MatP& H, int binSize, int nOrients, int softBin, int useHog, double clipHog, int full)
{
    gradHist(M, O, H, binSize, nOrients, softBin, full);
    // TODO: useHog, clipHog, etc
    return 1;
}

DRISHTI_ACF_NAMESPACE_END
