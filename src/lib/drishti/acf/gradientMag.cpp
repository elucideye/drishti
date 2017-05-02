/*!
  @file   gradientMag.cpp
  @author David Hirvonen (C++ implementation)
  @author P. Dollár (original matlab code)
  @brief  Computation of gradient magnitude.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// function [M,O] = gradientMag( I, channel, normRad, normConst, full )
//
// Compute gradient magnitude and orientation at each image location.
//
// If input image has k>1 channels and channel=0, keeps gradient with
// maximum magnitude (over all channels) at each location. Otherwise if
// channel is between 1 and k computes gradient for the given channel.
// If full==1 orientation is computed in [0,2*pi) else it is in [0,pi).
//
// If normRad>0, normalization is performed by first computing S, a smoothed
// version of the gradient magnitude, then setting: M = M./(S + normConst).
// S is computed by S = convTri( M, normRad ).
//
// This code requires SSE2 to compile and run (most modern Intel and AMD
// processors support SSE2). Please see: http://en.wikipedia.org/wiki/SSE2.
//
// USAGE
//  [M,O] = gradientMag( I, [channel], [normRad], [normConst], [full] )
//
// INPUTS
//  I          - [hxwxk] input k channel single image
//  channel    - [0] if>0 color channel to use for gradient computation
//  normRad    - [0] normalization radius (no normalization if 0)
//  normConst  - [.005] normalization constant
//  full       - [0] if true compute angles in [0,2*pi) else in [0,pi)
//
// OUTPUTS
//  M          - [hxw] gradient magnitude at each location
//  O          - [hxw] approximate gradient orientation modulo PI
//
// EXAMPLE
//  I=rgbConvert(imread('peppers.png'),'gray');
//  [Gx,Gy]=gradient2(I);
//  M=sqrt(Gx.^2+Gy.^2);
//  O=atan2(Gy,Gx);
//  full=0;
//  [M1,O1]=gradientMag(I,0,0,0,full);
//  D=abs(M-M1);
//  mean2(D);
//  if(full), o=pi*2; else o=pi; end
//  D=abs(O-O1);
//  D(~M)=0;
//  D(D>o*.99)=o-D(D>o*.99);
//  mean2(abs(D))
//
// See also gradient, gradient2, gradientHist, convTri
//
// Piotr's Image&Video Toolbox      Version 3.23
// Copyright 2013 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
// Please email me if you find bugs, or have suggestions or questions!
// Licensed under the Simplified BSD License [see external/bsd.txt]

//if(nargin<1 || isempty(I)), M=single([]); O=M; return; end
//if(nargin<2 || isempty(channel)), channel=0; end
//if(nargin<3 || isempty(normRad)), normRad=0; end
//if(nargin<4 || isempty(normConst)), normConst=.005; end
//if(nargin<5 || isempty(full)), full=0; end

//if(nargout<=1), M=gradientMex('gradientMag',I,channel,full);
//else [M,O]=gradientMex('gradientMag',I,channel,full); end

//if( normRad==0 ), return; end; S = convTri( M, normRad );
//gradientMex('gradientMagNorm',M,S,normConst); % operates on M

#include "drishti/acf/ACF.h"
#include <opencv2/highgui/highgui.hpp>

void grad1(float* I, float* Gx, float* Gy, int h, int w, int x);
void grad2(float* I, float* Gx, float* Gy, int h, int w, int d);
void gradMag(float* I, float* M, float* O, int h, int w, int d, bool full);
void gradMagNorm(float* M, float* S, int h, int w, float norm);

void gradMag(const cv::Mat& I, cv::Mat& M, cv::Mat& O, int d, bool full)
{
    M.create(I.size(), I.type());
    O.create(I.size(), I.type());
    float* i = const_cast<float*>(I.ptr<float>());
    float* m = M.ptr<float>();
    float* o = O.ptr<float>();
    gradMag(i, m, o, M.cols, M.rows, M.channels(), full);
}

void gradMagNorm(cv::Mat& M, const cv::Mat& S, float norm) // operates on M
{
    float* m = M.ptr<float>();
    float* s = const_cast<float*>(S.ptr<float>());
    gradMagNorm(m, s, M.cols, M.rows, norm);
}

DRISHTI_ACF_NAMESPACE_BEGIN

int Detector::gradientMag(const cv::Mat& I, cv::Mat& M, cv::Mat& O, int channel, int normRad, double normConst, int full, MatLoggerType logger)
{
    if (I.empty())
    {
        return 0;
    }

    cv::Mat data;
    ::gradMag(I, M, O, channel, full); // TODO: support M or M&O

    if (logger)
    {
        std::stringstream ss;
        ss << "M:" << M.cols << "x" << M.rows;
        logger(M, ss.str());
    }

    if (normRad != 0)
    {
        cv::Mat S(I.size(), I.depth());
        MatP Sp(S), Mp(M); // wrappers
        convTri(Mp, Sp, normRad);
        ::gradMagNorm(M, S, normConst);
    }

    return 0;
}

DRISHTI_ACF_NAMESPACE_END
