/*!
  @file   rgbConvert.cpp
  @author David Hirvonen (C++ implementation)
  @author P. Dollár (original matlab code)
  @brief  Colorspace conversions for RGB input.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// function J = rgbConvert( I, colorSpace, useSingle )
// Convert RGB image to other color spaces (highly optimized).
//
// If colorSpace=='gray' transforms I to grayscale. The output is within
// numerical error of Matlab's rgb2gray, except ~10x faster. The output in
// this case is hxwx1, and while the input must be hxwx3 for all other
// cases, the input for this case can also be hxwx1 (normalization only).
//
// If colorSpace=='hsv' transforms I to the HSV color space. The output is
// within numerical error of Matlab's rgb2hsv, except ~15x faster.
//
// If colorSpace=='rgb' or colorSpace='orig' only normalizes I to be in the
// range [0,1]. In this case both the input and output may have an arbitrary
// number of channels (that is I may be [hxwxd] for any d).
//
// If colorSpace=='luv' transforms I to the LUV color space. The LUV color
// space is "perceptually uniform" (meaning that two colors equally distant
// in the color space according to the Euclidean metric are equally distant
// perceptually). The L,u,v channels correspond roughly to luminance,
// green-red, blue-yellow. For more information see:
//   http://en.wikipedia.org/wiki/CIELUV - using this color spaces
//   http://en.wikipedia.org/wiki/CIELAB - more info about color spaces
// The LUV channels are normalized to fall in ~[0,1]. Without normalization
// the ranges are L~[0,100], u~[-88,182], and v~[-134,105] (and typically
// u,v~[-100,100]). The applied transformation is L=L/270, u=(u+88)/270, and
// v=(v+134)/270. This results in ranges L~[0,.37], u~[0,1], and v~[0,.89].
// Perceptual uniformity is maintained since divisor is constant
// (normalizing each color channel independently would break uniformity).
// To undo the normalization on an LUV image J use:
//   J=J*270; J(:,:,2)=J(:,:,2)-88; J(:,:,3)=J(:,:,3)-134;
// To test the range of the colorSpace use:
//   R=100; I=zeros(R^3,1,3); k=1; R=linspace(0,1,R);
//   for r=R, for g=R, for b=R, I(k,1,:)=[r g b]; k=k+1; end; end; end
//   J=rgbConvert(I,'luv'); [min(J), max(J)]
//
// This code requires SSE2 to compile and run (most modern Intel and AMD
// processors support SSE2). Please see: http://en.wikipedia.org/wiki/SSE2.
//
// USAGE
//  J = rgbConvert( I, colorSpace, [useSingle] );
//
// INPUTS
//  I          - [hxwx3] input rgb image (uint8 or single/double in [0,1])
//  colorSpace - ['luv'] other choices include: 'gray', 'hsv', 'rgb', 'orig'
//  useSingle  - [true] determines output type (faster if useSingle)
//
// OUTPUTS
//  J          - [hxwx3] single or double output image (normalized to [0,1])
//
// EXAMPLE - luv
//  I = imread('peppers.png');
//  tic, J = rgbConvert( I, 'luv' ); toc
//  figure(1); montage2( J );
//
// EXAMPLE - hsv
//  I=imread('peppers.png');
//  tic, J1=rgb2hsv( I ); toc
//  tic, J2=rgbConvert( I, 'hsv' ); toc
//  mean2(abs(J1-J2))
//
// EXAMPLE - gray
//  I=imread('peppers.png');
//  tic, J1=rgb2gray( I ); toc
//  tic, J2=rgbConvert( I, 'gray' ); toc
//  J1=single(J1)/255; mean2(abs(J1-J2))
//
// See also rgb2hsv, rgb2gray
//
// Piotr's Image&Video Toolbox      Version 3.02
// Copyright 2012 Piotr Dollar.  [pdollar-at-caltech.edu]
// Please email me if you find bugs, or have suggestions or questions!
// Licensed under the Simplified BSD License [see external/bsd.txt]

#include "acf/ACF.h"

#include <opencv2/imgproc/imgproc.hpp>

using namespace string_hash;

void rgbConvertMex(const MatP &I, MatP &J, int flag, double nrm);

DRISHTI_ACF_BEGIN

#define USE_OPENCV_CVTCOLOR 0

#if USE_OPENCV_CVTCOLOR
// TODO: replace this with mex file code (or rework flow to avoid this)
static void cvtColor(const MatP &src, MatP &dst, int code)
{
    cv::Mat src_, dst_;
    cv::merge(src.get(), src_);
    cv::cvtColor(src_, dst_, code);
    dst = MatP(dst_);
}
#endif

int Detector::rgbConvert(const MatP &IIn, MatP &J, const std::string &colorSpace, bool useSingle, bool isLuv )
{
    std::string cs;
    std::transform(colorSpace.begin(), colorSpace.end(), std::back_inserter(cs), ::tolower);

    int flag = 2;
    switch( string_hash::hash(cs) )
    {
        case "gray"_hash :
            flag = 0;
            break;
        case "rgb"_hash  :
            flag = 1;
            break;
        case "luv"_hash  :
            flag = 2;
            break;
        case "hsv"_hash  :
            flag = 3;
            break;
        case "orig"_hash :
            flag = 4;
            break;
        default :
            CV_Assert(false);
    }

    if(isLuv)
    {
        CV_Assert((cs == "luv") || (cs == "orig"));
        J = IIn;
        return 0;
    }

#if USE_OPENCV_CVTCOLOR
    if(IIn.empty())
    {
        J = cv::Mat3f();
        return 0;
    }

    // Just use opencv calls for now (be sure to compare w/ Piotr's code)
    switch(flag)
    {
        case 0:
            cvtColor( IIn, J, cv::COLOR_GRAY2RGB );
            break;
        case 2:
            cvtColor( IIn, J, cv::COLOR_RGB2Luv );
            break;
        case 3:
            cvtColor( IIn, J, cv::COLOR_RGB2HSV );
            break;
        case 1:
            J = IIn;
            break;
        case 4:
            J = IIn;
            break;
        default:
            CV_Assert(flag >= 0 && flag <= kinds.size());
    }
#else
    CV_Assert(flag >= 0);
    if(!IIn.empty() && (flag != 1) && (flag != 4))
    {
        rgbConvertMex(IIn, J, flag, useSingle);
    }
#endif

    return 0;
}

DRISHTI_ACF_END
