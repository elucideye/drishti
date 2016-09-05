/*!
  @file   chnsCompute.cpp
  @author David Hirvonen
  @author P. Dollár (original matlab code)
  @brief  Computation of Aggregated Channel Features for a single image.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// function chns = chnsCompute( I, varargin )
// Compute channel features at a single scale given an input image.
//
// Compute the channel features as described in:
//  P. Dollár, Z. Tu, P. Perona and S. Belongie
//  "Integral Channel Features", BMVC 2009.
// Channel features have proven very effective in sliding window object
// detection, both in terms of *accuracy* and *speed*. Numerous feature
// types including histogram of gradients (hog) can be converted into
// channel features, and overall, channels are general and powerful.
//
// Given an input image I, a corresponding channel is a registered map of I,
// where the output pixels are computed from corresponding patches of input
// pixels (thus preserving overall image layout). A trivial channel is
// simply the input grayscale image, likewise for a color image each color
// channel can serve as a channel. Other channels can be computed using
// linear or non-linear transformations of I, various choices implemented
// here are described below. The only constraint is that channels must be
// translationally invariant (i.e. translating the input image or the
// resulting channels gives the same result). This allows for fast object
// detection, as the channels can be computed once on the entire image
// rather than separately for each overlapping detection window.
//
// Currently, three channel types are available by default (to date, these
// have proven the most effective for sliding window object detection):
//  (1) color channels (computed using rgbConvert.m)
//  (2) gradient magnitude (computed using gradientMag.m)
//  (3) quantized gradient channels (computed using gradientHist.m)
// For more information about each channel type, including the exact input
// parameters and their meanings, see the respective m-files which perform
// the actual computatons (chnsCompute is essentially a wrapper function).
// The converted color channels serve as input to gradientMag/gradientHist.
//
// Additionally, custom channels can be specified via an optional struct
// array "pCustom" which may have 0 or more custom channel definitions. Each
// custom channel is generated via a call to "chns=feval(hFunc,I,pFunc{:})".
// The color space of I is determined by pColor.colorSpace, use the setting
// colorSpace='orig' if the input image is not an 'rgb' image and should be
// left unchanged (e.g. if I has multiple channels). The input I will have
// type single and the output of hFunc should also have type single.
//
// "shrink" (which should be an integer) determines the amount to subsample
// the computed channels (in applications such as detection subsamping does
// not affect performance). The params for each channel type are described
// in detail in the respective function. In addition, each channel type has
// a param "enabled" that determines if the channel is computed. If
// chnsCompute() is called with no inputs, the output is the complete
// default params (pChns). Otherwise the outputs are the computed channels
// and additional meta-data (see below). The channels are computed at a
// single scale, for (fast) multi-scale channel computation see chnsPyramid.
//
// An emphasis has been placed on speed, with the code undergoing heavy
// optimization. Computing the full set of channels used in the BMVC09 paper
// referenced above on a 480x640 image runs over *100 fps* on a single core
// of a machine from 2011 (although runtime depends on input parameters).
//
// USAGE
//  pChns = chnsCompute()
//  chns = chnsCompute( I, pChns )
//
// INPUTS
//  I           - [hxwx3] input image (uint8 or single/double in [0,1])
//  pChns       - parameters (struct or name/value pairs)
//   .shrink       - [4] integer downsampling amount for channels
//   .pColor       - parameters for color space:
//     .enabled      - [1] if true enable color channels
//     .smooth       - [1] radius for image smoothing (using convTri)
//     .colorSpace   - ['luv'] choices are: 'gray', 'rgb', 'hsv', 'orig'
//   .pGradMag     - parameters for gradient magnitude:
//     .enabled      - [1] if true enable gradient magnitude channel
//     .colorChn     - [0] if>0 color channel to use for grad computation
//     .normRad      - [5] normalization radius for gradient
//     .normConst    - [.005] normalization constant for gradient
//     .full         - [0] if true compute angles in [0,2*pi) else in [0,pi)
//   .pGradHist    - parameters for gradient histograms:
//     .enabled      - [1] if true enable gradient histogram channels
//     .binSize      - [shrink] spatial bin size (defaults to shrink)
//     .nOrients     - [6] number of orientation channels
//     .softBin      - [0] if true use "soft" bilinear spatial binning
//     .useHog       - [0] if true perform 4-way hog normalization/clipping
//     .clipHog      - [.2] value at which to clip hog histogram bins
//   .pCustom      - parameters for custom channels (optional struct array):
//     .enabled      - [1] if true enable custom channel type
//     .name         - ['REQ'] custom channel type name
//     .hFunc        - ['REQ'] function handle for computing custom channels
//     .pFunc        - [{}] additional params for chns=hFunc(I,pFunc{:})
//     .padWith      - [0] how channel should be padded (e.g. 0,'replicate')
//   .complete     - [] if true does not check/set default vals in pChns
//
// OUTPUTS
//  chns       - output struct
//   .pChns      - exact input parameters used
//   .nTypes     - number of channel types
//   .data       - [nTypes x 1] cell [h/shrink x w/shrink x nChns] channels
//   .info       - [nTypes x 1] struct array
//     .name       - channel type name
//     .pChn       - exact input parameters for given channel type
//     .nChns      - number of channels for given channel type
//     .padWith    - how channel should be padded (0,'replicate')
//
// EXAMPLE - default channels
//  I=imResample(imread('peppers.png'),[480 640]); pChns=chnsCompute();
//  tic, for i=1:100, chns=chnsCompute(I,pChns); end; toc
//  figure(1); montage2(cat(3,chns.data{:}));
//
// EXAMPLE - default + custom channels
//  I=imResample(imread('peppers.png'),[480 640]); pChns=chnsCompute();
//  hFunc=@(I) 5*sqrt(max(0,max(convBox(I.^2,2)-convBox(I,2).^2,[],3)));
//  pChns.pCustom=struct('name','Std02','hFunc',hFunc); pChns.complete=0;
//  tic, chns=chnsCompute(I,pChns); toc
//  figure(1); im(chns.data{4});
//
// See also rgbConvert, gradientMag, gradientHist, chnsPyramid
//
// Piotr's Image&Video Toolbox      Version 3.23
// Copyright 2013 Piotr Dollar & Ron Appel.  [pdollar-at-caltech.edu]
// Please email me if you find bugs, or have suggestions or questions!
// Licensed under the Simplified BSD License [see external/bsd.txt]

#include "drishti/acf/ACF.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

DRISHTI_ACF_BEGIN

static int addChn(Detector::Channels &chns, const MatP &data, const std::string &name, const std::string &padWith, int h, int w);

int Detector::chnsCompute(const MatP &IIn, const Options::Pyramid::Chns &pChnsIn, Detector::Channels &chns, bool isInit, MatLoggerType pLogger)
{
    Options::Pyramid::Chns pChns = pChnsIn;
    if( !pChnsIn.complete.has || (pChnsIn.complete.get() != 1) || IIn.empty() )
    {
        // Create default structures and merge:

        {
            // top level
            Options::Pyramid::Chns dfs;
            dfs.shrink = {"shrink", 4};
            dfs.complete = {"complete", 1};
            pChns.merge(dfs, 1);
        }
        {
            // pColor
            Options::Pyramid::Chns::Color dfs;
            dfs.enabled = { "enabled", 1 };
            dfs.smooth = { "smooth", 1 };
            dfs.colorSpace = { "colorSpace", "luv" };
            pChns.pColor.merge(dfs, 1);
        }
        {
            // pGradMag
            Options::Pyramid::Chns::GradMag dfs;
            dfs.enabled = { "enabled", 1 };
            dfs.colorChn = { "colorChn", 0 };
            dfs.normRad = { "normRad", 5 };
            dfs.normConst = { "normConst", 0.005 };
            dfs.full = { "full", 0  };
            pChns.pGradMag.merge(dfs, 1);;
        }
        {
            // pGradHist
            Options::Pyramid::Chns::GradHist dfs;
            dfs.enabled = { "enabled", 1 };
            dfs.nOrients = { "nOrients", 6 };
            dfs.softBin = { "softBin", 0 };
            dfs.useHog = { "useHog", 0 };
            dfs.clipHog = { "clipHog", 0.2 };
            pChns.pGradHist.merge(dfs, 1);
        }
        //std::cout << pChns << std::endl;
        // TODO
    }

    // Create output struct:
    Channels::Info info;

    // Crop I so divisible by shrink and get target dimensions:
    MatP I, MO;
    auto shrink = pChns.shrink.get();
    int h = IIn.rows();
    int w = IIn.cols();
    cv::Size cr( w % shrink, h % shrink );
    if(cr.width || cr.height)
    {
        h = h - cr.height;
        w = w - cr.width;
        I = IIn(cv::Range(0,h), cv::Range(0,w));
    }
    else
    {
        I = IIn;
    }

    if(I.channels() > 3)
    {
        std::copy(I.begin() + 3, I.end(), std::back_inserter(MO));
        while(I.channels() > 3)
        {
            I.pop_back();
        }
    }


    h = h/shrink;
    w = w/shrink;

    {
        // Compute color channels:
        auto p = pChns.pColor.get();
        std::string nm = "color channels";
        rgbConvert(I, I, p.colorSpace, true);

        if(I.channels())
        {
            convTri(I, I, p.smooth, 1);

            if(pLogger)
            {
                std::vector<char> TAGS { 'L', 'U', 'V' };
                for(int i = 0; i < 3; i++)
                {
                    std::stringstream ss;
                    ss << TAGS[i] << ":" << I[i].cols << "x" << I[i].rows;
                    pLogger(I[i], ss.str());
                }
            }
        }

        if(p.enabled.get())
        {
            addChn(chns, I, nm, "replicate", h, w);
        }
    }

    int full = 0;

    cv::Mat M, O;

    {
        // Compute gradient magnitude channel:
        auto p = pChns.pGradMag.get();
        std::string nm = "gradient magnitude";
        full = (p.full.has) ? p.full.get() : 0;

        if(MO.channels() == 2)
        {
            M = MO[0];
            O = MO[1];

            //cv::Vec2d vals;
            //cv::minMaxLoc(O, &vals[0], &vals[1]);
            //std::cout << vals << std::endl;
        }
        else if(I.channels())
        {
            if( pChns.pGradHist->enabled )
            {
                gradientMag(I[p.colorChn], M, O, /*p.colorChn*/ 0, p.normRad, p.normConst, full, pLogger);
            }
            else if( p.enabled )
            {
                gradientMag(I[p.colorChn], M, O, /*p.colorChn*/ 0, p.normRad, p.normConst, full, pLogger);
            }

            if(pLogger && !M.empty() && !O.empty())
            {
                {
                    // Log gradient magnitude:
                    std::stringstream ss;
                    ss << "Mnorm:" << M.cols << "x" << M.rows;
                    pLogger(M, ss.str());
                }

                {
                    // Log orientation:
                    std::stringstream ss;
                    ss << "O:" << O.cols << "x" << O.rows;
                    pLogger(O, ss.str());
                }
            }
        }

        if( p.enabled )
        {
            MatP Mp(M);
            addChn(chns, Mp, nm, {}, h, w);
        }
    }

    {
        // Compute gradient histogram channels:
        auto p = pChns.pGradHist.get();
        std::string nm = "gradient histogram";
        if(p.enabled.get())
        {
            int binSize = (p.binSize.has) ? p.binSize.get() : shrink;
            MatP Hp;
            if(!M.empty())
            {
                gradientHist(M, O, Hp, binSize, p.nOrients, p.softBin, p.useHog, p.clipHog, full);
                if(pLogger && !I.empty())
                {
                    cv::Mat canvas, h;
                    cv::hconcat(Hp.get(), h);

                    std::stringstream ss;
                    ss << "H:" << h.cols << "x" << h.rows;
                    pLogger(h, ss.str());
                }
            }

            addChn(chns, Hp, nm, {}, h, w);
        }
    }
    chns.pChns = pChns;

    return 0;
}

static int addChn(Detector::Channels &chns, const MatP &dataIn, const std::string &name, const std::string &padWith, int h, int w)
{
    //[h1,w1,~]=size(data);
    //if(h1~=h || w1~=w), data=imResampleMex(data,h,w,1);
    //assert(all(mod([h1 w1]./[h w],1)==0)); end

    MatP data;
    if(dataIn.size() != cv::Size(w,h))
    {
        data.create( cv::Size(w,h), dataIn.depth(), dataIn.channels() );

#if 0
        // OpenCV resize is typically a little faster than resample acf code are similar:
        //
        //907802(opencv:)  1130427(acf:)
        //238942(opencv:)  345563(acf:)
        //1376805(opencv:) 1635851(acf:)
        //185866(opencv:)  308654(acf:)
        //70380(opencv:)   76641(acf:)
        //445637(opencv:)  341431(acf:)
        //36914(opencv:)   97596(acf:)
        //16854(opencv:)   19379(acf:)
        //170856(opencv:)  554549(acf:)
        //11789(opencv:)   13580(acf:)
        cv::Mat tmpA, tmpB;
        cv::merge(A.get(), tmpA);
        cv::resize(tmpA, tmpB, {size.width,size.height});
        cv::split(tmpB, B.get());
#else
        imResample(dataIn, data, cv::Size(w, h), 1.0);
#endif

    }
    else
    {
        data = dataIn;
    }

    CV_Assert( data.size() == cv::Size(w,h) );

    // TODO: chns.info (C++ requires strong type)
    chns.nTypes++;
    chns.data.push_back( data );

    Detector::Channels::Info info;
    info.name = name;
    info.nChns = dataIn.channels();
    info.padWith = padWith;
    chns.info.emplace_back(info);

    return 0;
}

DRISHTI_ACF_END
