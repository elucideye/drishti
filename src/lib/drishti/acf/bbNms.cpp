/*!
  @file   bbNms.cpp
  @author David Hirvonen
  @brief  Bounding box non maxima suppression.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/acf/ACF.h"
#include "drishti/core/drishti_algorithm.h"

#include <vector>
#include <numeric>
#include <algorithm>

// function bbs = bbNms( bbs, varargin )
//
// Bounding box (bb) non-maximal suppression (nms).
//
// type=='max': nms of bbs using area of overlap criteria. For each pair of
// bbs, if their overlap, defined by:
// overlap(bb1,bb2) = area(intersect(bb1,bb2))/area(union(bb1,bb2))
// is greater than overlap, then the bb with the lower score is suppressed.
// In the Pascal critieria two bbs are considered a match if overlap>=.5. If
// ovrDnm='min', the 'union' in the above formula is replaced with 'min'.
//
// type=='maxg': Similar to 'max', except performs the nms in a greedy
// fashion. Bbs are processed in order of decreasing score, and, unlike in
// 'max' nms, once a bb is suppressed it can no longer suppress other bbs.

// type='cover': Perform nms by attempting to choose the smallest subset of
// the bbs such that each remaining bb is within overlap of one of the
// chosen bbs. The above reduces to the weighted set cover problem which is
// NP but greedy optimization yields provably good solutions. The score of
// each bb is set to the sum of the scores of the bbs it covers (the max can
//
// type=='ms': Mean shift nms of bbs with a variable width kernel. radii is
// a 4 element vector (x,y,w,h) that controls the amount of suppression
// along each dim. Typically the first two elements should be the same, as
// should the last two. Distance between w/h are computed in log2 space (ie
// w and w*2 are 1 unit apart), and the radii should be set accordingly.
// radii may need to change depending on spatial and scale stride of bbs.
//
// Although efficient, nms is O(n^2). To speed things up for large n, can
// divide data into two parts (according to x or y coordinate), run nms on
// each part, combine and run nms on the result. If maxn is specified, will
// split the data in half if n>maxn. Note that this is a heuristic and can
// change the results of nms. Moreover, setting maxn too small will cause an
// increase in overall performance time.
//
// Finally, the bbs are optionally resized before performing nms. The
// resizing is important as some detectors return bbs that are padded. For
// example, if a detector returns a bounding box of size 128x64 around
// objects of size 100x43 (as is typical for some pedestrian detectors on
// the INRIA pedestrian database), the resize parameters should be {100/128,
// 43/64, 0}, see bbApply>resize() for more info.
//
// USAGE
//  bbs = bbNms( bbs, [varargin] )
//
// INPUTS
//  bbs        - original bbs (must be of form [x y w h wt bbType])
//  varargin   - additional params (struct or name/value pairs)
//   .type       - ['max'] 'max', 'maxg', 'ms', 'cover', or 'none'
//   .thr        - [-inf] threshold below which to discard (0 for 'ms')
//   .maxn       - [inf] if n>maxn split and run recursively (see above)
//   .radii      - [.15 .15 1 1] supression radii ('ms' only, see above)
//   .overlap    - [.5] area of overlap for bbs
//   .ovrDnm     - ['union'] area of overlap denominator ('union' or 'min')
//   .resize     - {} parameters for bbApply('resize')
//   .separate   - [0] run nms separately on each bb type (bbType)
//
// OUTPUTS
//  bbs      - suppressed bbs
//
// EXAMPLE
//  bbs=[0 0 1 1 1; .1 .1 1 1 1.1; 2 2 1 1 1];
//  bbs1 = bbNms(bbs, 'type','max' )
//  bbs2 = bbNms(bbs, 'thr',.5, 'type','ms')
//
// See also bbApply, nonMaxSuprList
//
// Piotr's Image&Video Toolbox      Version 2.60
// Copyright 2012 Piotr Dollar.  [pdollar-at-caltech.edu]
// Please email me if you find bugs, or have suggestions or questions!
// Licensed under the Simplified BSD License [see external/bsd.txt]

// get parameters

using namespace string_hash;

DRISHTI_ACF_NAMESPACE_BEGIN

typedef Detector::Detection Detection;

static std::vector<Detection> nmsMs(const std::vector<Detection> &bbsIn, double thr, const std::vector<double> &radii )
{
    return bbsIn;
}

static std::vector<Detection> nmsCover(const std::vector<Detection> &bbsIn, double overlap, double ovrDnm)
{
    return bbsIn;
}

// Note: This is very close to the opencv rectangle grouping code (need to compare the two)
static std::vector<Detection> nmsMax(const std::vector<Detection> &bbsIn, double overlap, bool greedy, double ovrDnm)
{
    // for each i suppress all j st j>i and area-overlap>overlap:

    //std::cout << bbsIn.size() << std::endl;

    // i.e., ord = sort(bbsIn(:,5), 'descend');  bbs=bbsIn(ord,:)
    auto ord = drishti::core::ordered(bbsIn, [](const Detection &a, const Detection &b)
    {
        return a.score > b.score;
    });
    std::vector<Detector::Detection> bbs(bbsIn.size());

    // Convert rois to area and tl + br corner (preserve matlab readability)
    size_t n = bbs.size();
    struct Roi
    {
        int as, xs, xe, ys, ye, kp;
    };
    std::vector<Roi> coords(n);
    for(int i = 0; i < n; i++)
    {
        bbs[i] = bbsIn[ord[i]];

        //std::cout << bbs[i].score << std::endl;

        // Convenient storage of tl, br coordinates (per matlab code)
        coords[i].kp = 1;
        coords[i].as = bbs[i].roi.size().area();
        coords[i].xs = bbs[i].roi.x;
        coords[i].ys = bbs[i].roi.y;
        coords[i].xe = bbs[i].roi.br().x;
        coords[i].ye = bbs[i].roi.br().y;
    }

    for(int i = 0; i < n; i++)
    {
        if( greedy && !coords[i].kp )
        {
            continue;
        }

        for(int j = i+1; j < n; j++)
        {
            if(coords[j].kp == 0)
            {
                continue;
            }

            int iw = std::min(coords[i].xe, coords[j].xe) - std::max(coords[i].xs, coords[j].xs);
            if(iw <= 0)
            {
                continue;
            }

            int ih = std::min(coords[i].ye, coords[j].ye) - std::max(coords[i].ys, coords[j].ys);
            if(ih <= 0)
            {
                continue;
            }

            double o = (iw * ih) , u = (ovrDnm) ? (coords[i].as + coords[j].as - o) : std::min(coords[i].as, coords[j].as);
            o /= u;

            if(o > overlap)
            {
                coords[j].kp = 0;

                //std::cout << j << " " << o << std::endl;
            }
        }
    }

    // Delete the boxes with kp[i] < 0
    auto pkp = coords.begin();
    auto pbb = bbs.begin();
    while(pkp != coords.end())
    {
        pbb = (pkp++)->kp ? (pbb+1) : bbs.erase( pbb );
    }

    return bbs;
}

static void nms1( const std::vector<Detection> &bbsIn, std::vector<Detection> &bbs, const Detector::Options::Nms &pNms, double ovrDnm )
{
    // TODO: The original code splits large vectors in two, runs nms on each half, then runs nms again on result
    // We don't bother with that here:

    switch( string_hash::hash( (*pNms.type) ) )
    {
        case "max"_hash   :
        {
            bbs = nmsMax(bbsIn, pNms.overlap, 0, ovrDnm);
        }
        break;
        case "maxg"_hash  :
        {
            bbs = nmsMax(bbsIn, pNms.overlap, 1, ovrDnm);
        }
        break;
        case "ms"_hash    :
        {
            bbs = nmsMs(bbsIn, pNms.thr, pNms.radii);
        }
        break;
        case "cover"_hash :
        {
            bbs = nmsCover(bbsIn, pNms.overlap, ovrDnm);
        }
        break;
        default:
            CV_Assert(false);
            break;
    }
}

#define ACF_INFINITY std::numeric_limits<double>::max()

int Detector::bbNms(const std::vector<Detection> &bbsIn, const Options::Nms &pNmsI, std::vector<Detection> &bbs)
{
    Detector::Options::Nms dflt;
    dflt.type = {"type", std::string("max")};
    dflt.maxn = {"maxn", std::numeric_limits<double>::infinity()};
    dflt.radii = {"radii", std::vector<double>{0.15,0.15,1.0,1.0}};
    dflt.overlap = {"overlap", 0.5};
    dflt.ovrDnm = {"ovrDnm", std::string("union")};
    dflt.separate = {"separate", 0}; // NOTE: we are currently dealing with single class detections, so we don't need this

    Options::Nms pNms = pNmsI;
    pNms.merge( dflt, 1 );

    double thr = (pNms.thr.has) ? (*pNms.thr) : ((pNms.type.has && !pNms.type->compare("ms")) ? 0.0 : -ACF_INFINITY);

    int ovrDnm = 0; // std::cout << pNms.ovrDnm << " in " << pNmsI.ovrDnm << std::endl;
    switch( string_hash::hash( (*pNms.ovrDnm)) )
    {
        case "union"_hash :
            ovrDnm = 1;
            break;
        case "min"_hash   :
            ovrDnm = 0;
            break;
        default :
            CV_Assert(false);
    }

    CV_Assert( (*pNms.maxn) >= 2 );

    // (((((((((((( TODO ))))))))))))
    //   Original code looks like this, but why woul.d we proceed of bbs are set to zero?
    //   discard bbs below threshold and run nms1:
    //   if(isempty(bbs)), bbs=zeros(0,5); end;
    // (((((((((((( TODO ))))))))))))

    bbs = bbsIn;
    if(bbs.size() == 0)
    {
        return 0;    // For now we'll just return (TODO: check original code)
    }

    if(!pNms.type->compare("none"))
    {
        return 0;
    }

    // Find all bb with score > thr
    bbs.erase( std::remove_if(bbs.begin(), bbs.end(), [=](const Detection &bb)
    {
        return bb.score < thr;
    }), bbs.end() );
    if(bbs.size() == 0)
    {
        return 0;
    }

    // TODO: Resize delegate?
    // if( pNms.resize.has ) CV_Assert( false );

    Options::Nms pNms1;
    pNms1.type = { "type", pNms.type };
    pNms1.thr = { "thr", thr };
    pNms1.maxn = { "maxn", pNms.maxn };
    pNms1.radii = { "radii", pNms.radii };
    pNms1.overlap = { "overlap", pNms.overlap };

    // The original code was running NMS on union of all classifiers, but we are dealing with object detectors
    // independently:

    auto bbs1 = bbs;
    bbs.clear();
    nms1(bbs1, bbs, pNms1, ovrDnm); // NOTE: double(ovrDnm) is used here (not string)

    return 0;
}

DRISHTI_ACF_NAMESPACE_END

