/**
    This file is part of Deformable Shape Tracking (DEST).

    Copyright(C) 2015/2016 Christoph Heindl
    All rights reserved.

    This software may be modified and distributed under the terms
    of the BSD license.See the LICENSE file for details.
*/

#ifndef DEST_CONVERT_H
#define DEST_CONVERT_H

#include <dest/core/config.h>
//#if !defined(DEST_WITH_OPENCV)
//#error OpenCV is required for this part of DEST.
//#endif

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace dest {
    namespace util {
        
        /**
            Convert OpenCV image to DEST reusing memory.
         */
        inline core::MappedImage toDestHeaderOnly(const cv::Mat &src) {
            eigen_assert(src.channels() == 1);
            eigen_assert(src.type() == CV_8UC1);
            
            const int outerStride = static_cast<int>(src.step[0] / sizeof(unsigned char));
            
            return core::MappedImage(src.ptr<unsigned char>(), src.rows, src.cols, Eigen::OuterStride<Eigen::Dynamic>(outerStride));
        }

        /**
            Convert OpenCV image to DEST.
        */
        inline void toDest(const cv::Mat &src, core::Image &dst) {
            dst.resize(src.rows, src.cols);

            cv::Mat singleChannel;
            if (src.channels() == 3) {
                cv::cvtColor(src, singleChannel, CV_BGR2GRAY);
            } else {
                singleChannel = src;
            }
            
            core::MappedImage map = toDestHeaderOnly(singleChannel);
            dst = map;
        }

        /**
            Convert DEST image to OpenCV header.
        */
        inline void toCVHeaderOnly(const core::Image &src, cv::Mat &dst) {

            const int rows = static_cast<int>(src.rows());
            const int cols = static_cast<int>(src.cols());

            dst = cv::Mat(rows, cols, CV_8UC1, const_cast<unsigned char*>(src.data()));
        }

        /**
            Convert DEST image to OpenCV.
        */
        inline void toCV(const core::Image &src, cv::Mat &dst) {

            cv::Mat hdr;
            toCVHeaderOnly(src, hdr);
            hdr.copyTo(dst);
        }
        
        /**
            Convert OpenCV rectangle to DEST.
        */
        inline void toDest(const cv::Rect &src, core::Rect &dst) {
            dst = core::createRectangle(Eigen::Vector2f(src.tl().x, src.tl().y), Eigen::Vector2f(src.br().x, src.br().y));
        }
        
        /**
            Convert DEST rectangle to OpenCV.
        */
        inline void toCV(const core::Rect &src, cv::Rect_<float> &dst) {
            dst.x = src(0,0);
            dst.y = src(1,0);
            dst.width = src(0,3) - src(0,0);
            dst.height = src(1,3) - src(1,0);
        }

    }
}

#endif
