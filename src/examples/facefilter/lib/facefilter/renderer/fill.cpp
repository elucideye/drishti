/*! -*-c++-*-
  @file fill.cpp
  @brief Implementations for GPUImage fill geometry computations for ogles_gpgpu

  \copyright Copyright 2014-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
*/

#include <facefilter/renderer/fill.h>

// In: 3x3
//  +---+
//  |***|
//  |***|
//  |***|
//  +---+
//
//
// Out: 9x15
//  +---------+
//  |ooooooooo|
//  |ooooooooo|
//  |ooooooooo|
//  |*********|
//  |*********|
//  |*********|
//  |*********|
//  |*********|
//  |*********|
//  |*********|
//  |*********|
//  |*********|
//  |ooooooooo|
//  |ooooooooo|
//  |ooooooooo|
//  +---------+
//
// originalAspectRatio = 3/3 = 1.0
//   -> aspect ratio of input texture
// maxAspectRatio = 9/15 = 0.6
//   -> aspect ratio of destination "display"
//
// If `originalAspectRatio >= maxAspectRatio` that means the input texture shape is wider
// (relatively) than the output image, and if we think of placing the input texture inside
// the output texture and expanding it while preserving aspect ratio, then the left and right sides
// will "touch" before the top and bottom sides and we end up with a cinema style letterbox look.
// Converserly, if `originalAspectRatio < maxAspectRatio`, then the top and bottom sides will
// touch first.

cv::Rect2f rectScaleAspectFit(const cv::Rect2f& rect, const cv::Rect2f& maxRect)
{
    const float originalAspectRatio = rect.width / rect.height;
    const float maxAspectRatio = maxRect.width / maxRect.height;

    cv::Rect2f insetRect(maxRect.x, maxRect.y, maxRect.width, maxRect.height);
    if (originalAspectRatio >= maxAspectRatio)
    { // scale by width
        insetRect.height = insetRect.width * rect.height / rect.width;
        insetRect.y += (maxRect.height - insetRect.height) / 2.0f;
    }
    else
    { // scale by height
        insetRect.width = insetRect.height * rect.width / rect.height;
        insetRect.x += (maxRect.width - insetRect.width) / 2.0f;
    }

    return insetRect;
}

template <typename Operator>
cv::Rect2f rectScaleAspect(const cv::Rect2f& rect, const cv::Rect2f& maxRect, const Operator& op)
{
    const float originalAspectRatio = rect.width / rect.height;
    const float maxAspectRatio = maxRect.width / maxRect.height;

    cv::Rect2f insetRect(maxRect.x, maxRect.y, maxRect.width, maxRect.height);
    if (op(originalAspectRatio, maxAspectRatio))
    { // scale by width
        insetRect.height = insetRect.width * rect.height / rect.width;
        insetRect.y += (maxRect.height - insetRect.height) / 2.0f;
    }
    else
    { // scale by height
        insetRect.width = insetRect.height * rect.width / rect.height;
        insetRect.x += (maxRect.width - insetRect.width) / 2.0f;
    }

    return insetRect;
}

template <typename Operator>
RectTransform transformScaleAspect(const cv::Rect2f& rect, const cv::Rect2f& maxRect, const Operator& op)
{
    const cv::Rect2f insetRect = rectScaleAspect(rect, maxRect, op);

    // clang-format off
    return RectTransform
    {
        insetRect.x,
        insetRect.y,
        insetRect.width / maxRect.width,
        insetRect.height / maxRect.height
    };
    // clang-format on
}

// https://github.com/BradLarson/GPUImage/blob/master/framework/Source/iOS/GPUImageView.m
// Compatibility with GPUImageFillMode
RectTransform
recalculateViewGeometry(const cv::Size2f& inputImageSize, const cv::Size2f& currentViewSize, FillMode fillMode)
{
    const cv::Rect2f inputImageRect({ 0.f, 0.f }, inputImageSize);
    const cv::Rect2f currentViewRect({ 0.f, 0.f }, currentViewSize);

    switch (fillMode)
    {
        case kFillModeStretch:
            return { 0.f, 0.f, 1.f, 1.f };
        case kFillModePreserveAspectRatio:
            return transformScaleAspect(inputImageRect, currentViewRect, std::greater<float>());
        case kFillModePreserveAspectRatioAndFill:
            return transformScaleAspect(inputImageRect, currentViewRect, std::less<float>());
    }
}

RectTransform
recalculateViewGeometry(const cv::Size& inputImageSize, const cv::Size& currentViewSize, FillMode fillMode)
{
    const cv::Size2f inputImageSizef = inputImageSize;
    const cv::Size2f currentViewSizef = currentViewSize;
    return recalculateViewGeometry(inputImageSizef, currentViewSizef, fillMode);
}
