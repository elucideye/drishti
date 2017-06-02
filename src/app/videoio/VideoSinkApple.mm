/*!
  @file   videoio/VideoSinkApple.mm
  @author David Hirvonen (C++ adaptation)
  @brief  AVFoundation movie writer implementation.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This is an adaption of original source by Brad Larson from GPUImage:
  * https://github.com/BradLarson/GPUImage/blob/master/License.txt
  * https://github.com/BradLarson/GPUImage/blob/master/framework/Source/Mac/GPUImageMovieWriter.{h,m}

*/

#include "videoio/VideoSinkApple.h"

#include "drishti/core/make_unique.h"
#include "drishti/core/scope_guard.h"

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

#include <iostream>

DRISHTI_VIDEOIO_NAMESPACE_BEGIN

struct VideoSinkApple::Impl
{
    Impl(const std::string &filename)
    {
        NSString *path = [NSString stringWithUTF8String:filename.c_str()];
        movieURL = [NSURL fileURLWithPath:path];
        fileType = AVFileTypeQuickTimeMovie;
    }
    
    ~Impl()
    {
        if(isStarted)
        {
        }
    }
    
    void setProperties(const VideoSinkCV::Properties &properties)
    {
        videoSize = CGSizeMake(properties.width, properties.height);
    }
    
    NSMutableDictionary * defaultSettings()
    {
        NSMutableDictionary *outputSettings = [[NSMutableDictionary alloc] init];
        [outputSettings setObject:AVVideoCodecH264 forKey:AVVideoCodecKey];
        [outputSettings setObject:[NSNumber numberWithInt:videoSize.width] forKey:AVVideoWidthKey];
        [outputSettings setObject:[NSNumber numberWithInt:videoSize.height] forKey:AVVideoHeightKey];
        return outputSettings;
    }
    
    bool good()
    {
        if(!assetWriter)
        {
            return false;
        }
        switch(assetWriter.status)
        {
            case AVAssetWriterStatusUnknown:
            case AVAssetWriterStatusWriting: return true;
            default: return false;
        }
    }

    bool begin();
    bool operator()(const cv::Mat &image);
    bool end(const VideoSinkCV::CompletionHandler &handler);
    bool isStarted = false;
    
    CGSize videoSize;
    NSURL *movieURL;
    NSString *fileType;
    AVAssetWriter *assetWriter;
    AVAssetWriterInput *assetWriterVideoInput;
    AVAssetWriterInputPixelBufferAdaptor *assetWriterPixelBufferInput;
};

bool VideoSinkApple::Impl::begin()
{
    NSError *error = nil;
    assetWriter = [[AVAssetWriter alloc] initWithURL:movieURL fileType:fileType error:&error];
    if (error != nil)
    {
        NSLog(@"Error: %@", error);
        return false;
    }
    
    // Set this to make sure that a functional movie is produced,
    // even if the recording is cut off mid-stream. Only the last
    // second should be lost in that case.
    assetWriter.movieFragmentInterval = CMTimeMakeWithSeconds(1.0, 1000);
    
    NSMutableDictionary *outputSettings = defaultSettings();
    assetWriterVideoInput = [AVAssetWriterInput assetWriterInputWithMediaType:AVMediaTypeVideo outputSettings:outputSettings];
    assetWriterVideoInput.expectsMediaDataInRealTime = TRUE; // Revisit
    
    // You need to use BGRA for the video in order to get realtime encoding.
    // Use a color-swizzling shader to line up glReadPixels' normal RGBA output
    // with the movie input's BGRA.
    NSDictionary *sourcePixelBufferAttributesDictionary =
    [NSDictionary dictionaryWithObjectsAndKeys:
     [NSNumber numberWithInt:kCVPixelFormatType_32BGRA], kCVPixelBufferPixelFormatTypeKey,
     [NSNumber numberWithInt:videoSize.width], kCVPixelBufferWidthKey,
     [NSNumber numberWithInt:videoSize.height], kCVPixelBufferHeightKey, nil];
    
    assetWriterPixelBufferInput =
    [AVAssetWriterInputPixelBufferAdaptor
     assetWriterInputPixelBufferAdaptorWithAssetWriterInput:assetWriterVideoInput
     sourcePixelBufferAttributes:sourcePixelBufferAttributesDictionary];
    
    [assetWriter addInput:assetWriterVideoInput];
    
    return (assetWriter.status == AVAssetWriterStatusUnknown);
}

bool VideoSinkApple::Impl::end(const VideoSinkCV::CompletionHandler &handler)
{
    if(isStarted)
    {
        std::cout << assetWriter.status << std::endl;
        [assetWriterVideoInput markAsFinished];
        [assetWriter finishWritingWithCompletionHandler:^{ handler(); }];
        isStarted = false;
    }
    return true;
}

bool VideoSinkApple::Impl::operator()(const cv::Mat &image)
{
    static int counter = 0;
    CMTime frameTime = CMTimeMakeWithSeconds(static_cast<double>(counter++)/24.0, 1000.0);
    
    if (!isStarted)
    {
        [assetWriter startWriting];
        [assetWriter startSessionAtSourceTime:frameTime];
        if (assetWriter.status != AVAssetWriterStatusWriting)
        {
            return false;
        }
        else
        {
            isStarted = true;
        }
    }
    
    CVPixelBufferPoolRef pixelBufferPool = [assetWriterPixelBufferInput pixelBufferPool];
    if (pixelBufferPool == NULL)
    {
        return false;
    }
    
    CVPixelBufferRef pixelBuffer = NULL;
    CVReturn status = CVPixelBufferPoolCreatePixelBuffer (NULL, pixelBufferPool, &pixelBuffer);
    if ((pixelBuffer == NULL) || (status != kCVReturnSuccess))
    {
        return false;
    }
    drishti::core::scope_guard guard = [&] { CVPixelBufferRelease(pixelBuffer); };
    
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    GLubyte *pixelBufferData = (GLubyte *)CVPixelBufferGetBaseAddress(pixelBuffer);
    memcpy(pixelBufferData, image.ptr<void*>(), (image.step[0] * image.rows));
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);

    if(![assetWriterPixelBufferInput appendPixelBuffer:pixelBuffer withPresentationTime:frameTime])
    {
        return false;
    }

    return true;
}

VideoSinkApple::VideoSinkApple(const std::string& filename, const std::string &hint)
{
    impl = drishti::core::make_unique<Impl>(filename);
}

VideoSinkApple::~VideoSinkApple()
{
    
}

bool VideoSinkApple::good()
{
    return impl && impl->good();
}

bool VideoSinkApple::begin()
{
    return impl && impl->begin();
}

bool VideoSinkApple::end(const CompletionHandler &handler)
{
    if(impl)
    {
        @try
        {
            return impl->end(handler);
        }
        @catch (NSException *exception)
        {
            NSLog(@"NSException caught in VideoSinkApple" );
            NSLog(@"Name: %@", exception.name);
            NSLog(@"Reason: %@", exception.reason);
        }
    }
    else
    {
        handler();
    }
}

bool VideoSinkApple::operator()(const cv::Mat &image)
{
    if (impl)
    {
        @try
        {
            return (*impl)(image);
        }
        @catch (NSException *exception)
        {
            NSLog(@"NSException caught in VideoSinkApple");
            NSLog(@"Name: %@", exception.name);
            NSLog(@"Reason: %@", exception.reason);
        }
    }
}

void VideoSinkApple::setProperties(const Properties &properties)
{
    return impl->setProperties(properties);
}

DRISHTI_VIDEOIO_NAMESPACE_END


