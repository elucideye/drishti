/*
  MIMovieVideoSampleAccessor.h
  MovieMaker

  Copyright (c) 2015 Zukini Ltd. All rights reserved.

  https://github.com/SheffieldKevin/VideoSample

 ------------------------------------------------------------------------------
 
 The MIT License (MIT)
 
 Copyright (c) 2015 Zukini Ltd.
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

//@import Foundation;
//@import AVFoundation;

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

@class MICMSampleBuffer;

/**
 @brief A class for accessing frames from 1 or more video tracks of a movie.
 @discussion This class allows for random access of video frames but performance
 is much better if samples are requested in time order and frame times are not
 too far apart. You can also iterate through each sample using the next sample.
*/
@interface MIMovieVideoSampleAccessor : NSObject

// The last requested buffer.
@property (readonly) MICMSampleBuffer* currentBuffer;

// The time of the last requested buffer.
@property (readonly) CMTime currentTime;

// Can no longer use this sample accessor.
@property (readonly) BOOL isBroken;

/**
 @brief Designated initializer. Can return nil.
 @param movie The movie asset from which to obtain the video samples
 @param firstSampleTime The time from which to get the first sample.
 @param tracks An array of AVAssetTracks. If nil, then defaults to all video tracks
 @param videoSettings The settings used to create the CVPixelBuffer from sample.
        If nil, settings appropriate for creating a CGImageRef via a CGContext
        will be used.
 @param videoComposition The video composition to be used. If nil then the 
        composition will use the movie asset composition.
*/
- (instancetype)initWithMovie:(AVURLAsset*)movie
              firstSampleTime:(CMTime)firstTime
                       tracks:(NSArray*)tracks
                videoSettings:(NSDictionary*)videoSettings
             videoComposition:(AVVideoComposition*)composition;

/**
 @brief Designated initializer. Can return nil.
 @param movie The movie asset from which to obtain the video samples
 @param firstSampleTime The time from which to get the first sample.
 @param tracks An array of AVAssetTracks. If nil, then defaults to all video tracks
 @param videoSettings The settings used to create the CVPixelBuffer from sample.
 If nil, settings appropriate for creating a CGImageRef via a CGContext
 will be used.
 @param videoComposition The video composition to be used. If nil then the
 composition will use the movie asset composition.
 */
- (instancetype)initWithMovie:(AVURLAsset*)movie
              firstSampleTime:(CMTime)firstTime;

/// Get the sample buffer at the specified time.
- (MICMSampleBuffer*)sampleBufferAtTime:(CMTime)time;

/// Get the next sample buffer.
- (MICMSampleBuffer*)nextSampleBuffer;

/// Is the array of tracks the same as the sample accessor's list of tracks.
- (BOOL)equalTracks:(NSArray*)tracks;

@end
