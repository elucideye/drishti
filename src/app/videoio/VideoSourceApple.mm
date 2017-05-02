#include "VideoSourceApple.h"

#import "MIMovieVideoSampleAccessor.h"
#import "MICMSampleBuffer.h"

#import <CoreVideo/CVPixelBuffer.h>

#include "drishti/core/make_unique.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class VideoSourceApple::Impl
{
public:
    Impl(const std::string &filename) : filename(filename)
    {

    }

    ~Impl()
    {
        
    }

    void init()
    {
        NSString *path = [NSString stringWithUTF8String:filename.c_str()];
        NSURL *url = [NSURL fileURLWithPath:path]; // @"/Users/dhirvonen/Downloads/IMG_9345.MOV"];
    
        NSError *err;
        if ([url checkResourceIsReachableAndReturnError:&err] == NO)
        {
            NSLog(@"Not reachable!");
        }
    
        AVURLAsset *videoAsset = [AVURLAsset assetWithURL:url];
        sampleAccessor = [[MIMovieVideoSampleAccessor alloc] initWithMovie:videoAsset firstSampleTime:kCMTimeZero];
    }

    cv::Mat operator()(int i)
    {
        cv::Mat image; // BGR
        
        if(MICMSampleBuffer *sample = [sampleAccessor nextSampleBuffer])
        {
            CMSampleBufferRef buffer = [sample CMSampleBuffer];
            CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(buffer);
            
            // Begin processing:
            CVPixelBufferLockBaseAddress( pixelBuffer, 0 );
            
            //int format = CVPixelBufferGetPixelFormatType(pixelBuffer);
            
            int cols = CVPixelBufferGetWidth(pixelBuffer);
            int rows = CVPixelBufferGetHeight(pixelBuffer);
            unsigned char *pixels = (unsigned char *)CVPixelBufferGetBaseAddress(pixelBuffer);
            cv::Mat argb(rows, cols, CV_8UC4, pixels), bgra(rows, cols, CV_8UC4), bgr;
            cv::mixChannels(argb, bgra, {0,3, 1,2, 2,1, 3,0 });
            cv::cvtColor(bgra, image, cv::COLOR_BGRA2BGR);
            
            //End processing
            CVPixelBufferUnlockBaseAddress( pixelBuffer, 0 );
        }

        return m_frame = image;
    }

    bool good() const { return !m_frame.empty(); }
    
    cv::Mat m_frame; // assume sequential access

    MIMovieVideoSampleAccessor *sampleAccessor;
    std::string filename;
};

VideoSourceApple::VideoSourceApple(const std::string &filename)
{
    m_impl = drishti::core::make_unique<Impl>(filename);
    m_impl->init();
}

VideoSourceApple::~VideoSourceApple()
{
    
}

VideoSourceCV::Frame VideoSourceApple::operator()(int i)
{
    return Frame((*m_impl)(i), i);
}

bool VideoSourceApple::good() const
{
    return m_impl->good();
}


std::size_t VideoSourceApple::count() const
{
    // TODO?
    return static_cast<std::size_t>(std::numeric_limits<int>::max());
}
