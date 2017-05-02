#include "VideoSourceApple.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char **argv)
{
    CV_Assert(argc > 1);
    VideoSourceApple video(argv[1]);
    
    for(cv::Mat frame = video(); !frame.empty(); frame = video())
    {
        int i = 0;
        std::stringstream ss;
        ss << "/tmp/foo" << i++ << ".png";
        cv::imwrite(ss.str(), frame);
    }    
}
