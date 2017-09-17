/*! -*-c++-*-
  @file   glfwimshow.cpp
  @author David Hirvonen
  @brief  Google test for glfwimshow.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "imshow/imshow.h"

#include <opencv2/imgproc.hpp>

#include <limits>

static void draw(cv::Mat& image, const std::string& name, const cv::Scalar& color = { 0, 255, 0 })
{
    cv::putText(image, name, { image.cols / 4, image.rows / 2 }, CV_FONT_HERSHEY_PLAIN, 2.0, color, 1, 8, false);
}

std::pair<cv::Mat, std::string> create(const cv::Size& size, int type, const std::string& name)
{
    double maxVal = 255.f;
    switch (type)
    {
        case CV_8U:
            maxVal = std::numeric_limits<std::uint8_t>::max();
            break;
        case CV_8S:
            maxVal = std::numeric_limits<std::int8_t>::max();
            break;
        case CV_16U:
            maxVal = std::numeric_limits<std::uint16_t>::max();
            break;
        case CV_16S:
            maxVal = std::numeric_limits<std::int16_t>::max();
            break;
        case CV_32S:
            maxVal = std::numeric_limits<std::int32_t>::max();
            break;
        case CV_32F:
            maxVal = 1.0f;
            break;
        case CV_64F:
            maxVal = 1.0;
            break;
        default:
            break;
    }

    cv::Mat image(size, type, cv::Scalar(0, 0, 0, maxVal)); // alpha always 1
    draw(image, name, { maxVal, maxVal, maxVal });
    return std::make_pair(image, name);
}

int gauze_main(int argc, char** argv)
{
    const cv::Size size(640, 480);

    auto image32fc1 = create(size, CV_32FC1, "CV_32FC1");
    auto image32fc3 = create(size, CV_32FC3, "CV_32FC3");
    auto image32fc4 = create(size, CV_32FC4, "CV_32FC4");

    auto image8uc1 = create(size, CV_8UC1, "CV_8UC1");
    auto image8uc3 = create(size, CV_8UC3, "CV_8UC3");
    auto image8uc4 = create(size, CV_8UC4, "CV_8UC4");

    std::vector<std::pair<cv::Mat, std::string>> images{
        // CV_32F*
        image32fc1,
        image32fc3,
        image32fc4,

        // CV_8UC*
        image8uc1,
        image8uc3,
        image8uc4,
    };

    for (const auto& i : images)
    {
        //  Make sure both glfwtest and named image windows are updated:
        glfw::imshow("glfwtest", i.first);
        glfw::imshow(i.second.c_str(), i.first);
        glfw::waitKey(0);
    }

    glfw::destroyWindow("glfwtest");
    glfw::destroyAllWindows();

    return 0;
}

int main(int argc, char** argv)
{
    gauze_main(argc, argv);
}
