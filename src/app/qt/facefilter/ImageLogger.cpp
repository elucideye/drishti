/*! -*-c++-*-
  @file   ImageLogger.cpp
  @author David Hirvonen
  @brief  Implementation of a network image logger.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "ImageLogger.h"

#include "drishti/core/make_unique.h"
#include "drishti/core/drishti_stdlib_string.h"

#include <beast/websocket.hpp>
#include <beast/core.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <chrono>

DRISHTI_CORE_NAMESPACE_BEGIN

struct ImageLogger::Impl
{
public:
    using Clock = std::chrono::high_resolution_clock;
    using Duration = Clock::duration;
    using TimePoint = Clock::time_point;

    Impl(const std::string& host, const std::string& port)
        : host(host)
        , port(port)
    {
    }

    ~Impl() = default;

    float getElapsed(const TimePoint& now) const
    {
        return std::chrono::duration_cast<std::chrono::duration<float>>(now - timeOfLastImage).count();
    }

    bool isTooSoon(const TimePoint& now)
    {
        const float fps = 1.f / (getElapsed(Clock::now()) + 1e-6f);
        if (fps < maxFramesPerSecond)
        {
            timeOfLastImage = now;
            return false;
        }
        else
        {
            return true;
        }
    }

    std::string host;
    std::string port;
    float maxFramesPerSecond = std::numeric_limits<float>::max();
    TimePoint timeOfLastImage;
};

ImageLogger::ImageLogger(const std::string& host, const std::string& port)
{
    impl = drishti::core::make_unique<Impl>(host, port);
}

ImageLogger::~ImageLogger() = default;

void ImageLogger::setMaxFramesPerSecond(float value)
{
    impl->maxFramesPerSecond = value;
}

float ImageLogger::getMaxFramesPerSecond() const
{
    return impl->maxFramesPerSecond;
}

static std::vector<std::uint8_t> create(const cv::Mat& image)
{
    std::vector<uint8_t> buffer;
    cv::imencode(".png", image, buffer);
    return buffer;
}

void ImageLogger::operator()(const cv::Mat& image)
{
    if (impl->isTooSoon(Impl::Clock::now()))
    {
        return;
    }

    // Normal boost::asio setup
    boost::asio::io_service ios;
    boost::asio::ip::tcp::resolver r{ ios };
    boost::asio::ip::tcp::socket sock{ ios };
    boost::asio::connect(sock, r.resolve(boost::asio::ip::tcp::resolver::query{ impl->host, impl->port }));

    // WebSocket connect and send message using beast
    beast::websocket::stream<boost::asio::ip::tcp::socket&> ws{ sock };
    ws.handshake(impl->host, "/");
    ws.set_option(beast::websocket::message_type{ beast::websocket::opcode::binary });
    ws.write(boost::asio::buffer(create(image)));

    // Receive WebSocket message, print and close using beast
    beast::streambuf sb;
    beast::websocket::opcode op;
    ws.read(op, sb);
    ws.close(beast::websocket::close_code::normal);

    std::cout << beast::to_string(sb.data()) << "\n";
}

const std::string& ImageLogger::port() const
{
    return impl->port;
}

const std::string& ImageLogger::host() const
{
    return impl->host;
}

DRISHTI_CORE_NAMESPACE_END
