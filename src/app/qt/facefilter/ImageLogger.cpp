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

DRISHTI_CORE_NAMESPACE_BEGIN

struct ImageLogger::Impl
{
public:
    Impl(const std::string &host, const std::string &port)
        : host(host)
        , port(port)
    {}

    ~Impl() {}
  
    std::string host;
    std::string port;   
};

static std::vector<std::uint8_t> create(const cv::Mat& image)
{
    std::vector<uint8_t> buffer;
    cv::imencode(".png", image, buffer);
    return buffer;
}

ImageLogger::ImageLogger(const std::string& host, const std::string& port)
{
    impl = drishti::core::make_unique<Impl>(host, port);
}

ImageLogger::~ImageLogger()
{
    
}

void ImageLogger::operator()(const cv::Mat& image)
{
    // Normal boost::asio setup
    boost::asio::io_service ios;
    boost::asio::ip::tcp::resolver r{ ios };
    boost::asio::ip::tcp::socket sock{ ios };
    boost::asio::connect(sock, r.resolve(boost::asio::ip::tcp::resolver::query{ impl->host, impl->port }));
    
    // connect error code?
    
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

const std::string &ImageLogger::port() const
{
    return impl->port;
}

const std::string &ImageLogger::host() const
{
    return impl->host;
}

DRISHTI_CORE_NAMESPACE_END
