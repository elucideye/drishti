//
// Copyright (c) 2013-2017 Vinnie Falco (vinnie dot falco at gmail dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "drishti/core/drishti_stdlib_string.h" // must be first!!!
#include <beast/core/to_string.hpp>
#include <beast/websocket.hpp>
#include <boost/asio.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "cxxopts.hpp"

#include <iostream>
#include <string>

int gauze_main(int argc, char** argv)
{
    const int argumentCount = argc;

    std::string sAddress = "127.0.0.1";
    std::string sPort = "6000";
    std::string sImage;

    // clang-format off
    cxxopts::Options options("test-image-server", "Minimal image logging websocket server (beast)");
    options.add_options()
        ("a,address", "Address", cxxopts::value<std::string>(sAddress))
        ("p,port", "Port", cxxopts::value<std::string>(sPort))
        ("i,image", "Image", cxxopts::value<std::string>(sImage))
        ("h,help", "Print help message");
    // clang-format on

    options.parse(argc, argv);

    if ((argumentCount <= 1) || options.count("help"))
    {
        std::cout << options.help({ "" }) << std::endl;
        return 0;
    }

    // Create or load a test image to send out:
    cv::Mat image;
    if (sImage.empty())
    {
        // Create a blank test image
        image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 255, 0));
    }
    else
    {
        image = cv::imread(sImage);
        if (image.empty())
        {
            std::cerr << "Unable to open the specified image : " << sImage;
            return -1;
        }
    }

    // Normal boost::asio setup
    boost::asio::io_service ios;
    boost::asio::ip::tcp::resolver r{ ios };
    boost::asio::ip::tcp::socket sock{ ios };

    auto result = r.resolve(boost::asio::ip::tcp::resolver::query{ sAddress, sPort });
    boost::asio::connect(sock, result);

    // WebSocket connect and send message using beast
    beast::websocket::stream<boost::asio::ip::tcp::socket&> ws{ sock };
    ws.handshake(sAddress, "/");

    {
        std::vector<uint8_t> buffer;
        cv::imencode(".png", image, buffer);
        ws.set_option(beast::websocket::message_type{ beast::websocket::opcode::binary });
        ws.write(boost::asio::buffer(buffer));
    }

    // Receive WebSocket message, print and close using beast
    beast::streambuf sb;
    beast::websocket::opcode op;
    ws.read(op, sb);
    ws.close(beast::websocket::close_code::normal);
    std::cout << beast::to_string(sb.data()) << "\n";

    return 0;
}

int main(int argc, char** argv)
{
    try
    {
        return gauze_main(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unknown exception";
    }

    return 0;
}
