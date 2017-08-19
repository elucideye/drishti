// Copyright (c) 2013-2017 Vinnie Falco (vinnie dot falco at gmail dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "drishti/core/drishti_stdlib_string.h" // must be first!!!
#include "websocket_async_server.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/signal_set.hpp>

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include <opencv2/highgui.hpp>
#  include "imshow/imshow.h"
#  include <iostream>
#endif // DRISHTI_USE_IMSHOW
// clang-format on

#include "cxxopts.hpp"

#include <iostream>
#include <iomanip>

/// Block until SIGINT or SIGTERM is received.
void sig_wait()
{
    boost::asio::io_service ios;
    boost::asio::signal_set signals(ios, SIGINT, SIGTERM);
    signals.async_wait([&](boost::system::error_code const&, int) {});
    ios.run();
}

int gauze_main(int argc, char** argv)
{
    const int argumentCount = argc;

    std::string sOutput;
    std::string sAddress = "127.0.0.1";
    std::uint16_t port = 6000;
    bool doWindow = false;

    // clang-format off
    cxxopts::Options options("test-image-server", "Minimal image logging websocket server (beast)");
    options.add_options()
        ("a,address", "Address", cxxopts::value<std::string>(sAddress))
        ("p,port", "Port", cxxopts::value<std::uint16_t>(port))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("w,window", "Display images in window", cxxopts::value<bool>(doWindow))
        ("h,help", "Print help message");
    // clang-format on

    options.parse(argc, argv);

    if ((argumentCount <= 1) || options.count("help"))
    {
        std::cout << options.help({ "" }) << std::endl;
        return 0;
    }

    if (sOutput.empty())
    {
        std::cerr << "Must specify a valid output directory" << std::endl;
        return -1;
    }

    using namespace beast::websocket;
    using endpoint_type = boost::asio::ip::tcp::endpoint;
    using address_type = boost::asio::ip::address;

    beast::error_code ec;

    permessage_deflate pmd;
    pmd.client_enable = true;
    pmd.server_enable = true;
    pmd.compLevel = 3;

    websocket::async_server s1{ &std::cout, 1 };
    s1.set_option(read_message_max{ 64 * 1024 * 1024 });
    s1.set_option(auto_fragment{ false });
    s1.set_option(pmd);

    boost::asio::io_service ios;

    int counter = 0;
    
    // clang-format off
    websocket::async_server::streambuf_handler handler = [&](beast::streambuf& db)
    {
        std::stringstream ss;
        ss << sOutput << "/frame_" << std::setw(4) << std::setfill('0') << counter++ << ".png";

        std::ofstream os(ss.str(), std::ios::binary);
        if (os)
        {
            std::size_t count = 0;
            std::vector<char> buffer(db.size());
            for (const auto& b : db.data())
            {
                std::size_t s2 = boost::asio::buffer_size(b);
                const char* p2 = boost::asio::buffer_cast<const char*>(b);

                memcpy(buffer.data() + count, p2, s2);
                count += s2;
            }
            os.write(buffer.data(), buffer.size()); // write buffer asyncrhonously

// push buffer to single threaded display queue
#if defined(DRISHTI_USE_IMSHOW)
            cv::Mat image = cv::imdecode(buffer, CV_LOAD_IMAGE_COLOR);
            ios.post([=]() {
                glfw::imshow("image_server", image);
                glfw::waitKey(1);
            });
#endif
        }
        // clang-format on

        return 0;
    };

    s1.add(handler);
    s1.open(endpoint_type{ address_type::from_string(sAddress), port }, ec);

    boost::asio::signal_set signals(ios, SIGINT, SIGTERM);
    signals.async_wait([&](boost::system::error_code const&, int) {});
    ios.run();

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
