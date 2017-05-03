// Copyright (c) 2013-2017 Vinnie Falco (vinnie dot falco at gmail dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "drishti/core/drishti_stdlib_string.h" // must be first!!!
#include "websocket_async_server.hpp"
#include <boost/asio/io_service.hpp>
#include <boost/asio/signal_set.hpp>

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

int drishti_main(int argc, char** argv)
{
    const int argumentCount = argc;

    std::string sOutput;
    std::string sAddress = "127.0.0.1";
    std::uint16_t port = 6000;

    // clang-format off
    cxxopts::Options options("test-image-server", "Minimal image logging websocket server (beast)");
    options.add_options()
        ("a,address", "Address", cxxopts::value<std::string>(sAddress))
        ("p,port", "Port", cxxopts::value<std::uint16_t>(port))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
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

    int counter = 0;
    websocket::async_server::streambuf_handler handler = [&](beast::streambuf& db) {
        std::stringstream ss;
        ss << sOutput << "/frame_" << std::setw(4) << std::setfill('0') << counter++ << ".png";

        std::ofstream os(ss.str(), std::ios::binary);
        if (os)
        {
            for (const auto& b : db.data())
            {
                std::size_t s2 = boost::asio::buffer_size(b);
                const char* p2 = boost::asio::buffer_cast<const char*>(b);
                os.write(p2, s2);
            }
        }
        return 0;
    };
    s1.add(handler);

    s1.open(endpoint_type{address_type::from_string(sAddress), port}, ec);

    sig_wait();
}

int main(int argc, char** argv)
{
    try
    {
        return drishti_main(argc, argv);
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
