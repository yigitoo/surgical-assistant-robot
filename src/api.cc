#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>


#include <chrono>
#include <thread>

#ifdef __linux__

#include <unistd.h>

#elif _WIN32

#include <Windows.h>

#endif

#include <zmq.hpp>

/**
* @brief This file is using for handling ros requests and responses.
**/
int connect()
{
    using namespace std::chrono_literals;

    // initialize the zmq context with a single IO thread
    zmq::context_t context{1};

    // construct a REP (reply) socket and bind to interface
    zmq::socket_t socket{context, zmq::socket_type::rep};
    socket.bind("tcp://*:5555");

    // prepare some static data for responses
    const std::string data{"World"};

    for (;;)
    {
        zmq::message_t request;

        // receive a request from client
        socket.recv(request, zmq::recv_flags::none);
        std::cout << "Received " << request.to_string() << std::endl;

        // simulate work
        std::this_thread::sleep_for(1s);

        // send the reply to the client
        socket.send(zmq::buffer(data), zmq::send_flags::none);
    }

    return 0;
}

void sleep(int duration) {
#ifdef __linux__
    usleep(duration * 1000);
#elif _WIN32
    Sleep(duration);
#endif

}
