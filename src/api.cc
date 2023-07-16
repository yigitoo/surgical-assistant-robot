#include <zmq.hpp>
#include <string>
#include <iostream>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#ifdef __linux__
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#endif

int main ()
{
    //  Prepare our context and socket
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REQ);

    std::cout << "Connecting to hello world server…" << std::endl;
    socket.connect ("tcp://localhost:5555");

    //  Do 10 requests, waiting each time for a response
    for (int request_nbr = 0; request_nbr != 10; request_nbr++) {

        zmq::message_t request (6);
        memcpy ((void *) request.data (), "reply", 5);
        std::cout << "Sending Hello " << request_nbr << "…" << std::endl;
        socket.send (request);

        //  Get the reply.
        zmq::message_t reply;
        socket.recv (&reply);
        std::cout << "Received World " << reply << std::endl;
    
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
