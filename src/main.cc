#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include <zmq.hpp>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>


#ifdef __linux__
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#endif

#include "./api.hpp"
#include <ctime>

int communication_bridge(int result);
void sleep(int duration);
std::vector<std::string> splitstr(std::string str, char seperator);
int timestamp();
int main(int argc, char *argv[]);

int main(int argc, char *argv[])
{
    InitializeAPIFunctions();
    int result = (*MyInitAPI)();
    int programResult = communication_bridge(result);
    return programResult;
}

int communication_bridge(int result) {
    // Create a ZeroMQ context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REP);

    // Bind the socket to a TCP address (we use localhost and port 5555)
    socket.bind("tcp://*:5555"); // tcp://*:5555/ would be accepted
                                         // as well

    
    std::cout << "ZeroMQ reply server is running..." << std::endl;

    while (true) {
        // Wait for a request from a client
        zmq::message_t request;
        socket.recv(&request);
        // Process the request (in this example, we simply echo back the message)
        std::string receivedMessage(static_cast<char*>(request.data()), request.size());
        std::cout << "Received request: " << receivedMessage << std::endl;

        std::vector<std::string> request_payload = splitstr(receivedMessage,';');


        if (request_payload[0] == "get_motor_by_id" && stoi(request_payload[1]) <= 6) {
            int motor_id = stoi(request_payload[1]);

            // Send the reply back to the client
            std::string replyMessage = "{\"reply\": \"motor_id " + std::to_string(motor_id) + "\"}";
            zmq::message_t reply(replyMessage.size());
            memcpy(reply.data(), replyMessage.data(), replyMessage.size());
            socket.send(reply, zmq::send_flags::none);
            std::cout << "Sent reply: " << replyMessage << std::endl;
        } else if (request_payload[0] == "get_current" && stoi(request_payload[1]) <= 6)
        {

        } else if (request_payload[0] == "get_temperature" && stoi(request_payload[1]) <= 6)
        {

        } else if (request_payload[0] == "get_angular_info" && stoi(request_payload[1]) <= 6)
        {
            
        } else if (request_payload[0] == "get_cartesian_info" && stoi(request_payload[1]) <= 6) 
        {

        } else if (request_payload[0] == "get_torque_value" && stoi(request_payload[1]) <= 6) 
        {

        } else if(request_payload[0] == "get_temperatures") 
        {

        } else if(request_payload[0] == "get_angular_infos") 
        {

        } else if(request_payload[0] == "get_cartesian_infos") 
        {

        } else if(request_payload[0] == "get_torque_values") 
        {

        } else if(request_payload[0] == "get_torque_values") 
        {

        } else if(request_payload[0] == "get_torque_values") 
        {

        }  else {
            std::cout << "Error: Requst is not sending clearly" << std::endl;
            std::cout << "Request payload: " << receivedMessage << std::endl;
        }
    }

    // The server should never reach this point, but for completeness, close the socket and context
    socket.close();
    context.close();

    return EXIT_SUCCESS;
}


int timestamp() {
    return std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch())
        .count();
}
void sleep(int duration) {
#ifdef __linux__
    usleep(duration * 1000);
#elif _WIN32
    Sleep(duration);
#endif
}

// Create custom split() function.  
std::vector<std::string> splitstr(std::string str, char separator) {
    std::vector<std::string> vString;
    int startIndex = 0, endIndex = 0;
    for (int i = 0; i <= str.size(); i++) {
        
        // If we reached the end of the word or the end of the input.
        if (str[i] == separator || i == str.size()) {
            endIndex = i;
            std::string temp;
            temp.append(str, startIndex, endIndex - startIndex);
            vString.push_back(temp);
            startIndex = endIndex + 1;
        }
    }

    return vString;
}