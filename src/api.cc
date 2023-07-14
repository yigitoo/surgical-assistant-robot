#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>

/**
* @brief This file is using for handling ros requests and responses.
**/
int parse_file(const std::string filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        std::cout << "Error opening file" << std::endl;
        return -1;
    }

    std::string line;
    int count = 0;
    while (getline(file, line))
    {
        count++;
    }
    file.close();
    return count;
}
std::map<std::string, std::string> parse_requests()
{

}

std::vector<float> parse_servo_response()
{
    std::vector<float> servo_angles;
    /** @TODO: Get Servo angles */
    return servo_angles;
}
