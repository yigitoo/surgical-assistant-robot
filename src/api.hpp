#ifndef __API_H__
#define __API_H__

#include <string>
#include <vector>
#include <map>

std::map<std::string, std::string> parse_file(const std::string filepath);
std::map<std::string, std::string> parse_requests();
std::vector<float> parse_servo_angles();

#endif // __API_H__