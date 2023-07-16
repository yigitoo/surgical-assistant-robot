#ifndef __API_H__
#define __API_H__

#include <string>
#include <vector>
#include <map>

int main(int argc, char *argv[]);
int communication();
void sleep(int duration);
std::vector<std::string> splitstr(std::string str, char seperator);

#endif // __API_H__