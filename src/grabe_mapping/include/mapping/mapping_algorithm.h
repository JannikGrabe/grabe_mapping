#include <string>

class MappingAlgorithm {
    std::string name;
    std::string argument;

public:
    MappingAlgorithm(std::string name = "placeholder_name", std::string argument = "placeholder_argument" ) : name(name), argument(argument) {}
};