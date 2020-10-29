#include <string>

class MappingAlgorithm {
    std::string name;
    std::string argument;
    double argument_value;

public:
    MappingAlgorithm(std::string name = "placeholder_name", std::string argument = "placeholder_argument",
                        double argument_value = 0) : 
        name(name), argument(argument), argument_value(argument_value) 
    {

    }

    std::string get_name() const { return this->name; }
    
    std::string to_string() {
        return this->argument + " " + std::to_string(this->argument_value);
    }
};