#include <string>
#include <sstream>
#include <map>

class MappingAlgorithm {
    std::string name;

    std::map<std::string, double> parameters;

public:
    MappingAlgorithm(std::string name = "placeholder_name") : 
        name(name)
    {

    }

    std::string get_name() const { return this->name; }
    
    std::string to_string() {
        std::ostringstream out("", std::ios_base::app);

        for(std::map<std::string, double>::iterator it = this->parameters.begin(); it != this->parameters.end(); it++) {
            out << " " << it->first << " " << it->second;
        }

        return out.str();
    }

    bool set_parameter(std::string parameter, double value) {
        if(this->parameters.find(parameter) != this->parameters.end()) {
            this->parameters[parameter] = value;
            return true;
        } else {
            return false;
        }
    }

    bool add_parameter(std::string parameter, double default_value = 0.0) {
        if(this->parameters.find(parameter) == this->parameters.end()) {
            this->parameters.insert(std::pair<std::string, double>(parameter, default_value));
            return true;
        } else {
            return false;
        }
    }
};