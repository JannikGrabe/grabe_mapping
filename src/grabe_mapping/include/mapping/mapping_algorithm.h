#include <string>
#include <sstream>
#include <map>

class MappingAlgorithm {
    std::string name;

    struct Parameter {
        double value; 
        bool active;

        Parameter(double value, bool active = true) : value(value), active(active) {

        }
    };

    std::map<std::string, Parameter> parameters;

public:
    MappingAlgorithm(std::string name = "placeholder_name") : 
        name(name)
    {

    }

    std::string get_name() const { return this->name; }
    
    std::string to_string() {
        std::ostringstream out("", std::ios_base::app);

        for(std::map<std::string, Parameter>::iterator it = this->parameters.begin(); it != this->parameters.end(); it++) {
            if(it->second.active)
                out << " " << it->first << " " << it->second.value;
        }

        return out.str();
    }

    bool set_parameter(std::string parameter, double value) {
        if(this->parameters.find(parameter) != this->parameters.end()) {
            this->parameters[parameter] = Parameter(value);
            return true;
        } else {
            return false;
        }
    }

    bool set_parameter_active(std::string parameter, bool active) {
        if(this->parameters.find(parameter) != this->parameters.end()) {
            this->parameters[parameter].active = active;
            return true;
        } else {
            return false;
        }
    }

    bool add_parameter(std::string parameter, double default_value = 0.0, bool active = true) {
        if(this->parameters.find(parameter) == this->parameters.end()) {
            this->parameters.insert(std::pair<std::string, Parameter>(parameter, Parameter(default_value, active)));
            return true;
        } else {
            return false;
        }
    }
};