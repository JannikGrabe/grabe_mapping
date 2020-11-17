#include <map>
#include "parameter.h"
#include <sstream>

namespace grabe_mapping {

class Parameter_map {
private:
    std::map<std::string, Parameter<double>*> double_parameters;
    std::map<std::string, Parameter<int>*> int_parameters;
    std::map<std::string, Parameter<std::string>*> string_parameters;

    int type(std::string name) {
        if(this->double_parameters.find(name) != this->double_parameters.end()) {
            return 1;
        }
        else if(this->int_parameters.find(name) != this->int_parameters.end()) {
            return 2;
        }
        else if(this->string_parameters.find(name) != this->string_parameters.end()) {
            return 3;
        } else {
            return 0;
        }
    }

public:

    bool add_parameter(std::string name, double default_value, bool is_active = true) {
        if(this->type(name) != 0) {
            return false;
        }

        Parameter<double>* param = new Parameter<double>(default_value, is_active);
        
        this->double_parameters.insert(std::pair<std::string, Parameter<double>*>(name, param));

        return true;
    }

    bool add_parameter(std::string name, int default_value, bool is_active = true) {
        if(this->type(name) != 0) {
            return false;
        }

        Parameter<int>* param = new Parameter<int>(default_value, is_active);
        
        this->int_parameters.insert(std::pair<std::string, Parameter<int>*>(name, param));

        return true;
    }

    bool add_parameter(std::string name, std::string default_value, bool is_active = true) {
        if(this->type(name) != 0) {
            return false;
        }

        Parameter<std::string>* param = new Parameter<std::string>(default_value, is_active);
        
        this->string_parameters.insert(std::pair<std::string, Parameter<std::string>*>(name, param));

        return true;
    }

    bool add_parameter(std::string name, bool is_active = true) {
        if(this->type(name) != 0) {
            return false;
        }

        Parameter<std::string>* param = new Parameter<std::string>("without_value", is_active, false);
        
        this->string_parameters.insert(std::pair<std::string, Parameter<std::string>*>(name, param));

        return true;
    }

    bool set_value(std::string name, double value) {
        if(this->type(name) != 1) {
            return false;
        }

        this->double_parameters[name]->set_value(value);

        std::cout << name << " " << value << std::endl;
        return true;
    }

    bool set_value(std::string name, int value) {
        if(this->type(name) != 2) {
            return false;
        }
        this->int_parameters[name]->set_value(value);

        std::cout << name << " " << value << std::endl;
        return true;
    }

    bool set_value(std::string name, std::string value) {
        if(this->type(name) != 3) {
            return false;
        }

        Parameter<std::string>* param = this->string_parameters[name];

        if(param->get_has_value()) {
            param->set_value(value);
            std::cout << name << " " << value << std::endl;
            return true;
        } else {
            return false;
        }
    }

    bool set_active(std::string name, bool state) {
        int type = this->type(name);

        switch(type) {
            case 0:
                return false;
            case 1:
                this->double_parameters[name]->set_active(state);
                break;
            case 2:
                this->int_parameters[name]->set_active(state);
                break;
            case 3:
                this->string_parameters[name]->set_active(state);
                break;
        }

        return true;
    }

    bool toggle_active(std::string name) {
        int type = this->type(name);

        switch(type) {
            case 0:
                return false;
            case 1:
                this->double_parameters[name]->toggle_active();
                break;
            case 2:
                this->int_parameters[name]->toggle_active();
                break;
            case 3:
                this->string_parameters[name]->toggle_active();
                break;
        }

        return true;
    }

    bool get_value(std::string name, double& value) {
        if(this->type(name) != 1) {
            return false;
        }

        value = this->double_parameters[name]->get_value();
        return true;
    }

    bool get_value(std::string name, int& value) {
        if(this->type(name) != 1) {
            return false;
        }

        value = this->int_parameters[name]->get_value();
        return true;
    }

    bool get_value(std::string name, std::string& value) {
        if(this->type(name) != 1) {
            return false;
        }

        Parameter<std::string>* param = this->string_parameters[name];
        if(param->get_has_value()) {
            value = param->get_value();
            return true;
        }
        return false;
    }

    bool get_is_active(std::string name, bool& is_active) {
        int type = this->type(name);

        switch(type) {
            case 0:
                return false;
            case 1:
                is_active = this->double_parameters[name]->get_is_active();
                break;
            case 2:
                is_active = this->int_parameters[name]->get_is_active();
                break;
            case 3:
                is_active = this->string_parameters[name]->get_is_active();
                break;
        }

        return true;
    }

    std::string to_string() {
        std::ostringstream out("", std::ios_base::app);

        for(std::map<std::string, Parameter<double>*>::iterator it = this->double_parameters.begin(); 
            it != this->double_parameters.end(); 
            it++) {
            
            if(it->second->get_is_active()) {
                out << " " << it->first << it->second->to_string();
            }
        }

        for(std::map<std::string, Parameter<int>*>::iterator it = this->int_parameters.begin(); 
            it != this->int_parameters.end(); 
            it++) {
            
            if(it->second->get_is_active()) {
                out << " " << it->first << it->second->to_string();
            }
        }

        for(std::map<std::string, Parameter<std::string>*>::iterator it = this->string_parameters.begin(); 
            it != this->string_parameters.end(); 
            it++) {
            
            if(it->second->get_is_active()) {
                out << " " << it->first << it->second->to_string();
            }
        }

        return out.str();
    }
};

}