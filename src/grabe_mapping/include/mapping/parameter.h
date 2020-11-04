#include <sstream>
#include <iostream>

namespace grabe_mapping {

template <typename T>

class Parameter {

private:
    T value;
    bool active;
    bool has_value;

public: 
    Parameter() {
        this->active = true;
        this->has_value = true;
    }

    Parameter(T value, bool is_active = true, bool has_value = true) 
        : value(value), active(is_active), has_value(has_value) 
    {

    }

    T get_value() {
        return this->value;
    }

    bool get_is_active() {
        return this->active;
    }

    bool get_has_value() {
        return this->has_value;
    }

    void set_value(T value) {
        this->value = value;
        std::cout << this->value << std::endl;
    }

    void set_active(bool state) {
        this->active = state;
        std::cout << this->active << std::endl;
    }

    void toggle_active() {
        this->active = !this->active;
    }

    void set_has_value(bool state) {
        this->has_value = state;
    }

    void toggle_has_value() {
        this->has_value = !this->has_value;
    } 

    std::string to_string() {
        std::ostringstream out("", std::ios_base::app);
        if(this->has_value) {
            out << " " << value;
        }

        return out.str();
    }
};

}