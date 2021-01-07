#include <exception>

class Bad_point_exception : public std::exception {
public:

    Bad_point_exception(const char* filename, const char* source, const char* msg) : 
        filename(filename),
        source(source),
        msg(msg) 
    {

    }

    const char * what () const throw ()
    {
    	return msg;
    }

    const char * get_filename() const {
        return filename;
    } 

    const char * get_source() const {
        return source;
    }

private:
    const char* filename;
    const char* source;
    const char* msg;
};