#ifndef SCAN_MATCHING_CONFIG_H
#define SCAN_MATCHING_CONFIG_H

#include "beginner_tutorials/common_include.h"
#include <sstream>

namespace enc {

class Config
{
private:
    static Config* config_; 
    cv::FileStorage file_;
    Config () {} // private constructor makes a singleton
public:
    ~Config();  // close the file when deconstructing 
    // set a new config file 
    static void initialize( const std::string& filename ); 
    // access the parameter values
    template < typename T>
    static T get( const std::string& key )
    {
        return T( Config::config_->file_[key] );
    }
};

}

#endif