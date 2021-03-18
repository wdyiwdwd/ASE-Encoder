#include "beginner_tutorials/Config.h"

namespace enc {

void Config::initialize(const std::string& filename)
{
    if ( config_ == nullptr ){
        config_ = new Config();
    }
    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    if ( config_->file_.isOpened() == false )
    {
        std::cerr<< "parameter file " << filename << " does not exist." << std::endl;
        config_->file_.release();
        return;
    }
}


Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
    delete config_;
}

Config* Config::config_ = nullptr;

}