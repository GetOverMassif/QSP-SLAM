#include <boost/filesystem.hpp>

#include <iostream>


bool CreateDirIfNotExist(const std::string file_path)
{
  if (!boost::filesystem::exists(file_path)) {
    if (!boost::filesystem::create_directories(file_path)) {
    //   ROS_ERROR("Failed to create directory: %s", file_path.c_str());
      std::cerr << "Failed to create directory: " << file_path << std::endl;
      return false;
    }
  }
  return true;
}