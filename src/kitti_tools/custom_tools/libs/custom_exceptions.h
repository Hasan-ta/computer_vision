#ifndef CUSTOM_EXCEPTIONS_H
#define CUSTOM_EXCEPTIONS_H

#include <iostream>
#include <exception>
using namespace std;


namespace perception{
namespace kitti{

class myexception: public exception
{
  virtual const char* what() const throw()
  {
    return "\n Error!! Invalid Path. Make Sure the path contains training folder \n";
  }
} invalid_path;

}
}

#endif
