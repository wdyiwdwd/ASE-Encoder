#ifndef ASE_UTILS
#define ASE_UTILS

#include "ASEPrerequisite.h"
#include <string>

USING_NAMESPACE_STD;

ASE_NAMESPACE_START

class ASEUtils
{
public:
  ~ASEUtils() {};
  static double EuclideanDistance(const ASEDescriptor& des1, const ASEDescriptor& des2);
  static void CoutDescriptor(string name, const ASEDescriptor& des);
private:
  ASEUtils() {};
};

ASE_NAMESPACE_END


#endif