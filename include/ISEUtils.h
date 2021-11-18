#ifndef ISE_UTILS
#define ISE_UTILS

#include "ISEPrerequisite.h"
#include <string>

USING_NAMESPACE_STD;

ISE_NAMESPACE_START

class ISEUtils
{
public:
  ~ISEUtils() {};
  static double EuclideanDistance(const ISEDescriptor& des1, const ISEDescriptor& des2);
  static void CoutDescriptor(string name, const ISEDescriptor& des);
private:
  ISEUtils() {};
};

ISE_NAMESPACE_END


#endif