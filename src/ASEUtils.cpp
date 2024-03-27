#include "ASEUtils.h"

ASE_NAMESPACE_START

double ASEUtils::EuclideanDistance(const ASEDescriptor& des1, const ASEDescriptor& des2) {
  if (des1.size() == 0 || des1.size() != des2.size()) {
    cout << "EuclideanDistance: descriptor is invalid!" << endl;
    return 0;
  }
  double dist = 0;
  for (auto i = 0; i < des1.size(); i++) {
    dist += (des1[i] - des2[i]) * (des1[i] - des2[i]);
  }
  return sqrt(dist);
}

void ASEUtils::CoutDescriptor(string name, const ASEDescriptor& des) {
  cout << name << ": " << endl;
  for (auto i = 0; i < des.size(); i++) {
    cout << des[i] << " ";
  }
  cout << endl << endl;
}

ASE_NAMESPACE_END