#include "ISEUtils.h"
#include "ISEEncoder.h"

USING_NAMESPACE_STD;
USING_NAMESPACE_ISE;

int main()
{
  cout << "for test" << endl;
  ISEFeatureSet trainset({
    {1, 2, 3},
    {4, 1, 2.4},
    {1, -2, 2},
    {4, 0, 2}, 
    {0, 1, 2},
    {4, 2, 3},
    {1, 3, 2},
    {6, 3, 2},
    {4, 1, 5},
    {3, -2, 1},
    {5, 4, 7},
    {1.1, 2, 3}
  });

  ISEFeatureSet data({
    {1, 4, 4},
    {3, 2, 4},
    {2, -2, 1},
    {1, .60, 3}, 
    {1, 1, 2},
  });

  auto encoder = ISEEncoder::GetInstance();
  ISEEncoderParam param;
  param.mCombination = 3;
  param.mMaxDimention = 500;
  param.mPattern.mEncodingPattern = ISEEncodingPattern::ISE_ENCODING_COMPLETE;
  param.mPattern.mPruningRate = 0;
  param.mThread.mIsMultiThread = true;
  param.mThread.mThreadNumber = 12;
  encoder->SetParam(param);

  encoder->Train(trainset);
  
  ISEDescriptor result = encoder->Encode(data);
  ISEUtils::CoutDescriptor("descriptor", result);
  return 0; 
}