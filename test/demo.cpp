#include "ISEUtils.h"
#include "ISEEncoder.h"
#include <fstream>
#include <time.h>

USING_NAMESPACE_STD;
USING_NAMESPACE_ISE;

int main()
{
  cout << "for test" << endl;

  ifstream in("../data/trainingdata.txt");
  ISEFeatureSet trainingData(500, ISEFeature(32, 0));
  for (auto i = 0; i < 500; i++) {
    for (auto j = 0; j < 32; j++) {
      in >> trainingData[i][j];
    }
  }
  in.close();

  in.open("../data/encodingdata.txt");
  ISEFeatureSet data(500, ISEFeature(32, 0));
  for (auto i = 0; i < 500; i++) {
    for (auto j = 0; j < 32; j++) {
      in >> data[i][j];
    }
  }
  in.close();

  auto encoder = ISEEncoder::GetInstance();
  ISEEncoderParam param;
  param.mCombination = 7;
  param.mMaxDimention = 10000;
  param.mPattern.mEncodingPattern = ISEEncodingPattern::ISE_ENCODING_EFFICIENT;
  param.mPattern.mPruningRate = 10e-3;
  param.mThread.mIsMultiThread = true;
  param.mThread.mThreadNumber = 12;
  encoder->SetParam(param);


  clock_t trainingStart = clock();
  encoder->Train(trainingData);

  clock_t trainingEnd = clock();
  
  ISEDescriptor result = encoder->Encode(data);
  clock_t encodingEnd = clock();
  
  cout << "input data size: " << data.size() << " " << data[0].size() << endl;
  cout << "output data size: " << 1 << " " << result.size() << endl;
  cout << "training time: " << (1.0 * trainingEnd - trainingStart) / CLOCKS_PER_SEC << endl;
  cout << "encoding time: " << (1.0 * encodingEnd - trainingEnd) / CLOCKS_PER_SEC << endl;
  return 0; 
}