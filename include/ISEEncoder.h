#ifndef ISE_ENCODER
#define ISE_ENCODER

#include "ISEPrerequisite.h"
#include "ISEPrepocessor.h"
#include "ISETree.h"
#include "GMM.h"

USING_NAMESPACE_STD;
USING_NAMESPACE_EIGEN;

ISE_NAMESPACE_START

using ISEFeatureSegmentStarts = vector<int>;

constexpr static int ISE_DEFAULT_MAX_DIMENTION = 50000;
constexpr static int ISE_DEFAULT_COMBINATION_NUM = 6;
constexpr static int ISE_DEFAULT_THREAD_NUMBER = 12;

enum class ISEEncodingPattern {
  ISE_ENCODING_COMPLETE,
  ISE_ENCODING_EFFICIENT
};

struct ISEEncoderParam
{
  struct ISEThreadParam {
    int mThreadNumber;
    bool mIsMultiThread;
  };

  struct ISEEncodeingPatternParam {
    ISEEncodingPattern mEncodingPattern;
    double mPruningRate;
  };

  ISEThreadParam mThread;
  ISEEncodeingPatternParam mPattern;
  int mCombination;
  int mMaxDimention;

  ISEEncoderParam() {
    mMaxDimention = ISE_DEFAULT_MAX_DIMENTION;
    mCombination = ISE_DEFAULT_COMBINATION_NUM;
    mThread.mIsMultiThread = true;
    mThread.mThreadNumber = ISE_DEFAULT_THREAD_NUMBER;
    mPattern.mEncodingPattern = ISEEncodingPattern::ISE_ENCODING_COMPLETE;
    mPattern.mPruningRate = 0;
  }
};

class ISEEncoder
{
public:
  ~ISEEncoder() {
    ISEPrepocessor::Destroy();
    ISEEncoder::mEncoder = nullptr;
  }

  static ISEEncoderPtr GetInstance() {
    if (mEncoder == nullptr) {
      cout << "Create an ISE encoder!" << endl;
      mEncoder = new ISEEncoder();
    }
    return mEncoder;
  }

  inline void SetParam(ISEEncoderParam& param) {
    mParam = param;
  }

  inline void GetParam(ISEEncoderParam& param) {
    param = mParam;
  }

  void Train(ISEFeatureSet& featureSet);

  ISEDescriptor Encode(ISEFeatureSet& featureSet);

  double GetQuantityOfInformationByLevel(int level);

  int GetOptimalGMMNumberByRedundantKernels(ISEFeatureSet& featureSet, int segmentStart, int segmentEnd);

  int GetOptimalGMMNumberByBIC(ISEFeatureSet& featureSet, int segmentStart, int segmentEnd);

  void FitGMMByFeatureSegmentSet(ISEFeatureSet& featureSet, int segmentStart, int segmentEnd); 

  ISEFeatureSegmentStarts DivideFeatureSegmentsByImportances(ISEFeatureImportances& featureImportances);

private:

  ISEEncoder() {
    mIsTrained = false;
    mTree = ISETreeUniquePtr(new ISETree());
  };

  static ISEEncoderPtr mEncoder;
  bool mIsTrained;
  ISEEncoderParam mParam;
  ISEPrepocessorPtr mPrepoc;
  ISETreeUniquePtr mTree;
  ISEFeatureImportances mFeatureImportances;
  ISEThreadPoolPTR mThreads;
};

ISE_NAMESPACE_END

#endif