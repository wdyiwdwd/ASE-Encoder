#ifndef ASE_ENCODER
#define ASE_ENCODER

#include "ASEPrerequisite.h"
#include "ASEPrepocessor.h"
#include "ASETree.h"
#include "GMM.h"

USING_NAMESPACE_STD;
USING_NAMESPACE_EIGEN;

ASE_NAMESPACE_START

using ASEFeatureSegmentStarts = vector<int>;

constexpr static int ASE_DEFAULT_MAX_DIMENTION = 50000;
constexpr static int ASE_DEFAULT_COMBINATION_NUM = 6;
constexpr static int ASE_DEFAULT_THREAD_NUMBER = 12;

enum class ASEEncodingPattern {
  ASE_ENCODING_COMPLETE,
  ASE_ENCODING_EFFICIENT
};

struct ASEEncoderParam
{
  struct ASEThreadParam {
    int mThreadNumber;
    bool mIsMultiThread;
  };

  struct ASEEncodeingPatternParam {
    ASEEncodingPattern mEncodingPattern;
    double mPruningRate;
  };

  ASEThreadParam mThread;
  ASEEncodeingPatternParam mPattern;
  int mCombination;
  int mMaxDimention;

  ASEEncoderParam() {
    mMaxDimention = ASE_DEFAULT_MAX_DIMENTION;
    mCombination = ASE_DEFAULT_COMBINATION_NUM;
    mThread.mIsMultiThread = true;
    mThread.mThreadNumber = ASE_DEFAULT_THREAD_NUMBER;
    mPattern.mEncodingPattern = ASEEncodingPattern::ASE_ENCODING_COMPLETE;
    mPattern.mPruningRate = 0;
  }
};

class ASEEncoder
{
public:
  ~ASEEncoder() {
    ASEPrepocessor::Destroy();
    ASEEncoder::mEncoder = nullptr;
  }

  static ASEEncoderPtr GetInstance() {
    if (mEncoder == nullptr) {
      cout << "Create an ASE encoder!" << endl;
      mEncoder = new ASEEncoder();
    }
    return mEncoder;
  }

  inline void SetParam(ASEEncoderParam& param) {
    mParam = param;
  }

  inline void GetParam(ASEEncoderParam& param) {
    param = mParam;
  }

  void Train(ASEFeatureSet& featureSet);

  ASEDescriptor Encode(ASEFeatureSet& featureSet);

  double GetQuantityOfInformationByLevel(int level);

  int GetOptimalGMMNumberByRedundantKernels(ASEFeatureSet& featureSet, int segmentStart, int segmentEnd);

  int GetOptimalGMMNumberByBIC(ASEFeatureSet& featureSet, int segmentStart, int segmentEnd);

  void FitGMMByFeatureSegmentSet(ASEFeatureSet& featureSet, int segmentStart, int segmentEnd); 

  ASEFeatureSegmentStarts DivideFeatureSegmentsByImportances(ASEFeatureImportances& featureImportances);

private:

  ASEEncoder() {
    mIsTrained = false;
    mTree = ASETreeUniquePtr(new ASETree());
  };

  static ASEEncoderPtr mEncoder;
  bool mIsTrained;
  ASEEncoderParam mParam;
  ASEPrepocessorPtr mPrepoc;
  ASETreeUniquePtr mTree;
  ASEFeatureImportances mFeatureImportances;
  ASEThreadPoolPTR mThreads;
};

ASE_NAMESPACE_END

#endif