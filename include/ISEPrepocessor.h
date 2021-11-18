#ifndef ISE_PREPOCESSOR
#define ISE_PREPOCESSOR

#include "ISEPrerequisite.h"
#include <eigen3/Eigen/Eigenvalues>

USING_NAMESPACE_STD;
USING_NAMESPACE_EIGEN;

ISE_NAMESPACE_START

using ISEFeatureMatrix = MatrixXd;
using ISEEigenSolver = SelfAdjointEigenSolver<ISEFeatureMatrix>;
using ISEEigenValues = VectorXd;
using ISEEigenVectors = MatrixXd;
using ISEFeatureBorderSet = vector<double>;
using ISEFeatureMeans = vector<double>;
using ISEFeatureStds = vector<double>;

class ISEPrepocessor
{
public:
  ~ISEPrepocessor() {}

  static ISEPrepocessorPtr GetInstance() {
    if (mPocessor == nullptr) {
      mPocessor = new ISEPrepocessor();
    }
    return mPocessor;
  }

  static void Destroy() {
    if (mPocessor != nullptr) {
      cout << "safe delete prepocessor!" << endl;
      SAFE_DELETE(mPocessor);
    }
  }

  ISEFeatureImportances TrainPrepocessing(ISEFeatureSet& trainset);

  void TrainMaxMinScalingParam(ISEFeatureSet& traintset);
  void TrainZScoreScalingParam(ISEFeatureSet& trainset);

  ISEFeatureImportances TrainOrthogonalingMatrix(ISEFeatureSet& trainset);

  void MaxMinScaling(ISEFeatureSet& features);
  void ZScoreScaling(ISEFeatureSet& features);

  void Orthogonaling(ISEFeatureSet& features);

  void Prepocessing(ISEFeatureSet& features);

private:
  ISEPrepocessor() {};
  static ISEPrepocessorPtr mPocessor;
  
  ISEFeatureBorderSet mMax;
  ISEFeatureBorderSet mMin;
  ISEFeatureMeans mMeans;
  ISEFeatureStds mStds;
  ISEEigenValues mEigenValues;
  ISEEigenVectors mEigenVectors;
  int mFeatureDimention;
};

ISE_NAMESPACE_END

#endif