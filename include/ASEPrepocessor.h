#ifndef ASE_PREPOCESSOR
#define ASE_PREPOCESSOR

#include "ASEPrerequisite.h"
#include <eigen3/Eigen/Eigenvalues>

USING_NAMESPACE_STD;
USING_NAMESPACE_EIGEN;

ASE_NAMESPACE_START

using ASEFeatureMatrix = MatrixXd;
using ASEEigenSolver = SelfAdjointEigenSolver<ASEFeatureMatrix>;
using ASEEigenValues = VectorXd;
using ASEEigenVectors = MatrixXd;
using ASEFeatureBorderSet = vector<double>;
using ASEFeatureMeans = vector<double>;
using ASEFeatureStds = vector<double>;

class ASEPrepocessor
{
public:
  ~ASEPrepocessor() {}

  static ASEPrepocessorPtr GetInstance() {
    if (mPocessor == nullptr) {
      mPocessor = new ASEPrepocessor();
    }
    return mPocessor;
  }

  static void Destroy() {
    if (mPocessor != nullptr) {
      cout << "safe delete prepocessor!" << endl;
      SAFE_DELETE(mPocessor);
    }
  }

  ASEFeatureImportances TrainPrepocessing(ASEFeatureSet& trainset);

  void TrainMaxMinScalingParam(ASEFeatureSet& traintset);
  void TrainZScoreScalingParam(ASEFeatureSet& trainset);

  ASEFeatureImportances TrainOrthogonalingMatrix(ASEFeatureSet& trainset);

  void MaxMinScaling(ASEFeatureSet& features);
  void ZScoreScaling(ASEFeatureSet& features);

  void Orthogonaling(ASEFeatureSet& features);

  void Prepocessing(ASEFeatureSet& features);

private:
  ASEPrepocessor() {};
  static ASEPrepocessorPtr mPocessor;
  
  ASEFeatureBorderSet mMax;
  ASEFeatureBorderSet mMin;
  ASEFeatureMeans mMeans;
  ASEFeatureStds mStds;
  ASEEigenValues mEigenValues;
  ASEEigenVectors mEigenVectors;
  int mFeatureDimention;
};

ASE_NAMESPACE_END

#endif