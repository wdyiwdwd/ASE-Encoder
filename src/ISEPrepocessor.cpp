#include "ISEPrerequisite.h"
#include "ISEPrepocessor.h"

ISE_NAMESPACE_START

void ISEPrepocessor::TrainMaxMinScalingParam(ISEFeatureSet& trainset) {
  if (trainset.size() == 0) {
    cout << "TrainMaxMinScalingParam: traintrainset is empty!" << endl;
    return;
  }


  mMax.resize(trainset[0].size(), ISE_MIN_DOUBLE);
  mMin.resize(trainset[0].size(), ISE_MAX_DOUBLE);

  for (auto i = 0; i < trainset.size(); i++) {
    for (auto j = 0; j < trainset[i].size(); j++) {
      if (trainset[i][j] > mMax[j]) {
        mMax[j] = trainset[i][j];
      }
      if (trainset[i][j] < mMin[j]) {
        mMin[j] = trainset[i][j];
      }
    }
  }

}

void ISEPrepocessor::MaxMinScaling(ISEFeatureSet& features) {
  if (features.size() == 0) {
    cout << "MaxMinScaling: feature set is empty!" << endl;
    return;
  }
  if (mMax.size() == 0 || mMin.size() == 0) {
    cout << "not train maxmin-scaling param!" << endl;
    return;
  }
  if (mMax.size() != features[0].size() || mMin.size() != features[0].size()) {
    cout << "MaxMinScaling: maxmin-scaling params does not match feature set" << endl;
    return;
  }

  for (auto i = 0; i < features.size(); i++) {
    for (auto j = 0; j < features[i].size(); j++) {
      features[i][j] = (features[i][j] - mMin[j]) / (mMax[j] - mMin[j]);
      if (features[i][j] > 1) features[i][j] = 1;
      else if (features[i][j] < 0) features[i][j] = 0;
    }
  }

  // for (auto& feature : features) {
  //   for (auto& value : feature) {
  //     cout << value << " ";
  //   }
  //   cout << endl;
  // }
}

void ISEPrepocessor::TrainZScoreScalingParam(ISEFeatureSet& trainset) {
  if (trainset.size() == 0) {
    cout << "TrainZScoreScalingParam: traintrainset is empty!" << endl;
    return;
  }

  mMeans.resize(trainset[0].size(), 0);
  mStds.resize(trainset[0].size(), 0);

  for (auto i = 0; i < trainset.size(); i++) {
    for (auto j = 0; j < trainset[i].size(); j++) {
      mMeans[j] += (trainset[i][j]);
    }
  }

  for (auto i = 0; i < mMeans.size(); i++) {
    mMeans[i] /= trainset.size();
  }

  for (auto i = 0; i < trainset.size(); i++) {
    for (auto j = 0; j < trainset[i].size(); j++) {
      mStds[j] += (trainset[i][j] - mMeans[j]) * (trainset[i][j] - mMeans[j]);
    }
  }

  for (auto i = 0; i < mStds.size(); i++) {
    mStds[i] = sqrt(mStds[i] / trainset.size());
  }
}

void ISEPrepocessor::ZScoreScaling(ISEFeatureSet& features) {
  if (features.size() == 0) {
    cout << "ZScoreScaling: feature set is empty!" << endl;
    return;
  }
  if (mMeans.size() == 0 || mStds.size() == 0) {
    cout << "ZScoreScaling: not train z-scores-scaling param!" << endl;
    return;
  }
  if (mMeans.size() != features[0].size() || mStds.size() != features[0].size()) {
    cout << "ZScoreScaling: z-scores-scaling params does not match feature set" << endl;
    return;
  }

  // cout << "ZScoreScaling..." << endl;
  for (auto i = 0; i < features.size(); i++) {
    for (auto j = 0; j < features[i].size(); j++) {
      if (features[i][j] > mMeans[j] + 3 * mStds[j]) features[i][j] = mMeans[j] + 3 * mStds[j];
      else if (features[i][j] < mMeans[j] - 3 * mStds[j]) features[i][j] = mMeans[j] - 3 * mStds[j];
      if (mStds[j] == 0) features[i][j] = 0;
      else features[i][j] = (features[i][j] - mMeans[j]) / mStds[j];
    }
  }

}

ISEFeatureImportances ISEPrepocessor::TrainOrthogonalingMatrix(ISEFeatureSet& trainset) {
  if (trainset.size() == 0) {
    cout << "TrainOrthogonalingMatrix: trainset is empty!" << endl;
    return move(ISEFeatureImportances());
  }

  // constexpr double dimensionReductionRadio = 0.87;
  constexpr double dimensionReductionRadio = 0.87;

  ISEFeatureMatrix trainMatrix(trainset[0].size(), trainset.size());
  for (auto i = 0; i < trainset.size(); i++) {
    for (auto j = 0; j < trainset[0].size(); j++) {
      trainMatrix(j, i) = trainset[i][j];
    }
  }

  auto convMatrix = (1.0d / trainset.size()) * trainMatrix * trainMatrix.transpose();


  // the eigen values are sorted from small to large
  ISEEigenSolver eigenSolver(convMatrix);
  auto eigenValues = eigenSolver.eigenvalues();
  auto eigenVectors = eigenSolver.eigenvectors();


  auto reverseEigenValues = eigenValues;
  auto reverseEigenVectors = eigenVectors;
  for (auto i = 0; i < eigenValues.rows(); i++) {
    reverseEigenValues(eigenValues.rows() - i - 1) = eigenValues(i);
  }
  for (auto i = 0; i < eigenVectors.rows(); i++) {
    for (auto j = 0; j < eigenVectors.cols(); j++) {
      reverseEigenVectors(i, eigenVectors.cols() - j - 1) = eigenVectors(i, j);
    }
  }

  double eigenValueSum = 0;
  for (auto i = 0; i < reverseEigenValues.rows(); i++) {
    eigenValueSum += reverseEigenValues(i);
  }
  ISEFeatureImportances importances;
  double eigenValueTempSum = 0;
  for (auto i = 0; i < reverseEigenValues.rows(); i++) {
    importances.push_back(reverseEigenValues(i) / eigenValueSum);
    eigenValueTempSum += reverseEigenValues(i) / eigenValueSum;
    if (eigenValueTempSum > dimensionReductionRadio) break;
  }

  mFeatureDimention = importances.size();
  mEigenValues = reverseEigenValues.block(0, 0, importances.size(), 1);
  mEigenVectors = reverseEigenVectors.block(0, 0, reverseEigenVectors.rows(), importances.size());

  return importances;
} 

void ISEPrepocessor::Orthogonaling(ISEFeatureSet& features) {
  if (features.size() == 0) {
    cout << "Orthogonaling: feature set is empty!" << endl;
    return;
  }

  if (mEigenValues.rows() == 0) {
    cout << "Orthogonaling: the model is not well trained!" << endl;
    return;
  }

  ISEFeatureMatrix trainMatrix(features[0].size(), features.size());
  for (auto i = 0; i < features.size(); i++) {
    for (auto j = 0; j < features[0].size(); j++) {
      trainMatrix(j, i) = features[i][j];
    }
  }

  trainMatrix =  (mEigenVectors.transpose() * trainMatrix);


  ISEFeatureSet orthogonalingFeatures(features.size(), ISEFeature(mFeatureDimention, 0));

  for (auto i = 0; i < orthogonalingFeatures.size(); i++) {
    for (auto j = 0; j < orthogonalingFeatures[0].size(); j++) {
      orthogonalingFeatures[i][j] = trainMatrix(j, i);
    }
  }

  features = orthogonalingFeatures;
} 

ISEFeatureImportances ISEPrepocessor::TrainPrepocessing(ISEFeatureSet& trainset) {
  TrainZScoreScalingParam(trainset);
  ZScoreScaling(trainset);
  // TrainMaxMinScalingParam(trainset);
  // MaxMinScaling(trainset);
  ISEFeatureImportances importances = TrainOrthogonalingMatrix(trainset);
  Orthogonaling(trainset);
  return importances;
}

void ISEPrepocessor::Prepocessing(ISEFeatureSet& features) {
  ZScoreScaling(features);
  // MaxMinScaling(features);
  Orthogonaling(features);
}


ISEPrepocessorPtr ISEPrepocessor::mPocessor = nullptr;

ISE_NAMESPACE_END