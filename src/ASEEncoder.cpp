#include "ASEEncoder.h"

ASE_NAMESPACE_START

double ASEEncoder::GetQuantityOfInformationByLevel(int level) {
  if (level <= 1) {
    return 0;
  }
  double sumOfImportances = 0;
  for (int i = 0; i < level - 1; i++) {
    sumOfImportances += mFeatureImportances[i];
  }
  return sumOfImportances;
}

ASEFeatureSegmentStarts ASEEncoder::DivideFeatureSegmentsByImportances(ASEFeatureImportances& featureImportances) {
  ASEFeatureSegmentStarts segmentStarts(mParam.mCombination, 0);
  if (featureImportances.size() <= mParam.mCombination) {
    for (auto count = 0; count < featureImportances.size(); count++) {
      segmentStarts[count] = count;
    }
    return segmentStarts;
  }

  double allImportance = 0;
  for (auto i = 0; i < featureImportances.size(); i++) {
    allImportance += featureImportances[i];
  }

  int count = 1;
  double eachImportance = allImportance / mParam.mCombination;
  double sumOfImportance = 0;
  for (auto i = 0; i < featureImportances.size(); i++) {
    sumOfImportance += featureImportances[i];
    if (count >= segmentStarts.size()) break;
    if (sumOfImportance > count * eachImportance) {
      segmentStarts[count] = i + 1;
      count++;
    }
  }

  return segmentStarts;
}

int ASEEncoder::GetOptimalGMMNumberByRedundantKernels(ASEFeatureSet& featureSet, int segmentStart, int segmentEnd) {
  if (featureSet.size() == 0) {
    return 0;
  }
  int tGMMNumber = -1;
  double sampleRatio = 0.2;
  int sampleNum = ceil(sampleRatio * featureSet.size());
  int sampleDivide = floor(featureSet.size() / sampleNum);

  int featureDimension = segmentEnd - segmentStart;
  int maxIterations = 100;
  int redundantKernelNumber = ceil(pow(mParam.mMaxDimention, 1.0 / mParam.mCombination));
  string typeGMM = "diagonal";
  Gaussian_Mixture_Model gmm(typeGMM, featureDimension, redundantKernelNumber);
  double** data = new double*[sampleNum];
  for (auto i = 0; i < sampleNum; i++) {
    data[i] = new double[featureDimension];
    for (auto j = 0; j < featureDimension; j++) {
      data[i][j] = featureSet[i * sampleDivide][j + segmentStart];
    }
  }
  double lastLogLikelihood = 0;
  double logLikelihood;
  for (auto i = 0; i < maxIterations; i++){
    if (i == 0) gmm.Initialize(sampleNum, data);
    logLikelihood = gmm.Expectaion_Maximization(sampleNum, data) / sampleNum;

    if (abs(logLikelihood - lastLogLikelihood) < 10e-6) {
      break;
    }
    lastLogLikelihood = logLikelihood;
  }
  vector<ASEGMMWeight> weights(redundantKernelNumber, 0);
  for (auto i = 0; i < weights.size(); i++) {
    weights[i] = gmm.weight[i];
  }
  sort(weights.begin(), weights.end());
  int kernelNum = redundantKernelNumber;
  for (auto i = 0; i < weights.size(); i++) {
    if (weights[i] < 1.0 / redundantKernelNumber / 5) {
      kernelNum--;
    }
    else {
      break;
    }
  }

  return kernelNum;
}

int ASEEncoder::GetOptimalGMMNumberByBIC(ASEFeatureSet& featureSet, int segmentStart, int segmentEnd) {
  if (featureSet.size() == 0) {
    return 0;
  }
  int tGMMNumber = -1;
  double sampleRatio = 0.2;
  int sampleNum = ceil(sampleRatio * featureSet.size());
  int sampleDivide = floor(featureSet.size() / sampleNum);

  int featureDimension = segmentEnd - segmentStart;
  int maxIterations = 300;
  double minBIC = ASE_MAX_DOUBLE;
  for (auto numOfCluster = 2; numOfCluster <= ceil(pow(mParam.mMaxDimention, 1.0 / mParam.mCombination)); numOfCluster+=1) {
    string typeGMM = "diagonal";
    Gaussian_Mixture_Model gmm(typeGMM, featureDimension, numOfCluster);
    double** data = new double*[sampleNum];
    for (auto i = 0; i < sampleNum; i++) {
      data[i] = new double[featureDimension];
      for (auto j = 0; j < featureDimension; j++) {
        data[i][j] = featureSet[i * sampleDivide][j + segmentStart];
      }
    }
    double lastLogLikelihood = 0;
    double logLikelihood;
    for (auto i = 0; i < maxIterations; i++){
      if (i == 0) gmm.Initialize(sampleNum, data);
      logLikelihood = gmm.Expectaion_Maximization(sampleNum, data) / sampleNum;
      // cout << "iter: " << i << " log-likelihood: " << logLikelihood << endl;
      if (abs(logLikelihood - lastLogLikelihood) < 10e-6) {
        break;
      }
      lastLogLikelihood = logLikelihood;
    }
    double BIC = gmm.Compute_BIC(logLikelihood * sampleNum, sampleNum);
    // cout << "get BIC " << BIC << endl; 
    if (BIC < minBIC) {
      minBIC = BIC;
      tGMMNumber = numOfCluster;
    }
    else {
      break;
    }
  }

  return tGMMNumber;
}

void ASEEncoder::FitGMMByFeatureSegmentSet(ASEFeatureSet& featureSet, int segmentStart, int segmentEnd) {
  double minBIC = ASE_MAX_DOUBLE;
  ASEGMMClusters gmmResults;
  gmmResults.mFeatureDimension = segmentEnd - segmentStart;
  int numOfCluster = GetOptimalGMMNumberByBIC(featureSet, segmentStart, segmentEnd);
  int featureDimension = segmentEnd - segmentStart;
  int featureNumber = featureSet.size();
  int maxIterations = 1000;
  string typeGMM = "diagonal";
  Gaussian_Mixture_Model gmm(typeGMM, featureDimension, numOfCluster);
  double** data = new double*[featureNumber];
  for (auto i = 0; i < featureNumber; i++) {
    data[i] = new double[featureDimension];
    for (auto j = 0; j < featureDimension; j++) {
      data[i][j] = featureSet[i][j + segmentStart];
    }
  }
  double lastLogLikelihood = 0;
  double logLikelihood;
  for (auto i = 0; i < maxIterations; i++){
    if (i == 0) gmm.Initialize(featureNumber, data);
    logLikelihood = gmm.Expectaion_Maximization(featureNumber, data) / featureNumber;
    // cout << "iter: " << i << " log-likelihood: " << logLikelihood << endl;
    if (abs(logLikelihood - lastLogLikelihood) < 10e-6) {
      // cout << "iter: " << i << endl;
      break;
    }
    lastLogLikelihood = logLikelihood;
  }

  gmmResults.mClustersNum = numOfCluster;
  gmmResults.mCovClusters.resize(numOfCluster);
  for (auto j = 0; j < gmmResults.mCovClusters.size(); j++) {
    gmmResults.mCovClusters[j].resize(featureDimension);
    for (auto k = 0; k < gmmResults.mCovClusters[j].size(); k++) {
      gmmResults.mCovClusters[j][k].resize(featureDimension, 0);
      for (auto l = 0; l < gmmResults.mCovClusters[j][k].size(); l++) {
        if (k == l) 
          gmmResults.mCovClusters[j][k][l] = gmm.diagonal_covariance[j][k];
      }
    }
  }
  gmmResults.mMeanClusters.resize(numOfCluster);
  for (auto j = 0; j < gmmResults.mMeanClusters.size(); j++) {
    gmmResults.mMeanClusters[j].resize(featureDimension);
    for (auto k = 0; k < gmmResults.mMeanClusters[j].size(); k++) {
      gmmResults.mMeanClusters[j][k] = gmm.mean[j][k];
    }
  }
  gmmResults.mWeightClutser.resize(numOfCluster);
  for (auto j = 0; j < numOfCluster; j++) {
    gmmResults.mWeightClutser[j] = gmm.weight[j];
  }

  for (int i = 0; i < featureNumber; i++){
    delete[] data[i];
  }
  delete[] data;
  mTree->AddTreeLevel(gmmResults);
  // cout << *mTree << endl << endl;
}

void ASEEncoder::Train(ASEFeatureSet& featureSet) {
  mPrepoc = ASEPrepocessor::GetInstance();
  mFeatureImportances = mPrepoc->TrainPrepocessing(featureSet);
  ASEFeatureSegmentStarts segmentStart = DivideFeatureSegmentsByImportances(mFeatureImportances);
  ASEThreadReturnVector threadReturn;
  mThreads = ASEThreadPool::GetInstance(mParam.mThread.mThreadNumber);
  for (auto i = 0; i < segmentStart.size(); i++) {
    // fgetc(stdin);
    if (mParam.mThread.mThreadNumber) {
      threadReturn.emplace_back(mThreads->Enqueue(&ASEEncoder::FitGMMByFeatureSegmentSet, this, ASE_REF(featureSet), segmentStart[i], (i + 1 < segmentStart.size()) ? segmentStart[i + 1] : featureSet[0].size()));
    }
    else {
      FitGMMByFeatureSegmentSet(featureSet, segmentStart[i], (i + 1 < segmentStart.size()) ? segmentStart[i + 1] : featureSet[0].size());
    }
  }
  for (auto& t: threadReturn) {
    t.get();
  }
  mIsTrained = true;
  if (mParam.mThread.mIsMultiThread) {
    mTree->ReserveSpaceForMultiThread(mParam.mThread.mThreadNumber);
  }
  mTree->SetPurningParam(mParam.mPattern.mEncodingPattern == ASEEncodingPattern::ASE_ENCODING_EFFICIENT, mParam.mPattern.mPruningRate);
}

ASEDescriptor ASEEncoder::Encode(ASEFeatureSet& featureSet) {
  if (!mIsTrained) return ASEDescriptor();
  mPrepoc = ASEPrepocessor::GetInstance();
  mPrepoc->Prepocessing(featureSet);

  auto feedbackKernel =  [](ASEFeatureSegment& segment, ASETreeNodeData& data) -> double {
    double result = 0;
    double* featureSegment = segment.data();
    double* featureMean = data.mMeans.data();
    double* featureCov = new double [data.mDimension];
    for (auto i = 0; i < data.mDimension; i++) {
        featureCov[i] = data.mCovs[i][i];
    }
    Gaussian_Mixture_Model gmm("diagonal", data.mDimension, 1);
    result = data.mGMMWeight * gmm.Gaussian_Distribution(featureSegment, featureMean, featureCov);
    // result = gmm.Gaussian_Distribution(featureSegment, featureMean, featureCov);
    delete[] featureCov;
    return result;
  };
  
  if (mParam.mThread.mIsMultiThread) {
    mTree->UpdateTreeNodeWeightsWithMultiThreads(featureSet, feedbackKernel);
  } else {
    mTree->UpdateTreeNodeWeights(featureSet, feedbackKernel);
  }

  //  ASEDescriptor des = mTree->GetDescriptorFromTree(mTree->GetTreeLevel());
  ASEDescriptor des = mTree->GetDescriptorFromTree(mTree->GetTreeLevel() - 1);
  mTree->ResetWeightsOfTree();

  // power normalization
  for (auto i = 0; i < des.size(); i++) {
    des[i] = sqrt(des[i]);
  }

  // L2 normalization
  double normalizationSum = 0;
  for (auto i = 0; i < des.size(); i++) {
    normalizationSum += des[i] * des[i];
  }
  for (auto i = 0; i < des.size(); i++) {
    des[i] = des[i] / sqrt(normalizationSum);
  }

  return des;
}

ASEEncoderPtr ASEEncoder::mEncoder = nullptr;

ASE_NAMESPACE_END