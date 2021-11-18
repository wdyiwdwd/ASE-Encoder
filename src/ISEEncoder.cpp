#include "ISEEncoder.h"

ISE_NAMESPACE_START

double ISEEncoder::GetQuantityOfInformationByLevel(int level) {
  if (level <= 1) {
    return 0;
  }
  double sumOfImportances = 0;
  for (int i = 0; i < level - 1; i++) {
    sumOfImportances += mFeatureImportances[i];
  }
  return sumOfImportances;
}

ISEFeatureSegmentStarts ISEEncoder::DivideFeatureSegmentsByImportances(ISEFeatureImportances& featureImportances) {
  ISEFeatureSegmentStarts segmentStarts(mParam.mCombination, 0);
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

int ISEEncoder::GetOptimalGMMNumberByRedundantKernels(ISEFeatureSet& featureSet, int segmentStart, int segmentEnd) {
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
  vector<ISEGMMWeight> weights(redundantKernelNumber, 0);
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

int ISEEncoder::GetOptimalGMMNumberByBIC(ISEFeatureSet& featureSet, int segmentStart, int segmentEnd) {
  if (featureSet.size() == 0) {
    return 0;
  }
  int tGMMNumber = -1;
  double sampleRatio = 0.3;
  int sampleNum = ceil(sampleRatio * featureSet.size());
  int sampleDivide = floor(featureSet.size() / sampleNum);

  int featureDimension = segmentEnd - segmentStart;
  int maxIterations = 200;
  double minBIC = ISE_MAX_DOUBLE;
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

void ISEEncoder::FitGMMByFeatureSegmentSet(ISEFeatureSet& featureSet, int segmentStart, int segmentEnd) {
  double minBIC = ISE_MAX_DOUBLE;
  ISEGMMClusters gmmResults;
  gmmResults.mFeatureDimension = segmentEnd - segmentStart;
  int numOfCluster = GetOptimalGMMNumberByBIC(featureSet, segmentStart, segmentEnd);
  int featureDimension = segmentEnd - segmentStart;
  int featureNumber = featureSet.size();
  int maxIterations = 300;
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

void ISEEncoder::Train(ISEFeatureSet& featureSet) {
  mPrepoc = ISEPrepocessor::GetInstance();
  mFeatureImportances = mPrepoc->TrainPrepocessing(featureSet);
  ISEFeatureSegmentStarts segmentStart = DivideFeatureSegmentsByImportances(mFeatureImportances);
  ISEThreadReturnVector threadReturn;
  mThreads = ISEThreadPool::GetInstance(mParam.mThread.mThreadNumber);
  for (auto i = 0; i < segmentStart.size(); i++) {
    // fgetc(stdin);
    if (mParam.mThread.mThreadNumber) {
      threadReturn.emplace_back(mThreads->Enqueue(&ISEEncoder::FitGMMByFeatureSegmentSet, this, ISE_REF(featureSet), segmentStart[i], (i + 1 < segmentStart.size()) ? segmentStart[i + 1] : featureSet[0].size()));
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
  mTree->SetPurningParam(mParam.mPattern.mEncodingPattern == ISEEncodingPattern::ISE_ENCODING_EFFICIENT, mParam.mPattern.mPruningRate);
}

ISEDescriptor ISEEncoder::Encode(ISEFeatureSet& featureSet) {
  if (!mIsTrained) return ISEDescriptor();
  mPrepoc = ISEPrepocessor::GetInstance();
  mPrepoc->Prepocessing(featureSet);
  auto feedbackKernel =  [](ISEFeatureSegment& segment, ISETreeNodeData& data) -> double {
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

  ISEDescriptor des = mTree->GetDescriptorFromTree(mTree->GetTreeLevel());
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

ISEEncoderPtr ISEEncoder::mEncoder = nullptr;

ISE_NAMESPACE_END