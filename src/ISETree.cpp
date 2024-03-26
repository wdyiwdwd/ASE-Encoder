#include "ISETree.h"

ISE_NAMESPACE_START

ostream& operator << (ostream& out, ISETreeNode& treeNode) {
  out << "||" << treeNode.mData.mDimension << " " << treeNode.mWeight << "|| "; 
  return out;
}

ostream& operator << (ostream& out, ISETree& tree) {
  for (auto i = 0; i < tree.mHeads.size(); i++) {
    auto nodePtr = tree.mHeads[i];
    int count = 0;
    while(nodePtr != nullptr) {
      if (count > 0 && count % nodePtr->GetSubNum() == 0) {
        out << "    ";
      }
      out << *nodePtr;
      count++;
      nodePtr = nodePtr->GetNext();
    }
    out << endl << endl;
  }
  return out;
}


ISETreeNode::ISETreeNode(int level, int order, int subNum, ISETreeNodeType nodeType) : mNodeLevel(level), mNodeOrder(order), mSubNum(subNum), mNodeType(nodeType), mWeight(0), mTempWeight(0), mNext(nullptr), mParent(nullptr) {}

ISETreeNode::ISETreeNode(const ISETreeNode& node) {
  mNodeLevel = node.mNodeLevel;
  mSubNum = node.mSubNum;
  mNodeOrder = node.mNodeOrder;
  mWeight = node.mWeight;
  mTempWeight = node.mTempWeight;
  mData = node.mData;
  mNodeType = node.mNodeType;
  mThreadTempWeights = node.mThreadTempWeights;
  mThreadWeights = node.mThreadWeights;
}

void ISETreeNode::AddChildren(ISETreeNodePtr treeNode) {
    // if (mChildren.size() > 0) {
    //   mChildren[mChildren.size() - 1]->AddNextBrother(treeNode);
    // }
    mChildren.push_back(treeNode);
    treeNode->mParent = this;
}

void ISETreeNode::ComputeWeightsForMultiThread() {
  if (mNodeType == ISETreeNodeType::ISE_TREENODE_TYPE_ROOT) return;
  mWeight = 0;
  for (auto i = 0; i < mThreadWeights.size(); i++) {
    mWeight += mThreadWeights[i];
  }
  mThreadWeights = ISETreeNodeWeightsForMultiThread(mThreadWeights.size(), 0);
  mThreadTempWeights = ISETreeNodeTempWeightsForMultiThread(mThreadWeights.size(), 0);
}

// for ISE_Tree

ISETree::ISETree() : mTreeLevel(1) {
  mRoot = new ISETreeNode(0, 0, 1, ISETreeNodeType::ISE_TREENODE_TYPE_ROOT);
  mRoot->SetWeight(1.0d);
  mRoot->SetTempWeight(1.0d);
  mHeads.push_back(mRoot);
  mLevelSubNums.push_back(1);
  mLevelNums.push_back(1);

  mNeedPurning = false;
  mPurningRate = 0;
}


ISETree::~ISETree() {
  if (mRoot == nullptr) return;
  ISETreeNodeTraverseQueue tnq;
  tnq.push(mRoot);
  while (!tnq.empty()) {
    auto frontNode = tnq.front();
    if (frontNode == nullptr) {
      tnq.pop();
      continue;
    }
    auto children = frontNode->GetChildren();
    for (auto i = 0; i < children.size(); i++) {
        tnq.push(children[i]);
    }
    tnq.pop();
    SAFE_DELETE(frontNode);
  }
}

ISEDescriptor ISETree::GetDescriptorFromTree() {
  if (mTreeLevel < 0) return ISEFeature({});
  return GetDescriptorFromTree(mTreeLevel - 1);
}

ISEDescriptor ISETree::GetDescriptorFromTree(int level) {
  if (level < 0 || level >= mTreeLevel) {
    return GetDescriptorFromTree();
  }
  ISEDescriptor result(mLevelNums[level]);
  ISETreeNodePtr nodePtr = mHeads[level];
  int count = 0;
  while (nodePtr != nullptr) {
    result[count] = nodePtr->GetWeight();
    count++;
    nodePtr = nodePtr->GetNext();
  }
  return result;
}

int ISETree::GetDescriptorSize(int level) {
  if (level >= 0 && level < mLevelNums.size()) {
    return mLevelNums[level];
  }
  return 0;
}

void ISETree::AddTreeLevel(ISEGMMClusters& clusters) {
  auto nodePtr = mHeads[mHeads.size() - 1];
  ISETreeNodePtr newHead = nullptr;
  ISETreeNodePtr nodeTail = nullptr;
  int clusterNum = clusters.mClustersNum;
  while (nodePtr != nullptr) {
    for (auto i = 0; i < clusters.mClustersNum; i++) {
      auto treeNode = new ISETreeNode(nodePtr->GetLevel() + 1, i, clusterNum, ISETreeNodeType::ISE_TREENODE_TYPE_CLUSTER);
      treeNode->SetData(ISETreeNodeData(clusters.mFeatureDimension, clusters.mWeightClutser[i], clusters.mMeanClusters[i], clusters.mCovClusters[i]));
      if (newHead == nullptr) newHead = treeNode;
      if (nodeTail != nullptr) nodeTail->SetNext(treeNode);
      nodePtr->AddChildren(treeNode);
      nodeTail = treeNode;
    }
    nodePtr = nodePtr->GetNext();
  }
  if (newHead != nullptr) {
    mHeads.push_back(newHead);
  }
  mLevelSubNums.push_back(clusters.mClustersNum);
  mLevelNums.push_back(mLevelNums[mLevelNums.size() - 1] * clusters.mClustersNum);
  mTreeLevel++;
}

void ISETree::UpdateTreeNodeWeightsByPruningNodeWithDeep(ISETreeNodePtr node, ISETreeNodeWeights& weights, int featureNumber) {
  if (node == nullptr) return;
  double weight = 1;
  if (node->GetParent() != nullptr) {
    weight = node->GetParent()->GetTempWeight() * weights[node->GetLevel()][node->GetOrder()];
    weight = weight > mPurningRate / (mLevelNums[node->GetLevel()]) ? weight : 0;
    node->SetTempWeight(weight);
    // cout << node->GetTempWeight() << endl;
  }
  if (weight > 0) {
    node->SetWeight(node->GetWeight() + node->GetTempWeight() / featureNumber);
    for (auto i = 0; i < node->GetChildren().size(); i++) {
      UpdateTreeNodeWeightsByPruningNodeWithDeep(node->GetChildren()[i], weights, featureNumber);
    }
  }
}

void ISETree::UpdateTreeNodeWeightsByPruningNodeWithLevel(ISETreeNodeWeights& weights, int featureNumber) {
  ISETreeNodeTraverseQueue treeNodeQueue;
  treeNodeQueue.push(mRoot);
  while (!treeNodeQueue.empty()) {
    auto node = treeNodeQueue.front();
    double weight = 1;
    if (node->GetParent() != nullptr) {
      weight = node->GetParent()->GetTempWeight() * weights[node->GetLevel()][node->GetOrder()];
      weight = weight > mPurningRate / (mLevelNums[node->GetLevel()]) ? weight : 0;
      node->SetTempWeight(weight);
    }
    if (weight > 0) {
      node->SetWeight(node->GetWeight() + node->GetTempWeight() / featureNumber);
      for (auto k = 0; k < node->GetChildren().size(); k++) {
        treeNodeQueue.push(node->GetChildren()[k]);
      }
    }
    treeNodeQueue.pop();
  }
}

void ISETree::UpdateTreeNodeThreadWeightsByPruningNodeWithLevel(int threadID, ISETreeNodeWeights& weights, int featureNumber) {
  ISETreeNodeTraverseQueue treeNodeQueue;
  treeNodeQueue.push(mRoot);
  while (!treeNodeQueue.empty()) {
    auto node = treeNodeQueue.front();
    double weight = 1;
    if (node->GetParent() != nullptr) {
      weight = node->GetParent()->GetThreadTempWeight(threadID) * weights[node->GetLevel()][node->GetOrder()];
      weight = weight > mPurningRate / (mLevelNums[node->GetLevel()]) ? weight : 0;
      node->SetThreadTempWeight(threadID, weight);
    }
    if (weight > 0) {
      node->SetThreadWeight(threadID, node->GetThreadWeight(threadID) + node->GetThreadTempWeight(threadID) / featureNumber);
      for (auto k = 0; k < node->GetChildren().size(); k++) {
        treeNodeQueue.push(node->GetChildren()[k]);
      }
    }
    treeNodeQueue.pop();
  }
}

void ISETree::UpdateTreeNodeWeightsStrictly(ISETreeNodeWeights& weights, int featureNumber) {
  for (auto i = 1; i < mHeads.size(); i++) {
    auto nodePtr = mHeads[i];
    while(nodePtr != nullptr) {
      double weight = nodePtr->GetParent()->GetTempWeight() * weights[nodePtr->GetLevel()][nodePtr->GetOrder()];
      nodePtr->SetTempWeight(weight);
      nodePtr->SetWeight(nodePtr->GetWeight() + nodePtr->GetTempWeight() / featureNumber);
      nodePtr = nodePtr->GetNext();
    }
  }
}

void ISETree::UpdateTreeNodeThreadWeightsStrictly(int threadID, ISETreeNodeWeights& weights, int featureNumber) {
  for (auto i = 1; i < mHeads.size(); i++) {
    auto nodePtr = mHeads[i];
    while(nodePtr != nullptr) {
      double weight = nodePtr->GetParent()->GetThreadTempWeight(threadID) * weights[nodePtr->GetLevel()][nodePtr->GetOrder()];
      nodePtr->SetThreadTempWeight(threadID, weight);
      nodePtr->SetThreadWeight(threadID, nodePtr->GetThreadWeight(threadID) + nodePtr->GetThreadTempWeight(threadID) / featureNumber);
      nodePtr = nodePtr->GetNext();
    }
  }
}

void ISETree::GetWeightsFromOneFeature(ISEFeature& feature, ISETreeNodeWeights& weights, ISETreeNodeWeightKernelCallback kernelCallback) {
  int featureStart = 0;
  int featureEnd = -1;
  int j = 0;
  weights.push_back(ISETreeNodeLevelHeadWeights(1, 1));
  while (featureStart < feature.size()){
    auto nodePtr = mHeads[j + 1];
    featureEnd = featureStart + nodePtr->GetData().mDimension;
    ISETreeNodeLevelHeadWeights lhWeights(mLevelSubNums[j + 1], 0);
    ISEFeatureSegment fsegment(feature.begin() + featureStart, feature.begin() + featureEnd);
    int count = 0;
    double sumWeights = 0;
    while (nodePtr != nullptr) {
      lhWeights[count] = kernelCallback(fsegment, nodePtr->GetData());
      sumWeights += lhWeights[count];
      count++;
      nodePtr = nodePtr->GetNext();
      if (count >= mLevelSubNums[j + 1]) break;
    }
    for (auto k = 0; k < lhWeights.size(); k++) {
      lhWeights[k] = lhWeights[k] / (sumWeights + ISE_MIN_DOUBLE);
    }
    weights.push_back(lhWeights);
    featureStart = featureEnd;
    j++;
  }
}

void ISETree::UpdateTreeNodeWeightsInOneThreadKernel(int threadID, ISEFeatureSet& normFeatures, int start, int end, ISETreeNodeWeightKernelCallback kernelCallback) {
  for (auto i = start; i < end; i++) {
      ISETreeNodeWeights encodingWeights;
      GetWeightsFromOneFeature(normFeatures[i], encodingWeights, kernelCallback);
      if (!mNeedPurning) {
        UpdateTreeNodeThreadWeightsStrictly(threadID, encodingWeights, normFeatures.size());
      } else {
        UpdateTreeNodeThreadWeightsByPruningNodeWithLevel(threadID, encodingWeights, normFeatures.size());
      }
  }
}

void ISETree::ReserveSpaceForMultiThread(int threads) {
  for (auto i = 0; i < mHeads.size(); i++) {
    auto nodePtr = mHeads[i];
    while(nodePtr != nullptr) {
      nodePtr->ResizeThreadWeights(threads);
      nodePtr = nodePtr->GetNext();
    }
  }
}

void ISETree::UpdateTreeNodeWeights(ISEFeatureSet& normFeatures, ISETreeNodeWeightKernelCallback kernelCallback) {
  if (normFeatures.size() == 0) {
    cout << "UpdateTreeNodeWeights: the features is empty!" << endl;
    return;
  }

  for (auto i = 0; i < normFeatures.size(); i++) {
      ISETreeNodeWeights encodingWeights;
      GetWeightsFromOneFeature(normFeatures[i], encodingWeights, kernelCallback);

      if (!mNeedPurning) {
        UpdateTreeNodeWeightsStrictly(encodingWeights, normFeatures.size());
      }
      else {
        // UpdateTreeNodeWeightsByPruningNodeWithDeep(mRoot, encodingWeights, normFeatures.size());
        UpdateTreeNodeWeightsByPruningNodeWithLevel(encodingWeights, normFeatures.size());
      }
  }
}

void ISETree::UpdateTreeNodeWeightsWithMultiThreads(ISEFeatureSet& normFeatures, ISETreeNodeWeightKernelCallback kernelCallback) {
  if (normFeatures.size() == 0) {
    cout << "UpdateTreeNodeWeights: the features is empty!" << endl;
    return;
  }

  ISEThreadPoolPTR threadPool = ISEThreadPool::GetInstance();

  ISEThreadReturnVector threadReturn;
  for (auto i = 0; i < threadPool->GetWorkerSize(); i++) {
    int start = i * normFeatures.size() / threadPool->GetWorkerSize();
    int end = (i + 1) * normFeatures.size() / threadPool->GetWorkerSize();
    if (i + 1 == threadPool->GetWorkerSize()) end = normFeatures.size();
    threadReturn.emplace_back(threadPool->Enqueue(&ISETree::UpdateTreeNodeWeightsInOneThreadKernel, this, i, ISE_REF(normFeatures), start, end, kernelCallback));
  }
  for (auto& t: threadReturn) {
    t.get();
  }

  for (auto i = 1; i < mHeads.size(); i++) {
    auto nodePtr = mHeads[i];
    while(nodePtr != nullptr) {
      nodePtr->ComputeWeightsForMultiThread();
      nodePtr = nodePtr->GetNext();
    }
  }
  
}

void ISETree::ResetWeightsOfTree() {
  for (auto i = 1; i < mHeads.size(); i++) {
    auto nodePtr = mHeads[i];
    while (nodePtr != nullptr) {
      nodePtr->SetTempWeight(0);
      nodePtr->SetWeight(0);
      nodePtr = nodePtr->GetNext();
    }
  }
}

ISE_NAMESPACE_END