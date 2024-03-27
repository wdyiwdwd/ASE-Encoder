#include "ASETree.h"

ASE_NAMESPACE_START

ostream& operator << (ostream& out, ASETreeNode& treeNode) {
  out << "||" << treeNode.mData.mDimension << " " << treeNode.mWeight << "|| ";
  return out;
}

ostream& operator << (ostream& out, ASETree& tree) {
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


ASETreeNode::ASETreeNode(int level, int order, int subNum, ASETreeNodeType nodeType) : mNodeLevel(level), mNodeOrder(order), mSubNum(subNum), mNodeType(nodeType), mWeight(0), mTempWeight(0), mNext(nullptr), mParent(nullptr) {}

ASETreeNode::ASETreeNode(const ASETreeNode& node) {
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

void ASETreeNode::AddChildren(ASETreeNodePtr treeNode) {
    // if (mChildren.size() > 0) {
    //   mChildren[mChildren.size() - 1]->AddNextBrother(treeNode);
    // }
    mChildren.push_back(treeNode);
    treeNode->mParent = this;
}

void ASETreeNode::ComputeWeightsForMultiThread() {
  if (mNodeType == ASETreeNodeType::ASE_TREENODE_TYPE_ROOT) return;
  mWeight = 0;
  for (auto i = 0; i < mThreadWeights.size(); i++) {
    mWeight += mThreadWeights[i];
  }
  mThreadWeights = ASETreeNodeWeightsForMultiThread(mThreadWeights.size(), 0);
  mThreadTempWeights = ASETreeNodeTempWeightsForMultiThread(mThreadWeights.size(), 0);
}

// for ASE_Tree

ASETree::ASETree() : mTreeLevel(1) {
  mRoot = new ASETreeNode(0, 0, 1, ASETreeNodeType::ASE_TREENODE_TYPE_ROOT);
  mRoot->SetWeight(1.0f);
  mRoot->SetTempWeight(1.0f);
  mHeads.push_back(mRoot);
  mLevelSubNums.push_back(1);
  mLevelNums.push_back(1);

  mNeedPurning = false;
  mPurningRate = 0;
}


ASETree::~ASETree() {
  if (mRoot == nullptr) return;
  ASETreeNodeTraverseQueue tnq;
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

ASEDescriptor ASETree::GetDescriptorFromTree() {
  if (mTreeLevel < 0) return ASEFeature({});
  return GetDescriptorFromTree(mTreeLevel - 1);
}

ASEDescriptor ASETree::GetDescriptorFromTree(int level) {
  if (level < 0 || level >= mTreeLevel) {
    return GetDescriptorFromTree();
  }
  ASEDescriptor result(mLevelNums[level]);
  ASETreeNodePtr nodePtr = mHeads[level];
  int count = 0;
  while (nodePtr != nullptr) {
    result[count] = nodePtr->GetWeight();
    count++;
    nodePtr = nodePtr->GetNext();
  }
  return result;
}

int ASETree::GetDescriptorSize(int level) {
  if (level >= 0 && level < mLevelNums.size()) {
    return mLevelNums[level];
  }
  return 0;
}

void ASETree::AddTreeLevel(ASEGMMClusters& clusters) {
  auto nodePtr = mHeads[mHeads.size() - 1];
  ASETreeNodePtr newHead = nullptr;
  ASETreeNodePtr nodeTail = nullptr;
  int clusterNum = clusters.mClustersNum;
  while (nodePtr != nullptr) {
    for (auto i = 0; i < clusters.mClustersNum; i++) {
      auto treeNode = new ASETreeNode(nodePtr->GetLevel() + 1, i, clusterNum, ASETreeNodeType::ASE_TREENODE_TYPE_CLUSTER);
      treeNode->SetData(ASETreeNodeData(clusters.mFeatureDimension, clusters.mWeightClutser[i], clusters.mMeanClusters[i], clusters.mCovClusters[i]));
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

void ASETree::UpdateTreeNodeWeightsByPruningNodeWithDeep(ASETreeNodePtr node, ASETreeNodeWeights& weights, int featureNumber) {
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

void ASETree::UpdateTreeNodeWeightsByPruningNodeWithLevel(ASETreeNodeWeights& weights, int featureNumber) {
  ASETreeNodeTraverseQueue treeNodeQueue;
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

void ASETree::UpdateTreeNodeThreadWeightsByPruningNodeWithLevel(int threadID, ASETreeNodeWeights& weights, int featureNumber) {
  ASETreeNodeTraverseQueue treeNodeQueue;
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

void ASETree::UpdateTreeNodeWeightsStrictly(ASETreeNodeWeights& weights, int featureNumber) {
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

void ASETree::UpdateTreeNodeThreadWeightsStrictly(int threadID, ASETreeNodeWeights& weights, int featureNumber) {
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

void ASETree::GetWeightsFromOneFeature(ASEFeature& feature, ASETreeNodeWeights& weights, ASETreeNodeWeightKernelCallback kernelCallback) {
  int featureStart = 0;
  int featureEnd = -1;
  int j = 0;
  weights.push_back(ASETreeNodeLevelHeadWeights(1, 1));
  while (featureStart < feature.size()){
    auto nodePtr = mHeads[j + 1];
    featureEnd = featureStart + nodePtr->GetData().mDimension;
    ASETreeNodeLevelHeadWeights lhWeights(mLevelSubNums[j + 1], 0);
    ASEFeatureSegment fsegment(feature.begin() + featureStart, feature.begin() + featureEnd);
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
      lhWeights[k] = lhWeights[k] / (sumWeights + ASE_MIN_DOUBLE);
    }
    weights.push_back(lhWeights);
    featureStart = featureEnd;
    j++;
  }
}

void ASETree::UpdateTreeNodeWeightsInOneThreadKernel(int threadID, ASEFeatureSet& normFeatures, int start, int end, ASETreeNodeWeightKernelCallback kernelCallback) {
  for (auto i = start; i < end; i++) {
      ASETreeNodeWeights encodingWeights;
      GetWeightsFromOneFeature(normFeatures[i], encodingWeights, kernelCallback);
      if (!mNeedPurning) {
        UpdateTreeNodeThreadWeightsStrictly(threadID, encodingWeights, normFeatures.size());
      } else {
        UpdateTreeNodeThreadWeightsByPruningNodeWithLevel(threadID, encodingWeights, normFeatures.size());
      }
  }
}

void ASETree::ReserveSpaceForMultiThread(int threads) {
  for (auto i = 0; i < mHeads.size(); i++) {
    auto nodePtr = mHeads[i];
    while(nodePtr != nullptr) {
      nodePtr->ResizeThreadWeights(threads);
      nodePtr = nodePtr->GetNext();
    }
  }
}

void ASETree::UpdateTreeNodeWeights(ASEFeatureSet& normFeatures, ASETreeNodeWeightKernelCallback kernelCallback) {
  if (normFeatures.size() == 0) {
    cout << "UpdateTreeNodeWeights: the features is empty!" << endl;
    return;
  }

  for (auto i = 0; i < normFeatures.size(); i++) {
      ASETreeNodeWeights encodingWeights;
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

void ASETree::UpdateTreeNodeWeightsWithMultiThreads(ASEFeatureSet& normFeatures, ASETreeNodeWeightKernelCallback kernelCallback) {
  if (normFeatures.size() == 0) {
    cout << "UpdateTreeNodeWeights: the features is empty!" << endl;
    return;
  }

  ASEThreadPoolPTR threadPool = ASEThreadPool::GetInstance();

  ASEThreadReturnVector threadReturn;
  for (auto i = 0; i < threadPool->GetWorkerSize(); i++) {
    int start = i * normFeatures.size() / threadPool->GetWorkerSize();
    int end = (i + 1) * normFeatures.size() / threadPool->GetWorkerSize();
    if (i + 1 == threadPool->GetWorkerSize()) end = normFeatures.size();
    threadReturn.emplace_back(threadPool->Enqueue(&ASETree::UpdateTreeNodeWeightsInOneThreadKernel, this, i, ASE_REF(normFeatures), start, end, kernelCallback));
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

void ASETree::ResetWeightsOfTree() {
  for (auto i = 1; i < mHeads.size(); i++) {
    auto nodePtr = mHeads[i];
    while (nodePtr != nullptr) {
      nodePtr->SetTempWeight(0);
      nodePtr->SetWeight(0);
      nodePtr = nodePtr->GetNext();
    }
  }
}

ASE_NAMESPACE_END