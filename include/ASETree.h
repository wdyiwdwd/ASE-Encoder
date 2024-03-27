#ifndef ASE_TREE
#define ASE_TREE

#include "ASEPrerequisite.h"
#include <queue>
#include <functional>

USING_NAMESPACE_STD;

ASE_NAMESPACE_START

using ASETreeNodeWeightsForMultiThread = vector<double>;
using ASETreeNodeTempWeightsForMultiThread = vector<double>;

using ASETreeNodeChildren = vector<ASETreeNodePtr>;
using ASETreeLevelHeads = vector<ASETreeNodePtr>;
using ASETreeLevelNums = vector<int>;
using ASETreeLevelSubNums = vector<int>;

using ASETreeNodeLevelHeadWeights = vector<double>;
using ASETreeNodeWeights = vector<ASETreeNodeLevelHeadWeights>;
using ASETreeNodeTraverseQueue = queue<ASETreeNodePtr>;

using ASEGMMWeight = double;
using ASEGMMMeans = vector<double>;
using ASEGMMCovRow = vector<double>;
using ASEGMMCovs = vector<ASEGMMCovRow>;

using ASEGMMWeightClusters = vector<ASEGMMWeight>;
using ASEGMMMeanClusters = vector<ASEGMMMeans>;
using ASEGMMCovCliusters = vector<ASEGMMCovs>;

struct ASEGMMClusters
{
  int mClustersNum;
  int mFeatureDimension;
  ASEGMMWeightClusters mWeightClutser;
  ASEGMMMeanClusters mMeanClusters;
  ASEGMMCovCliusters mCovClusters;
};


enum class ASETreeNodeType {
  ASE_TREENODE_TYPE_ROOT,
  ASE_TREENODE_TYPE_CLUSTER
};

struct ASETreeNodeData {
  int mDimension;
  ASEGMMMeans mMeans;
  ASEGMMCovs mCovs;
  ASEGMMWeight mGMMWeight;
  
  ASETreeNodeData() = default;
  ASETreeNodeData(int dimension, ASEGMMWeight weight, ASEGMMMeans& means, ASEGMMCovs& covs) : mDimension(dimension), mGMMWeight(weight), mMeans(means), mCovs(covs) {}
  ASETreeNodeData(const ASETreeNodeData& data) = default;
};

using ASETreeNodeWeightKernelCallback = function<double(ASEFeatureSegment&, ASETreeNodeData&)>;

class ASETreeNode
{
public:
  ASETreeNode(int level, int order, int subNum, ASETreeNodeType nodeType);
  ASETreeNode(const ASETreeNode& node);
  ~ASETreeNode() {}

  friend ostream& operator << (ostream& out, ASETreeNode& treeNode);

  inline void AddNextBrother(ASETreeNodePtr brother) {
    mNext = brother;
  }

  inline void SetOrder(int order) {
    mNodeOrder = order;
  }

  inline int GetOrder() {
    return mNodeOrder;
  }

  inline int GetLevel() {
    return mNodeLevel;
  }

  inline int GetSubNum() {
    return mSubNum;
  }

  inline ASETreeNodeData& GetData() {
    return mData;
  }

  inline void SetData(ASETreeNodeData&& data) {
    mData = move(data);
  }

  inline double GetWeight() {
    return mWeight;
  }

  inline void SetWeight(double weight) {
    mWeight = weight;
  }

  inline double GetTempWeight() {
    return mTempWeight;
  }

  inline void SetTempWeight(double tempWeight) {
    mTempWeight = tempWeight;
  }

  inline ASETreeNodePtr GetParent() {
    return mParent;
  }

  inline ASETreeNodePtr GetNext() {
    return mNext;
  }

  inline void SetNext(ASETreeNodePtr next) {
    mNext = next;
  }

  inline ASETreeNodeChildren GetChildren() {
    return mChildren;
  }

  inline bool IsSubHead() {
    if (mParent != nullptr && mParent->mChildren.size() > 0) {
      return this == mParent->mChildren[0];
    }
    return false;
  }

  inline bool IsSubTail() {
    if (mParent != nullptr && mParent->mChildren.size() > 0){
      return mParent->mChildren[mParent->mChildren.size() - 1] == this;
    }
    return false;
  }

  inline void ResizeThreadWeights(int threadNum) {
    if (mNodeType == ASETreeNodeType::ASE_TREENODE_TYPE_ROOT) {
      mThreadWeights = ASETreeNodeWeightsForMultiThread(threadNum, 1);
      mThreadTempWeights = ASETreeNodeTempWeightsForMultiThread(threadNum, 1);
    }
    else {
      mThreadWeights = ASETreeNodeWeightsForMultiThread(threadNum, 0);
      mThreadTempWeights = ASETreeNodeTempWeightsForMultiThread(threadNum, 0);
    }
  }

  inline void SetThreadTempWeight(int threadID, double tempWeight) {
    if (threadID >= mThreadTempWeights.size()) {
      return;
    }
    mThreadTempWeights[threadID] = tempWeight;
  }

  inline void SetThreadWeight(int threadID, double weight) {
    if (threadID >= mThreadWeights.size()) {
      return;
    }
    mThreadWeights[threadID] = weight;
  }

  inline double GetThreadTempWeight(int threadID) {
    if (threadID >= mThreadTempWeights.size()) {
      return -1;
    }
    return mThreadTempWeights[threadID];
  }

  inline double GetThreadWeight(int threadID) {
    if (threadID >= mThreadWeights.size()) {
      return -1;
    }
    return mThreadWeights[threadID];
  }

  void ComputeWeightsForMultiThread();

  void AddChildren(ASETreeNodePtr treeNode);

  // double ComputeWeight(double value, ASETreeNodePtr brother);

private:
  int mNodeLevel;
  int mSubNum;
  int mNodeOrder;
  double mWeight;
  double mTempWeight;
  ASETreeNodeWeightsForMultiThread mThreadWeights;
  ASETreeNodeTempWeightsForMultiThread mThreadTempWeights;

  ASETreeNodeData mData;
  ASETreeNodeType mNodeType;
  ASETreeNodePtr mParent;
  ASETreeNodePtr mNext;
  ASETreeNodeChildren mChildren;
};


class ASETree
{
public:
  ASETree();
  ASETree(const ASETree& tree) = delete;
  ~ASETree();

  friend ostream& operator << (ostream& out, ASETree& tree);

  inline void SetPurningParam(bool needPurining, double purningRate) {
    mNeedPurning = needPurining;
    mPurningRate = purningRate;
  }

  ASEDescriptor GetDescriptorFromTree(int level);
  ASEDescriptor GetDescriptorFromTree();

  void AddTreeLevel(ASEGMMClusters& clusters);

  void GetWeightsFromOneFeature(ASEFeature& feature, ASETreeNodeWeights& weights, ASETreeNodeWeightKernelCallback kernelCallback);

  void ReserveSpaceForMultiThread(int threads);

  void UpdateTreeNodeWeightsByPruningNodeWithDeep(ASETreeNodePtr node, ASETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeWeightsByPruningNodeWithLevel(ASETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeThreadWeightsByPruningNodeWithLevel(int threadID,ASETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeWeightsStrictly(ASETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeThreadWeightsStrictly(int threadID, ASETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeWeights(ASEFeatureSet& normFeatures, ASETreeNodeWeightKernelCallback kernelCallback);

  void UpdateTreeNodeWeightsWithMultiThreads(ASEFeatureSet& normFeatures, ASETreeNodeWeightKernelCallback kernelCallback);

  void UpdateTreeNodeWeightsInOneThreadKernel(int threadID, ASEFeatureSet& normFeatures, int start, int end,  ASETreeNodeWeightKernelCallback kernelCallback);

  inline int GetTreeLevel() {
    return mTreeLevel;
  }

  int GetDescriptorSize(int level);

  void ResetWeightsOfTree();

private:

  int mTreeLevel;
  ASETreeNodePtr mRoot;
  ASETreeLevelHeads mHeads;
  ASETreeLevelSubNums mLevelSubNums;
  ASETreeLevelNums mLevelNums;

  bool mNeedPurning;
  double mPurningRate;
};

ASE_NAMESPACE_END

#endif