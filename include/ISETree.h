#ifndef ISE_TREE
#define ISE_TREE

#include "ISEPrerequisite.h"
#include <queue>
#include <functional>

USING_NAMESPACE_STD;

ISE_NAMESPACE_START

using ISETreeNodeWeightsForMultiThread = vector<double>;
using ISETreeNodeTempWeightsForMultiThread = vector<double>;

using ISETreeNodeChildren = vector<ISETreeNodePtr>;
using ISETreeLevelHeads = vector<ISETreeNodePtr>;
using ISETreeLevelNums = vector<int>;
using ISETreeLevelSubNums = vector<int>;

using ISETreeNodeLevelHeadWeights = vector<double>;
using ISETreeNodeWeights = vector<ISETreeNodeLevelHeadWeights>;
using ISETreeNodeTraverseQueue = queue<ISETreeNodePtr>;

using ISEGMMWeight = double;
using ISEGMMMeans = vector<double>;
using ISEGMMCovRow = vector<double>;
using ISEGMMCovs = vector<ISEGMMCovRow>;

using ISEGMMWeightClusters = vector<ISEGMMWeight>;
using ISEGMMMeanClusters = vector<ISEGMMMeans>;
using ISEGMMCovCliusters = vector<ISEGMMCovs>;

struct ISEGMMClusters
{
  int mClustersNum;
  int mFeatureDimension;
  ISEGMMWeightClusters mWeightClutser;
  ISEGMMMeanClusters mMeanClusters;
  ISEGMMCovCliusters mCovClusters;
};


enum class ISETreeNodeType {
  ISE_TREENODE_TYPE_ROOT,
  ISE_TREENODE_TYPE_CLUSTER
};

struct ISETreeNodeData {
  int mDimension;
  ISEGMMMeans mMeans;
  ISEGMMCovs mCovs;
  ISEGMMWeight mGMMWeight;
  
  ISETreeNodeData() = default;
  ISETreeNodeData(int dimension, ISEGMMWeight weight, ISEGMMMeans& means, ISEGMMCovs& covs) : mDimension(dimension), mGMMWeight(weight), mMeans(means), mCovs(covs) {}
  ISETreeNodeData(const ISETreeNodeData& data) = default;
};

using ISETreeNodeWeightKernelCallback = function<double(ISEFeatureSegment&, ISETreeNodeData&)>;

class ISETreeNode
{
public:
  ISETreeNode(int level, int order, int subNum, ISETreeNodeType nodeType);
  ISETreeNode(const ISETreeNode& node);
  ~ISETreeNode() {}

  friend ostream& operator << (ostream& out, ISETreeNode& treeNode);

  inline void AddNextBrother(ISETreeNodePtr brother) {
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

  inline ISETreeNodeData& GetData() {
    return mData;
  }

  inline void SetData(ISETreeNodeData&& data) {
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

  inline ISETreeNodePtr GetParent() {
    return mParent;
  }

  inline ISETreeNodePtr GetNext() {
    return mNext;
  }

  inline void SetNext(ISETreeNodePtr next) {
    mNext = next;
  }

  inline ISETreeNodeChildren GetChildren() {
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
    if (mNodeType == ISETreeNodeType::ISE_TREENODE_TYPE_ROOT) {
      mThreadWeights = ISETreeNodeWeightsForMultiThread(threadNum, 1);
      mThreadTempWeights = ISETreeNodeTempWeightsForMultiThread(threadNum, 1);
    }
    else {
      mThreadWeights = ISETreeNodeWeightsForMultiThread(threadNum, 0);
      mThreadTempWeights = ISETreeNodeTempWeightsForMultiThread(threadNum, 0);
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

  void AddChildren(ISETreeNodePtr treeNode);

  // double ComputeWeight(double value, ISETreeNodePtr brother);

private:
  int mNodeLevel;
  int mSubNum;
  int mNodeOrder;
  double mWeight;
  double mTempWeight;
  ISETreeNodeWeightsForMultiThread mThreadWeights;
  ISETreeNodeTempWeightsForMultiThread mThreadTempWeights;

  ISETreeNodeData mData;
  ISETreeNodeType mNodeType;
  ISETreeNodePtr mParent;
  ISETreeNodePtr mNext;
  ISETreeNodeChildren mChildren;
};


class ISETree
{
public:
  ISETree();
  ISETree(const ISETree& tree) = delete;
  ~ISETree();

  friend ostream& operator << (ostream& out, ISETree& tree);

  inline void SetPurningParam(bool needPurining, double purningRate) {
    mNeedPurning = needPurining;
    mPurningRate = purningRate;
  }

  ISEDescriptor GetDescriptorFromTree(int level);
  ISEDescriptor GetDescriptorFromTree();

  void AddTreeLevel(ISEGMMClusters& clusters);

  void GetWeightsFromOneFeature(ISEFeature& feature, ISETreeNodeWeights& weights, ISETreeNodeWeightKernelCallback kernelCallback);

  void ReserveSpaceForMultiThread(int threads);

  void UpdateTreeNodeWeightsByPruningNodeWithDeep(ISETreeNodePtr node, ISETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeWeightsByPruningNodeWithLevel(ISETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeThreadWeightsByPruningNodeWithLevel(int threadID,ISETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeWeightsStrictly(ISETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeThreadWeightsStrictly(int threadID, ISETreeNodeWeights& weights, int featureNumber);

  void UpdateTreeNodeWeights(ISEFeatureSet& normFeatures, ISETreeNodeWeightKernelCallback kernelCallback);

  void UpdateTreeNodeWeightsWithMultiThreads(ISEFeatureSet& normFeatures, ISETreeNodeWeightKernelCallback kernelCallback);

  void UpdateTreeNodeWeightsInOneThreadKernel(int threadID, ISEFeatureSet& normFeatures, int start, int end,  ISETreeNodeWeightKernelCallback kernelCallback);

  inline int GetTreeLevel() {
    return mTreeLevel;
  }

  int GetDescriptorSize(int level);

  void ResetWeightsOfTree();

private:

  int mTreeLevel;
  ISETreeNodePtr mRoot;
  ISETreeLevelHeads mHeads;
  ISETreeLevelSubNums mLevelSubNums;
  ISETreeLevelNums mLevelNums;

  bool mNeedPurning;
  double mPurningRate;
};

ISE_NAMESPACE_END

#endif