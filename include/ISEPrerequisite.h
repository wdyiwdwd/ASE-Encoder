#ifndef ISE_PREREQUISITE
#define ISE_PREREQUISITE

#include <iostream>
#include <vector>
#include <memory>
#include <locale>
#include <eigen3/Eigen/Core>
#include "ThreadPool.h"

#define ISE_NAMESPACE_START namespace ise {
#define ISE_NAMESPACE_END }

#define USING_NAMESPACE_STD using namespace std
#define USING_NAMESPACE_EIGEN using namespace Eigen
#define USING_NAMESPACE_ISE using namespace ise

#define ISE_MIN_DOUBLE std::numeric_limits<double>::min()
#define ISE_MAX_DOUBLE std::numeric_limits<double>::max()

#define ISE_REF(obj) std::ref(obj)

#define SAFE_DELETE(p) do {delete p; p = nullptr;} while(0)


ISE_NAMESPACE_START

using ISEFeature = std::vector<double>;;
using ISEFeatureSegment = std::vector<double>;
using ISEFeatureImportances = std::vector<double>;
using ISEFeatureSet = std::vector<ISEFeature>;
using ISEDescriptor = std::vector<double>;

using ISEThreadPool = ThreadPool;
using ISEThreadPoolPTR = ThreadPool*;
using ISEThreadReturnType = std::future<void>;
using ISEThreadReturnVector = std::vector<ISEThreadReturnType>;

class ISEUtils;

class ISEPrepocessor;
using ISEPrepocessorPtr = ISEPrepocessor *;

class ISEEncoder;
using ISEEncoderPtr = ISEEncoder *;

class ISETreeNode;
using ISETreeNodeSharedPtr = std::shared_ptr<ISETreeNode>;
using ISETreeNodePtr = ISETreeNode *;

class ISETree;
using ISETreeUniquePtr = std::unique_ptr<ISETree>;
using ISETreeSharedPtr = std::shared_ptr<ISETree>;
using ISETreePtr = ISETree *;

ISE_NAMESPACE_END

#endif
