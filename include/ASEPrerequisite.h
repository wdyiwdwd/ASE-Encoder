#ifndef ASE_PREREQUISITE
#define ASE_PREREQUISITE

#include <iostream>
#include <vector>
#include <memory>
#include <locale>
#include <eigen3/Eigen/Core>
#include "ThreadPool.h"

#define ASE_NAMESPACE_START namespace ase {
#define ASE_NAMESPACE_END }

#define USING_NAMESPACE_STD using namespace std
#define USING_NAMESPACE_EIGEN using namespace Eigen
#define USING_NAMESPACE_ASE using namespace ase

#define ASE_MIN_DOUBLE std::numeric_limits<double>::min()
#define ASE_MAX_DOUBLE std::numeric_limits<double>::max()

#define ASE_REF(obj) std::ref(obj)

#define SAFE_DELETE(p) do {delete p; p = nullptr;} while(0)


ASE_NAMESPACE_START

using ASEFeature = std::vector<double>;;
using ASEFeatureSegment = std::vector<double>;
using ASEFeatureImportances = std::vector<double>;
using ASEFeatureSet = std::vector<ASEFeature>;
using ASEDescriptor = std::vector<double>;

using ASEThreadPool = ThreadPool;
using ASEThreadPoolPTR = ThreadPool*;
using ASEThreadReturnType = std::future<void>;
using ASEThreadReturnVector = std::vector<ASEThreadReturnType>;

class ASEUtils;

class ASEPrepocessor;
using ASEPrepocessorPtr = ASEPrepocessor *;

class ASEEncoder;
using ASEEncoderPtr = ASEEncoder *;

class ASETreeNode;
using ASETreeNodeSharedPtr = std::shared_ptr<ASETreeNode>;
using ASETreeNodePtr = ASETreeNode *;

class ASETree;
using ASETreeUniquePtr = std::unique_ptr<ASETree>;
using ASETreeSharedPtr = std::shared_ptr<ASETree>;
using ASETreePtr = ASETree *;

ASE_NAMESPACE_END

#endif
