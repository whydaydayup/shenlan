运行时roscore执行
rosrun imlsMatcher ..



libnabo一个快速为低维度空间提供K最近邻居算法库。它提供了一个干净的,传统的无标量类型无关的C ++模板API。

有一个float点集point set `M` , 需要查询点a query point `q`, 可以找到在点集`M`中查询到点`q`的the `K` nearest neighbours:

```
#include "nabo/nabo.h"
using namespace Nabo;
using namespace Eigen;
...
NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);

const int K = 5;
VectorXi indices(K);
VectorXf dists2(K);

nns->knn(q, indices, dists2, K);
```

* 上面例子中, 
    * `M` is an Eigen matrix (column major, float)
    *  `q` 是一个Eigen vector (float).
    * 注意:  `M`必须在整个使用libnabo的过程中存在,  **must stay alive** , 否则otherwise `knn` 的结果是未定义的. 结果 `indices` 和 `dists2` 是表示 对应于`M`列的索引和相应平方距离的Eigen vectors,
    * `examples/trivial.cpp` 此例子的可编译版本, 
    * `examples/usage.cpp` 包含多点查询multi-point的更复杂的例子.

* 在build目录运行 `make doc` 生成可浏览的文档`doc/html`. 主页`doc/html/index.html` 包含更详细的使用方法libnabo.

```c++
mkdir ~/lib/
cd ~/lib
git clone git://github.com/ethz-asl/libnabo.git
cd libnabo
SRC_DIR=`pwd` //记录当前路径
BUILD_DIR=${SRC_DIR}/build //声明编译路径
mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR}　//创建编译路径
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ${SRC_DIR}　//编译类型及路径
make //编译
sudo make install //安装
```

