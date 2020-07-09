退化场景的优化论文On dependence 





在提取特征点的时候，每条线的每个分区内只选取2个边缘点和4个平面点

 论文里说为了采样均匀，另一方面我认为是为了配准时搜索量小一些 ???

 loam在运行比较大的图的时候，后面地图在出现的时候，前面的地图会被刷新截掉，重建的图像边缘在不断丢失。应该怎么修改可以让它显示大图呢，Rviz界面里最下面的一个pointcloud2设置Decay Time调大就可以。

PCL求曲率

 https://blog.csdn.net/GoodLi199309/article/details/80537310 



https://blog.csdn.net/liuyanpeng12333/article/details/82737181 

https://blog.csdn.net/weixin_43211438/article/list/2

https://blog.csdn.net/qq_21842097/article/details/80714483

https://blog.csdn.net/OORRANNGGE/article/details/85624782

https://zhuanlan.zhihu.com/c_131391131

https://blog.csdn.net/xiaoma_bk/article/list/3

LOAM不以常规的3d icp策略而以3d雷达点云特征点为主要匹配方式结合IMU，实现实时地计算高精度的6自由度激光雷达里程计，在此基础上再结合3d icp得到高精度地图点云数据。

loam_velodyne包原始作者是针对vlp-16激光雷达设置的，测试发现它对其他平台的IMU数据兼容性不好，

测试结果发现因为没有IMU，loam_velodyne对剧烈晃动、快速转向适应性差，为了得到好的结果，需要保证运动的平稳、匀速。同时雷达输出点云的速率提升到20HZ后，slam效果也有提高。20hz的设置需要使用windows配置软件设置。



LOAM的整体思想就是将复杂的SLAM问题分为：1. 高频的运动估计； 2. 低频的环境建图。

Lidar接收数据，首先进行Point Cloud Registration，Lidar Odometry以10Hz的频率进行运动估计和坐标转换，Lidar Mapping以1Hz的频率构建三维地图，Transform Integration完成位姿的优化。这样并行的结构保证了系统的实时性。

laserOdometry从特征点中估计运动（10HZ），然后整合数据发送给laserMapping（1HZ），在这个过程中，算是一种数据的降采样吧。也是为了保证运行online的时效性，以及节省内存占用的大小。

相对于其它直接匹配两个点云，loam是通过提取特征点匹配后计算坐标变换。



PointCloudRegistration
1.对每一帧点云

构建低漂移的里程计，
作者想解决的问题： 构建低漂移的里程计。论文完成的工作是使用3D激光雷达构建出的里程计构来建三维点云地图。 但是作者的目标是缩小激光里程计的漂移和计算量，重点并不是构建出三维地图，所以作者没有考虑回环问题（loop closure）。

作者实现的方法 ： 作者的思路同SLAM（simulanteous localization and mapping）的思路一致，算法思路如下图

![image-20191024182615195](/home/riki/.config/Typora/typora-user-images/image-20191024182615195.png)

主要经过以下几个步骤

1. Point Cloud Registration - 接收激光数据，提取特征点（边和面）并排除瑕点。
2. Lidar Odometry - 通过特征点云的匹配估计运动以10hz频率发布一个里程计。此时的里程计精度并不高。
3. Lidar Mapping - 接收里程计信息和当前点云数据，基于粗匹配的里程计位姿进行精匹配，根据估计出的位姿将点云注册到地图中，并按照1hz发布更新后的位姿。输出的laser_cloud_surround为地图
4. Transform Integration - 接收 lidar odometry 和 lidar mapping 发布的位姿Odometry信息，对位姿进行融合优化, 以10hz 的频率发布里程计信息。

如何选取特征点 边点（edge point）和平面点 （planar point） 
作者首先定义了一个衡量点平滑度的公式 , 通过测量点两边的距离差异，来衡量该点的平滑程度。c值最大的点将作为边点而cz值最小的点为平面点。 将360度分为四个区域，每个区域最多产生两个边点和四个平面点。这些边点和平面点如下图所示。
这些提取出来的点作为特征点来完成前后帧的匹配，来估计自身的位姿。
构建损失函数 特征点选取后， 根据 tf 关系转换到 map 坐标系下，在map 中寻找最近的 边点和平面点, 将两个最近的边点和平面点当成对应点构建一个转换矩阵，（t_x, t_y, t_z, theta)， 对该转换举证求雅克比，使用高斯牛顿法缩小损失函数直至收敛。

![image-20191103123349693](/home/riki/.config/Typora/typora-user-images/image-20191103123349693.png)

根据点的曲率c来将点划分为不同的类别（边/面特征或不是特征）

![image-20191025093049327](/home/riki/.config/Typora/typora-user-images/image-20191025093049327.png)

最大的c值,称为edge points, 最小的c值,称为planar points.



15,17,19

1 > 15+2.5=17.5

17,15



 (y0-y1)*(z0-z2)i+j(z0-z1)(x0-x2)+k(x0-x1)(y0-y2)-k(x0-x2)(y0-y1)-i(y0-y2)(z0-z1)-j(xx0-x1)(z0-z2)



imuHandler（） 
	1. 减去重力对imu的影响,
	2. 解析出当前时刻的imu时间戳，角度以及各个轴的加速度
	3. 将加速度转换到世界坐标系轴下,
	4. 进行航距推算，假定为匀速运动(没有IMU时为匀速运动,有IMU时假定为匀加速运动)推算出当前时刻的位置
	5. 推算当前时刻的速度信息
cartographer中IMU用于提供位姿的先验信息。但LOAM中并没有使用IMU提供下一时刻位姿的先验信息，而是利用IMU去除激光传感器在运动过程中非匀速（加减速）部分造成的误差（运动畸变）。因为LOAM是基于匀速运动的假设，但实际中激光传感器的运动肯定不是匀速的，因此使用IMU来去除非匀速运动部分造成的误差，以满足匀速运动的假设。

IMU的数据可以提供给我们IMU坐标系三个轴相对于世界坐标系的欧拉角和三个轴上的加速度。但由于加速度受到重力的影响所以首先得去除重力影响。在去除重力影响后我们想要获得IMU在世界坐标系下的运动，因此根据欧拉角就可以将IMU三轴上的加速度转换到世界坐标系下的加速度。 然后根据加速度利用公式s1 = s2+ vt + 1/2at\*t来计算位移。因此我们可以求出每一帧IMU数据在世界坐标系下对应的位移和速度。

AccumulateIMUShift()函数主要用于获取每一帧IMU数据对应IMU(/camera坐标系)在全局世界坐标系下的位移和速度。

```
transform point to the global frame 由激光雷达坐标系(点云当前时刻坐标系下) 转换到 世界坐标系下
R = Ry(yaw)*Rx(pitch)*Rz(roll)
Ry(yaw)*Rx(pitch)*Rz(roll)*Pl

transfrom global points to the local frame 由世界坐标系 转换到 激光雷达坐标系下(点云数据开始第一点的坐标系下)
R.inverse = Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse
Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * Pg
```



预处理：读取点云后，先计算激光雷达的起始角度和终止角度，这里有一点需要注意一下，激光雷达一个扫描周期扫过的角度不一定刚好是360度。起始角为一帧点云数据中第一个点的水平角，终止角为最后一个点的水平角。

去除点云中的极小值（坐标接近于0），对点云的坐标顺序进行调整，将坐标系改成：向左—x轴，向上—y轴，向前—z轴，根据垂直角计算每个点在第几条扫描线上（scanID），然后根据数据点的水平角计算该点在一个扫描周期中的相对时间，公式为：
$$RelativeTime = ScanPeriod\times\frac{HorizontalAngle-StartAngle}{EndAngle-StartAngle}$$
如果有IMU的话，借助IMU数据把数据点投影到开始扫描的位置。然后把调整过的点云按照各自所在的扫描线存入一个vector中，vector的大小为激光雷达的线数。这样一来就得到了一堆分布有序的点云。



(2)提取特征：loam把点云的特征点根据曲率分为两类——角点和平面点。计算曲率的算法跟论文里的公式有一点出入。论文里计算曲率的公式为：

![image-20191103135856139](/home/riki/.config/Typora/typora-user-images/image-20191103135856139.png)


而代码里用的是公式求和后向量的平方和。计算曲率时，为了让特征点能够尽量均匀地分布，把每条扫描线分成若干个区域，默认分成6个区域，在每个区域内取2个角点，4个平面点。计算曲率时，在这个点的左边取n个点，右边取n个点作为领域，默认的n = 5。然后对当前特征区域内的点根据曲率排序，取曲率最大的2个点作为角点，曲率最小的4个点作为平面点，存到各自的点云变量中。






laserCloudHandler（）函数

1. 从话题/velodyne_points中获得sensor_msgs::PointCloud2消息的点云数据，并将这些点云中的点按照其空间中与Z轴成的角度将其分类到16根激光线束中并记录线束号和获取的相对时间（也就是代码中的intensity，这个不是强度）；
2. 就是提取特征，遍历每个线束上的点，求该点的曲率，通过曲率将点进行特征点分类；
3. 根据IMU获得的数据去除激光传感器加减速造成的误差（也就是运动畸变）。

函数ShiftToStartIMU(pointTime)
知道了每个点在世界坐标系下的位置，求这一帧数据中该点相对于起始点由于加减速造成的运动畸变。因此首先要求出世界坐标系下的加减速造成的运动畸变，然后将运动畸变值经过绕y、x、z轴旋转后得到起始点坐标系下(Lidar坐标系下)的运动畸变。这里的坐标系一定要搞清楚为什么要放的起始点的坐标系下。
VeloToStartIMU()函数，作用是求当前点的速度相对于点云起始点的速度畸变，先计算全局坐标系下的然后再转换到起始点的坐标系中。 
TransformToStartIMU(PointType *p)函数作用是将当前点先转换到世界坐标系下然后再由世界坐标转换到点云起始点坐标系下。 然后减去加减速造成的非匀速畸变的值。

```
// 对所有的激光点一个一个求出在该点前后5个点(10点)的偏差，作为cloudCurvature点云数据的曲率
```

laserCloud有效点: 过滤了无效点(NaN点),N_SCANS在0~15之间的全部点,按照线号排列的点,消除了运动畸变(无IMU匀速运动模型, 有IMU匀加速运动模型)   /velodyne_cloud_2

​		// 如果有IMU的话，借助IMU数据把数据点投影到开始扫描的位置。

特征点Label的判断:
	2-曲率很大cornerPointsSharp，
	1-曲率比较大cornerPointsLessSharp, 
	-1-曲率很小surfPointsFlat，
	0-曲率比较小surfPointsLessFlat
	(其中1包含了2, 0包含了-1, 0和1构成了点云全部的点)
使用前后5点来计算当前点的曲率,只有同一个SCAN计算的曲率才合法
去除的第一种点是不能与激光线接近于平行的点(这些点在这一帧数据 中可以看到，但是下一帧数据可能就看不到了 也就没有 办法匹配了), 去除第二类的点是可能会遮挡的点(可能也会在下一帧数据中看不到)。

代码中，实际把每个线(总共16线)分为6段进行处理，起始位置为sp,终止位置为ep. 每段都将曲率按照升序排列。 每一段中有2个曲率很大Label=2的边沿点,20个曲率大Label=1的边沿点,   4个曲率很小Label=-1的平面点, 剩下的全是比较小Label=0的平面点
同时为了防止特征点聚集，使得特征点在每个方向上尽量分布均匀, 将曲率1曲率比较大和 -1曲率很小的点 的前后各5个连续距离比较近(距离的平方小于0.05)的点筛选出去, 不作为特征点
将所得的曲率较小Label=0的点(less flat)点最多，对每个分段less flat的点进行体素栅格Voxel滤波,保存到surfPointsLessFlat

这里的/camera坐标系就是Lidar坐标系
imuTrans发送的是
	1.  一帧点云数据开始第一个点的欧拉角
	2.  一帧点云数据中最后一个点的欧拉角
	3.  最后一个点相对于第一个点的畸变位移
	4.  最后一个点相对于第一个点的畸变速度

![image-20191103123835212](/home/riki/.config/Typora/typora-user-images/image-20191103123835212.png)

https://blog.csdn.net/weixin_43211438/article/details/88600683

激光扫描模型
首先介绍一下VLP16的激光扫描模型：
论文中称单个线束为一个Scan, 对全部16线组成的一帧点云称为一个Sweep,
虽然是用的多线激光雷达,但是LOAM是针对单个Scan提取特征点的,这里主要考虑到线束间角分辨率(竖直分辨率)与单个线内点间角分辨率(水平分辨率)存在的差异.
角分辨率越大, 代表越远的物体, 反射的两点距离越大, 中间丢失的信息越多.因此, LOAM没有针对Scan和Scan之间的点的关联性提取和描述特征, 而是直接针对单个Scan提取特征.
要读懂代码中特征提取中的一些处理, 需要弄清楚VLP16扫描时的运动模型,简单的总结为:
一帧内所有的点, 都是按顺序穿行扫描的, 同一个时间点,只会有一次发送,紧接着一次接收。先从水平第一个角度,一般在0°左右,扫描这个水平角度上竖直方向所有16个点(对应16个SCAN)的深度,当然这16个点也是串行按顺序的,然后转到下一个水平角度, 比如0.3°开始, 水平分辨率0.4°,那么下个角度就是0.7°,然后1.1°.一直顺时针扫完一圈, 完成一个Sweep数据的采集.当然， Velodyne的User Manual里面讲的更清楚。
由于从驱动得到的一个Sweep是以点云的形式输出（也就是一堆点，每个点有XYZI的信息，点和点之间无其他关系信息）, 因此我们并不知道每个点属于哪个Scan, 对应哪个水平角度,因此, 我们需要根据上面的扫描模型去计算每个点的竖直角度和水平角度.

根据VLP16的激光扫描模型, 对单帧点云（paper中称为一个Sweep）进行分线束（分为16束）, 每束称为一个Scan， 并记录每个点所属线束和每个点在此帧电云内的相对扫描时间（相对于本帧第一个点）。
针对单个Scan提取特征点, 而相对时间会在laserOdometry中用于运动补偿.所有Scan的特征点,拼到两个点云中(因为是corner和surface两种特征点,所以是两个点云).至此,每帧点云,输出两种对应的特征点云, 给下一个节点laserOdometry。

角点（cornerPoints）和平面点（surfacePoints）







注解：
代码流程：订阅了2个节点和发布了6个节点。通过回调函数的处理，将处理后的点云重新发出去。

功能：对点云和IMU数据进行预处理，用于特征点的配准。

具体实现：一次扫描的点通过曲率值来分类，特征点曲率大于阈值的为边缘点；特征点曲率小于阈值的为平面点。为了使特征点均匀的分布在环境中，将一次扫描划分为4个独立的子区域。每个子区域最多提供2个边缘点和4个平面点。此外，将不稳定的特征点（瑕点）排除。

在ROS中点云的数据类型

在ROS中表示点云的数据结构有： sensor_msgs::PointCloud      sensor_msgs::PointCloud2     pcl::PointCloud<T>

关于PCL在ros的数据的结构，具体的介绍可查 看            wiki.ros.org/pcl/Overview

关于sensor_msgs::PointCloud2   和  pcl::PointCloud<T>之间的转换使用pcl::fromROSMsg 和 pcl::toROSMsg 

sensor_msgs::PointCloud   和   sensor_msgs::PointCloud2之间的转换

使用sensor_msgs::convertPointCloud2ToPointCloud 和sensor_msgs::convertPointCloudToPointCloud2.





Lidar coordinate system {L} is a 3D coordinate system with its origin at the geometric center of the lidar. The x-axis is pointing to the left, the y-axis is pointing upward, and the z-axis is pointing forward.

LaserOdometry进行点云数据配准，完成运动估计

laserCloudCornerLast, laserCloudSurfLast表示的是上一时刻t-1的less sharp,less flat point
cornerPointsSharp, surfPointsFlat表示的是当前t时刻的Sharp, flat point

将实时接收到的特征点(Sharp point, flat point)数据作为 t 时刻的特征点,  在保存的上一帧 t-1 点云中特征点(less Sharp,less flat)中寻找到匹配的点



https://blog.csdn.net/xiaoma_bk/article/details/83758796

分为三步：初始化，点云处理，坐标转化

1.检测系统是否初始化，如果未初始化时，进行初始化

激光的匹配涉及到两组数据，因此初始化相当于首先获取将一组数据。

2.在点云足够多的条件下，开始正常工作。这里我们设定整个L-M运动估计的迭代次数为25次，以保证运算效率。迭代部分又可分为：对特征边/面上的点进行处理，构建Jaccobian矩阵，L-M运动估计求解。

L-M方法其实就是非线性最小二乘，是Gauss-Newton优化的一种改进（增加了一个阻尼因子，代码中的s），所以关键在于如何把点云配准和运动估计的问题转换为L-M优化求解的问题。

主要思路就是：构建约束方程 -> 约束方程求偏导构建Jaccobian矩阵 -> L-M求解。

下面再一步一步来看：关于构建约束方程的问题就是这节标题中提到的点云配准的问题，其基本思想就是从上一帧点云中找到一些边/面特征点，在当前帧点云中同样找这么一些点，建立他们之间的约束关系。
		找t+1时刻的某个特征边/面上的点在t时刻下对应的配准点，论文作者给出如上图的思路。

​		特征线：利用KD树找点i在t时刻点云中最近的一点j，并在j周围（上下几条线的范围内）找次近点l，于是我们把（j，l）称为点i在t时刻点云中的对应。

​		特征面：与特征线类似，先找最近点j，在j周围找l，在j周围找m，将（j，l，m）称为点i在t时刻点云中的对应。

3.特征线上的点配准：

1)将点坐标转换到起始点云坐标系中

2)每迭代五次,搜索一次最近点和次临近点(降采样)

(计算量的一种平衡吧，每次更新transform后都更新一次最近匹配计算资源消耗大)

if (iterCount % 5 == 0){下面代码}

3)找到pointSel(当前时刻边特征中的某一点)在laserCloudCornerLast中的1个最邻近点 

   pointSearchInd(对应点的索引) , pointSearchSqDis:(pointSel与对应点的欧氏距离)

4)如果最临近的点距离小于25时，计算出  搜索到的点 所在线数 （closestPointScan 上一时刻最近点所在的线数）

5)从找得到的最邻近点开始，遍历所有边特征点,找到与最邻近点相距3条线的特征点时跳出

6)向下三条线，找最近点和次临近点

7)获得了临近点和次临近点

当前所有边特征点在上一时刻边特征点云中对应的最邻近点的索引

_pointSearchCornerInd1[i] = closestPointInd;
当前所有边特征点在上一时刻边特征点云中对应的次邻近点的索引

_pointSearchCornerInd2[i] = minPointInd2;

4.特征线的构建Jaccobian矩阵

1)当前点云坐标pointSel，上一时刻临近点坐标tripod1，上一时刻次次临近点坐标tripod2

2）点到直线的距离中$d\varepsilon =\frac{\left | \left ( \tilde{x}_{\left (k+1,i \right )}^{L} - \bar{x}_{\left (k,j \right )}^{L} \right ) \times \left ( \tilde{x}_{\left (k+1,i \right )}^{L} - \bar{x}_{\left (k,l \right )}^{L} \right ) \right |}{ \left | \bar{x}_{\left (k+1,j \right )}^{L} - \bar{x}_{\left (k,j \right )}^{L} \right | }$

$\left | \left ( \tilde{x}_{\left (k+1,i \right )}^{L} - \bar{x}_{\left (k,j \right )}^{L} \right ) \times \left ( \tilde{x}_{\left (k+1,i \right )}^{L} - \bar{x}_{\left (k,l \right )}^{L} \right ) \right |$   ，实际就乘上一个两向量夹角的正弦值     分别作差并叉乘后的向量模长

3)求两点之间的模长$\left | \bar{x}_{\left (k+1,j \right )}^{L} - \bar{x}_{\left (k,j \right )}^{L} \right |$

3)向量[la；lb；lc] 为距离ld2分别对x0 y0 z0的偏导  transform的偏导

4)计算点到直线的距离，以及偏差

5)定义阻尼因子，点到直线距离越小阻尼因子越大 ，计算出偏差 （xyz强度）

6)满足阈值(ld2 < 0.5)，将特征点插入



5.特征面上点配准

1)遍历平面特征点，pointSel(当前时刻边特征中的某一点)

2)每迭代五次,搜索一次最近点和次临近点(降采样)

3)找到pointSel(当前时刻边特征中的某一点)在laserCloudCornerLast中的1个最邻近点 

   pointSearchInd(对应点的索引) , pointSearchSqDis:(pointSel与对应点的欧氏距离)

4)如果最临近的点距离小于25时，计算出  搜索到的点 所在线数 （closestPointScan 上一时刻最近点所在的线数）

5)从找得到的最邻近点开始，遍历所有边特征点,找到与最邻近点相距3条线的特征点时跳出

6)向下三条线

7)确定目标点



6.特征面的构建Jaccobian矩阵   点到面的过程

每个特征点对应的Jaccobian矩阵的三个元素都保存在coeffSel中，后面采用L-M方法解算的时候直接调用就行了。



7.L-M运动估计求解

假设：雷达的运动是连续的。将所有对应到的点求到直线的距离到面的距离之和最短然后按照Levenberg-Marquardt算法迭代计算，得到两帧之间的变换，最后通过累计计算odom。

公式（1,2,3）特征提取与计算误差
我们认为Lidar匀速运动，因此公式(4)实现了Lidar变换矩阵的内插，得到其扫描点i时刻的位姿
公式(5)就是经典的刚体的旋转平移变换。
公式(6)为罗德里格斯公式，将旋转矩阵用旋转矢量表示，这种方法的最大优势在于节约计算所消耗的资源。
公式(7)-(8)分别为旋转矢量公式中的参数计算。搞清楚这些以后，就开始L-M的计算了;

1)匹配到的点的个数(即存在多少个约束)

2)遍历每个对应点，并且构建Jaccobian矩阵

3)采用Levenberg-Marquardt计算 ，首先建立当前时刻Lidar坐标系下提取到的特征点与点到直线/平面的约束方程。而后对约束方程求对坐标变换(3旋转+3平移)的偏导。公式参见论文(2)-(8)

pointOri 当前时刻点坐标，coeff 该点所对应的偏导数

4)具体构建某一点的矩阵分许：  arx ary arz 偏导数

5)最小二乘计算(QR分解法)

 8.坐标转换

算出了点云间的相对运动，但他们是在这两帧点云的局部坐标系下的，我们需要把它转换到世界坐标系下，因此需要进行转换

https://blog.csdn.net/xuewend/article/details/84645213

```
绕z轴旋转rz
x1 = cos(rz) * x - sin(rz) * y;
y1 = sin(rz) * x + cos(rz) * y;
z1 = z
绕x轴旋转rx
x2 = x1
y2 = cos(rx) * y1 - sin(rx) * z1
z2 = sin(rx) * y1 + cos(rx) * z1
绕y轴旋转ry
x3 = cos(ry) * x2 + sin(ry) * z2
y3 = y2
z3 = -sin(ry) * x2 + cos(ry) * z2
```

比较坑的是作者用Axis-Angle形式的旋转向量作为优化变量，用欧拉角来旋转，多了一些不必要的数据转换。

运动估计:

整个优化算法并不是把所有的距离项最小化就结束了。这时候还不能保证当前帧的边缘点和平面点已经成功对应上了上一帧中的边缘和平面，每迭代若干次，都要重新用KDTree搜索最近点，然后构建新的误差项。在BasicLaserOdometry::process()函数内，每迭代5次都会重新寻找最近的边缘和平面块。

$[Math Processing Error]x_{0}^{\prime}=x-s t_{x}$
$[Math Processing Error]y_{0}^{\prime}=y-s t_{y}$
$[Math Processing Error]z_{0}^{\prime}=z-s t_{z}$

$[Math Processing Error]\begin{aligned} R_{X} &=\left[\begin{array}{ccc}{1} & {0} & {0} \\ {0} & {\cos \theta} & {-\sin \theta} \\ {0} & {\sin \theta} & {\cos \theta}\end{array}\right] \\ R_{Y} &=\left[\begin{array}{ccc}{\cos \theta} & {0} & {\sin \theta} \\ {0} & {1} & {0} \\ {-\sin \theta} & {0} & {\cos \theta}\end{array}\right] \\ R_{Z}&=\left[\begin{array}{ccc}{\cos \theta} & {-\sin \theta} & {0} \\ {\sin \theta} & {\cos \theta} & {0} \\ {0} & {0} & {1}\end{array}\right] \end{aligned}$

```
//在右手系中绕X轴旋转p° 对应的矩阵Rx
        | 1    0        0 |
  Rx=   | 0   cosp   -sinp|
        | 0   sinp    cosp|
//在右手系中绕Y轴旋转h° 对应的矩阵Ry
        | cosh   0   sinh|
  Ry=   |  0     1     0 |
        |-sinh   0   cosh|
//在右手系中绕Z轴旋转b° 对应的矩阵Rz
        |cosb  -sinb   0 |
  Rz=   |sinb   cosb   0 |
        |  0     0     1 |
Z-X-Y顺规的欧拉角对应的组合旋转矩阵就是R = Rz*Rx*Ry
Z-X-Y顺规的欧拉角对应的组合旋转矩阵就是R = Rz*Rx*Z-X-Y顺规的欧拉角(30°，62°，28°)就是先绕Z轴旋转28°，然后绕X轴旋转30°，最后绕Y轴旋转62°。


左手系下的基础旋转矩阵
基础旋转矩阵的差异体现在sin值的取正负的问题上。仔细分析，这是可以理解的。在右手坐标系下绕X轴旋转p°，相当于右手系中旋转-p°，而sin(-p) = - sin(p) cos(p) = cos(-p)，所以有下面的差异。

//在右手系中绕X轴旋转p° 对应的矩阵Rx
        | 1     0        0 |
  Rx=   | 0    cosp    sinp|
        | 0   -sinp    cosp|
//在右手系中绕Y轴旋转h° 对应的矩阵Ry
        | cosh   0   -sinh|
  Ry=   |  0     1      0 |
        | sinh   0    cosh|
//在右手系中绕Z轴旋转b° 对应的矩阵Rz
        |cosb    sinb   0 |
  Rz=   |-sinb   cosb   0 |
        |  0      0     1 |
```



注解：
功能：优化Lidar的位姿，在此基础上完成低频的环境建图

解释：经过前两个节点的处理可以完成一个完整激光里程计，可以概略地估计出Lidar的相对运动，可以直接利用估计的Lidar位姿和对应时刻的量测值完成建图。但测量噪声是不可避免的，因此Lidar位姿估计偏差一定存在。通过与地图匹配的方式来优化激光的位姿，利用已构建地图对位姿估计结果进行修正。当前扫描的点云和地图中所有点云去配准，这个计算消耗太大，因此为了保证实时性，作者在这里采用了一种低频处理方法，即调用建图节点的频率仅为调用里程计节点频率的十分之一。

点云数据进来后，经过前两个节点的处理可以完成一个完整但粗糙的里程计，可以概略地估计出Lidar的相对运动。如果不受任何测量噪声的影响，这个运动估计的结果足够精确，没有任何漂移，那我们可以直接利用估计的Lidar位姿和对应时刻的量测值完成建图。但这就如同现实中不存在一个不受外力就能匀速直线运动的小球一样，量测噪声是不可避免的，因此Lidar位姿估计偏差一定存在。

Lidar里程计的结果不准确，拼起来的点也完全不成样子，且它会不断发散，因此误差也会越来越大。我们对特征的提取仅仅只是关注了他们的曲率，且点云中的点是离散的，无法保证上一帧的点在下一帧中仍会被扫到。因此，我们需要依靠别的方式去优化Lidar里程计的位姿估计精度。在SLAM领域，一般会采用与地图匹配的方式来优化这一结果。我们始终认为后一时刻的观测较前一时刻带有更多的误差，换而言之，我们更加信任前一时刻结果。因此我们对已经构建地图的信任程度远高于临帧点云配准后的Lidar运动估计。所以我们可以利用已构建地图对位姿估计结果进行修正。









局部点云（一段距离的MAP）存储在kd数里面，方便查找
较远距离的点云点被丢弃，为了保持kd树的紧凑性，kd树很大，搜索很麻烦
激光里程计10HZ，local map更新以1HZ采集点云，去地图中做match，需要提取的feature是之前的10倍

centerCubeI,J,K表示的是当前估计的雷达位姿所在的cube的中心点坐标，表示的是小Cube的索引
laserCloudCenWidth = 10; // 邻域宽度, cm为单位
laserCloudCenHeight = 5; // 邻域高度, cm为单位
laserCloudCenDepth = 10; // 邻域深度, cm为单位
transformTobeMapped估计出来的 世界坐标系下的Lidar坐标系
int centerCubeI = int( (transformTobeMapped[3] + 25.0) / 50.0 ) + laserCloudCenWidth; // laserCloudCenWidth = 10; // 邻域宽度, cm为单位
int centerCubeJ = int( (transformTobeMapped[4] + 25.0) / 50.0 ) + laserCloudCenHeight; // laserCloudCenHeight = 5; // 邻域高度
int centerCubeK = int( (transformTobeMapped[5] + 25.0) / 50.0 ) + laserCloudCenDepth; // laserCloudCenDepth = 10; // 邻域深度


// 为了在子cube的5*5*5的邻域内搜索来找到配准点!!
子cube的索引合法（0<centerCubeI<laserCloudWidth=21，0<centerCubeJ<laserCloudHeight=11，0<centerCubeK<laserCloudDepth=21）
CUBE_SIZE=50.0，CUBE_HALF=CUBE_SIZE/2
换算成实际比例的，在世界坐标系下的坐标,   世界坐标系下的 小cube 的中心的坐标
float centerX = 50.0 * (i - laserCloudCenWidth); // 10(laserCloudCenWidth会动态变化)
float centerY = 50.0 * (j - laserCloudCenHeight); // 5
float centerZ = 50.0 * (k - laserCloudCenDepth); // 10

先根据当前估计的雷达位姿transformTobeMapped计算得到centerCubeI,J,K（表示的是当前估计的雷达位姿所在的cube的中心点坐标（加了偏置laserCloudCen）），表示的是小Cube的索引
子CUBE的大小是50的立方体，
为了后面的在子CUBE的5*5*5的领域内寻找配准点，调整centerCubeI,J,K的大小，调整之后取值范围:3 < centerCubeI < 18， 3 < centerCubeJ < 8, 3 < centerCubeK < 18
5*5*5的区域内找匹配点（区域大小(5*50)^3的立方体大小？？）
大的区域21*11*21=4851个小的CUBE的区域，每个CUBE50*50*50的大小（即大的CUBE是21*50=1050？这是10米的大Cube？（边长为10m的立方体））
用点表示每个CUBE的中心点坐标

如果当前所在的CUBE处于下边界，表明地图向负方向延伸的可能性比较大，则循环移位，将数组中心点向上边界调整一个单位

//点云方块集合最大数量
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;//4851
laserCloudCornerArray, laserCloudSurfArray
//array都是以50米为单位的立方体地图，运行过程中会一直保存(有需要的话可考虑优化，只保存近邻的，或者直接数组开小一点)
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];保存了指向 pcl::PointCloud<PointType>类的实例的指针  的 数组, 每个类的实例是50大小的立方体CUBE, 每个类
指向PointCloud<pcl::PointXYZ>类对象的指针
pcl::PointCloud<PointType>类的一个实例保存一坨点云, 一坨点云包含很多个点



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl 是一个命名空间，跟std类似，PointCloud是类模板，<pcl::PointXYZ>是模板类实例化的类型，PointCloud<pcl::PointXYZ>就是一个实例化了的模板类，ptr是只能指针，相当于之前普通指针声明的*，cloud是指针变量，就是一个指向PointCloud<pcl::PointXYZ>类对象的指针，new pcl::PointCloud<pcl::PointXYZ>就是给了一个地址初始化指针

pcl::PointCloud<PointT>, 
	PointT点云中的单点的类型, pcl::PointXYZ只保存点的xyz坐标, data[3]=1.0f只是用来满足存储对齐, PointXYZI保存点的float x, y, z坐标和intensity;
类中包含成员: int width点云数据中的宽度,int height点云数据中的高度,
	std::vector points存储类型为PointT的点的向量, points[i].data[0]或points[i].x可访问点的x坐标值
	bool is_dense点云中的所有数据都是有限的(true),还是其中的一些点不是有限的,它们的XYZ值可能包含inf/NaN这样的值(false)
	Eigen::Vector4f sensor_origin_指定传感器的采集位姿（==origin/translation==）这个成员通常是可选的，并且在PCL的主流算法中用不到。
	Eigen::Quaternionf sensor_orientaion_指定传感器的采集位姿（方向）。这个成员通常是可选的，并且在PCL的主流算法中用不到。
