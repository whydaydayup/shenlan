2.实际调用

CV_EXPORTS_W bool solvePnP( InputArray objectPoints, InputArray imagePoints,
                            InputArray cameraMatrix, InputArray distCoeffs,
                            OutputArray rvec, OutputArray tvec,
                            bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE );
参数说明 

| 参数变量名        | 类型                      | 说明                     |
| ----------------- | ------------------------- | ------------------------ |
| objectPoints      | std::vector\<cv::Point3d> | 特征点物理坐标           |
| imagePoints       | std::vector\<cv::Point2d> | 特征点图像坐标           |
| cameraMatrix      | cv::Mat(3, 3, CV_32FC1）  | 相机内参：3*3的float矩阵 |
| distCoeffs        | cv::Mat(1, 5, CV_32FC1)   | 相机畸变参数             |
| rvec              |                           | 输出的旋转向量           |
| tvec              |                           | 输出的平移向量           |
| useExtrinsicGuess |                           |                          |
| flags             |                           | 计算方法                 |

enum { SOLVEPNP_ITERATIVE = 0,
       SOLVEPNP_EPNP      = 1, //!< EPnP: Efficient Perspective-n-Point Camera Pose Estimation @cite lepetit2009epnp
       SOLVEPNP_P3P       = 2, //!< Complete Solution Classification for the Perspective-Three-Point Problem 
       SOLVEPNP_DLS       = 3, //!< A Direct Least-Squares (DLS) Method for PnP  @cite hesch2011direct
       SOLVEPNP_UPNP      = 4, //!< Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation 
       SOLVEPNP_AP3P      = 5, //!< An Efficient Algebraic Solution to the Perspective-Three-Point Problem 
       SOLVEPNP_MAX_COUNT      //!< Used for count
};



## ***warning：***

切记，虽然Opencv的参数偏爱float类型，但是solvepnp中除了相机内参和畸变参数矩阵是用float类型外，其余的矩阵都是double类型，不然出出现计算结果不正确的情况。
  

P3P算法要求输入的控制点个数只能是4对。

```c++
//三种方法求解
solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_ITERATIVE);    //实测迭代法似乎只能用4个共面特征点求解，5个点或非共面4点解不出正确的解
cv::solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_P3P);            //Gao的方法可以使用任意四个特征点，特征点数量不能少于4也不能多于4
solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);         //该方法可以用于N点位姿估计;与前两种有偏差

    //旋转向量变旋转矩阵
    //提取旋转矩阵
    double rm[9];
    cv::Mat rotM(3, 3, CV_64FC1, rm);
    Rodrigues(rvec, rotM);
```