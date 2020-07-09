#include "imls_icp.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"

#include "pcl-1.7/pcl/io/pcd_io.h"

//pcl::visualization::CloudViewer g_cloudViewer("cloud_viewer");

class imlsDebug
{
public:
    imlsDebug()
    {

        m_FrameID = 0;

        m_laserscanSub = m_nh.subscribe("sick_scan",5,&imlsDebug::championLaserScanCallback,this);
    }

    void ConvertChampionLaserScanToEigenPointCloud(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg,
                                                   Eigen::Vector3d odomPose,
                                                   std::vector<Eigen::Vector2d>& eigen_pts)
    {
        //变换矩阵
        Eigen::Matrix3d Tmatrix;
        Tmatrix << std::cos(odomPose(2)),-std::sin(odomPose(2)),odomPose(0),
                   std::sin(odomPose(2)), std::cos(odomPose(2)),odomPose(1),
                   0,0,1;

        //转换到里程计坐标系中．
        eigen_pts.clear();
        for(int i = 0; i < msg->ranges.size();i++)
        {
            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;

            double lx = msg->ranges[i] * std::cos(msg->angles[i]);
            double ly = msg->ranges[i] * std::sin(msg->angles[i]);

            if(std::isnan(lx) || std::isinf(ly)||
               std::isnan(ly) || std::isinf(ly))
                continue;

            Eigen::Vector3d lpt(lx,ly,1.0);

            Eigen::Vector3d opt = Tmatrix * lpt;

            eigen_pts.push_back(Eigen::Vector2d(opt(0),opt(1)));
        }
    }

    /*
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    float32 angle_min
    float32 angle_max
    float32 scan_time
    float32 time_increment
    float32 range_min
    float32 range_max
    float32[] ranges
    float32[] angles
    float32[] intensities
     */

    // 构造函数
    void championLaserScanCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
    {
        static bool isFirstFrame = true;
        Eigen::Vector3d nowPose;
        // getOdomPose(激光雷达消息的时间戳, nowPose坐标),获得激光点对应的里程计位姿nowPose(x,y,yaw)
        if(getOdomPose(msg->header.stamp, nowPose) == false)
        {
            std::cout <<"Failed to get Odom Pose"<<std::endl;
            return ;
        }

        // 确实是第一帧
        if(isFirstFrame == true)
        {
            std::cout <<"First Frame"<<std::endl;
            isFirstFrame = false; // 后面的设置为不是第一帧
            m_prevLaserPose = nowPose;// 将当前的位姿存为上一帧的位姿
            ConvertChampionLaserScanToEigenPointCloud(msg,nowPose,m_prevPointCloud);
            return ; // 退出构造函数
        }

        // (x - x')^2 + (y - y')^2, 前后两个位置x,y距离的平方
        double delta_dist2 = std::pow(nowPose(0) - m_prevLaserPose(0),2) + std::pow(nowPose(1) - m_prevLaserPose(1),2);
        // fabs()输入输出是float的abs绝对值
        // | theta - theta' | 弧度, tfNormalizeAngle返回的弧度在[-TFSIMD_PI, TFSIMD_PI](即-pi, pi)之间, 前后两个位置的航向角误差
        double delta_angle = std::fabs(tfNormalizeAngle(nowPose(2) - m_prevLaserPose(2)));

        if(delta_dist2 < 0.2 * 0.2 && // 前后两个位置的距离<0.2
            // tfScalar tfRadians(tfScalar x) { return x * TFSIMD_RADS_PER_DEG; }
           delta_angle < tfRadians(10.0)) // 前后两个位置的航向角差小于10度
        {
            return ; // 退出循环
        }

        m_prevLaserPose = nowPose; // 将当前的位姿存为上一帧的位姿


        std::vector<Eigen::Vector2d> nowPts;
        //msg当前的激光信息,nowPose当前激光对应的里程计信息, nowPts?
        // 将当前的激光信息转换成Eigen中的x,y坐标?
        ConvertChampionLaserScanToEigenPointCloud(msg,nowPose,nowPts);


        //TODO
        //调用imls进行icp匹配，并输出结果．
        m_imlsMatcher.setSourcePointCloud(nowPts);
        m_imlsMatcher.setTargetPointCloud(m_prevPointCloud);

        Eigen::Matrix3d rPose,rCovariance;
        if(m_imlsMatcher.Match(rPose,rCovariance))
        {
            std::cout <<"IMLS Match Successful:"<<rPose(0,2)<<","<<rPose(1,2)<<","<<atan2(rPose(1,0),rPose(0,0))*57.295<<std::endl;
        }
        else
        {
            std::cout <<"IMLS Match Failed!!!!"<<std::endl;
        }

        //end of TODO

        m_laserscanSub.shutdown();
    }

    // 获得当前激光数据时刻的里程计数据
    bool getOdomPose(ros::Time t, // 激光雷达消息的时间戳
                     Eigen::Vector3d& pose) // 返回位姿(x, y, yaw)坐标+航向角
    {
        // Get the robot's pose获得机器人的位姿
        // static tf::Quaternion tf::createQuaternionFromRPY(double	roll, double pitch, double yaw)从roll,pitch,yaw创建一个四元素表示旋转分量
        // tf::Transform(Quaternion, Vector3)创建转换矩阵(Rotation+Translation)
        tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0,0,0)),
                                        t, // ros::Time stamp_; ///< The timestamp associated with this data
                                        "/base_link"); // std::string frame_id_; ///< The frame_id associated this data
        tf::Stamped<tf::Transform> odom_pose;
        try
        {
            // tf::TransformListener m_tfListener; 类的对象,一个tf监听器
            // Transform a Stamped Pose into the target frame
            // void transformPose(const std::string& target_frame, const Stamped<tf::Pose>& stamped_in, Stamped<tf::Pose>& stamped_out) const;
            // 将创建的带时间戳的ident位姿(base_link坐标系下面),转换到/odom里程计坐标系中,并保存到带时间戳的odom_pose变量中(/odom坐标系下面)
            m_tfListener.transformPose("/odom", ident, odom_pose);
        }
        // Pass through exceptions from tf2
        //typedef tf2::TransformException TransformException;
        catch(tf::TransformException e) // 出现错误,抓取错误
        {
            // 警告信息
            ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
            return false;
        }

        // 获得odom_pose的旋转分量中的偏航角
        double yaw = tf::getYaw(odom_pose.getRotation());
        // 返回的pose(x, y, yaw)
        pose << odom_pose.getOrigin().x(), odom_pose.getOrigin().y(), yaw;

        return true;
    }

    int m_FrameID;

    ros::NodeHandle m_nh;

    IMLSICPMatcher m_imlsMatcher;

    Eigen::Vector3d m_prevLaserPose;

    std::vector<Eigen::Vector2d> m_prevPointCloud;// ???

    tf::TransformListener m_tfListener;
    ros::Subscriber m_laserscanSub;
    ros::Publisher m_pointcloudPub;
    ros::Publisher m_normalsPub;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imls_debug");

    imlsDebug imls_debug;

    ros::spin();

    return (0);
}

