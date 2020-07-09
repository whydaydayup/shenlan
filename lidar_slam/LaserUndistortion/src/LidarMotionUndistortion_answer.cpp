#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>

pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");

class LidarMotionCalibrator
{
public:

    LidarMotionCalibrator(tf::TransformListener* tf)
    {
        tf_ = tf;
        scan_sub_ = nh_.subscribe("rplidar_scan", 10, &LidarMotionCalibrator::ScanCallBack, this);// this是类里面的回调,若不写成类,就不用了??
    }


    ~LidarMotionCalibrator()
    {
        if(tf_!=NULL)
            delete tf_;
    }

    // 拿到原始的激光数据来进行处理
    void ScanCallBack(const sensor_msgs::LaserScanConstPtr& scan_msg)
    {
        //转换到矫正需要的数据
        ros::Time startTime, endTime;
        startTime = scan_msg->header.stamp;

        sensor_msgs::LaserScan laserScanMsg = *scan_msg;

        //得到最终点的时间
        int beamNum = laserScanMsg.ranges.size();
        endTime = startTime + ros::Duration(laserScanMsg.time_increment * beamNum);

        // 将数据复制出来
        std::vector<double> angles,ranges;
        for(int i = 0; i < beamNum;i++)
        {
            double lidar_dist = laserScanMsg.ranges[i];
            double lidar_angle = laserScanMsg.angle_min + laserScanMsg.angle_increment * i;

            ranges.push_back(lidar_dist);
            angles.push_back(lidar_angle);
        }

        //转换为pcl::pointcloud for visuailization

        visual_cloud_.clear();
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;


            pcl::PointXYZRGB pt;
            pt.x = ranges[i] * cos(angles[i]);
            pt.y = ranges[i] * sin(angles[i]);
            pt.z = 1.0;

            // pack r/g/b into rgb
            unsigned char r = 255, g = 0, b = 0;    //red color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }
        std::cout << std::endl;


        //进行矫正
        Lidar_Calibration(ranges,angles,
                          startTime,
                          endTime,
                          tf_);

        //转换为pcl::pointcloud for visuailization
        for(int i = 0; i < ranges.size();i++)
        {

            if(ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            pcl::PointXYZRGB pt;
            pt.x = ranges[i] * cos(angles[i]);
            pt.y = ranges[i] * sin(angles[i]);
            pt.z = 1.0;

            unsigned char r = 0, g = 255, b = 0;    // blue color
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            visual_cloud_.push_back(pt);
        }

        std::cout <<std::endl<<std::endl;



        //进行显示
         g_PointCloudView.showCloud(visual_cloud_.makeShared());
    }


    /**
     * @name getLaserPose()获得对应时间戳下的机器人位姿,   没进行插值,离时间戳最近的转换关系
     * @brief 得到机器人在里程计坐标系中的位姿tf::Pose: (旋转/平移)
     *        得到dt时刻激光雷达在odom坐标系的位姿
     * @param odom_pos  机器人的位姿
     * @param dt        dt时刻
     * @param tf_
    */
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener * tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = "base_laser";
        robot_pose.stamp_ = dt;   //设置为ros::Time()表示返回最近的转换关系

        // get the global pose of the robot
        try
        {
            // 时间同步是通过监听tf来确定里程计的时间和位置
            // getLaserPose为什么把每一个激光点的时间当成是waitfortransform的输入
            // 因为每一个激光点都是畸变的(除了第一个点)，所以获取每一个激光点那个时间的里程计位姿，
            // 通过这个位姿信息来把这个激光点矫正到第一个点的那个基准去
            if(!tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5)))             // 0.15s 的时间可以修改
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            tf_->transformPose("/odom", robot_pose, odom_pose);
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }


    /**
     * 去除的是激光雷达运动的畸变,激光雷达旋转一周的同时车辆继续往前运动,
     * 造成旋转一周最后一个激光点的基础坐标系不是第一个激光点的基础坐标系
     * @brief Lidar_MotionCalibration
     *        激光雷达运动畸变去除分段函数;
     *        在此分段函数中，认为机器人是匀速运动；
     * @param frame_base_pose       标定完毕之后的基准坐标系
     * @param frame_start_pose      本分段第一个激光点对应的位姿
     * @param frame_end_pose        本分段最后一个激光点对应的位姿
     * // range距离 = sqrt(px*px + py*py)   angles角度 = arctan( py/px )
     * // px = range * cos(angles);  py = range * sin(angles);
     * @param ranges                激光数据－－距离
     * @param angles                激光数据－－角度
     * @param startIndex            本分段第一个激光点在激光帧中的下标
     * @param beam_number           本分段的激光点数量
     */
    void Lidar_MotionCalibration(
		// frame_base_pose是激光雷达一圈数据中第一个激光点发射的机器人位姿.frame_base_pose就是frame_start_pose
		// 后面的激光点数据要根据位姿转换到第一个激光点的机器人坐标系来
		    // typedef tf::Transform tf::Pose转换矩阵(旋转,平移)
		    // Matrix3x3 m_basis;
            // Vector3   m_origin;
            tf::Stamped<tf::Pose> frame_base_pose, // frame_base_pose == frame_start_pose
            tf::Stamped<tf::Pose> frame_start_pose,
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<double>& ranges,
            std::vector<double>& angles,
            int startIndex, // 本分段第一个激光点在激光帧中的下标
            int& beam_number)
    {
        //每个位姿进行线性插值时的步长 = 1/(本分段的激光点数量-1)
        double beam_step = 1.0 / (beam_number-1);

	    // getRotation + get
        //机器人的起始角度 和 最终角度  getRotation()Return a quaternion representing the rotation.
        // frame_start_pose.getRotation().getW() / getX()/ Y Z
        tf::Quaternion start_angle_q =   frame_start_pose.getRotation(); // 第一个激光点的位姿中的旋转分量
        tf::Quaternion   end_angle_q =   frame_end_pose.getRotation(); // 最后一个激光点的位姿中的旋转分量

        //转换到弧度 getYaw()获得绕Z轴旋转的值,航向角
        // static double tf::getYaw (const Quaternion &	bt_q) getting yaw from a Quaternion.
        double start_angle_r = tf::getYaw(start_angle_q); // 起始的航向角   // 第一个激光点的绕Z轴的旋转角度
        double base_angle_r = tf::getYaw(frame_base_pose.getRotation()); // 基准坐标系的航向角  // 基准坐标的绕Z轴的旋转角度==第一个激光点的绕Z轴的旋转角度

        //机器人的起始位姿
        // Vector3& tf::Transform::getOrigin() // Return the origin vector translation.
        tf::Vector3 start_pos = frame_start_pose.getOrigin(); // 第一个激光点的位姿中的平移分量
	    // 将z方向置为0
	    // void tf::Vector3::setZ(tfScalar z)  //Set the z value.
        start_pos.setZ(0); // 将第一个激光点的坐标中的Z分量置为0,没有高度的变化

        //最终位姿
        tf::Vector3 end_pos = frame_end_pose.getOrigin(); // 最后一个激光点的位姿的平面坐标x,y,z
        end_pos.setZ(0); // 将最后一个激光点的坐标中的Z分量置为0,没有高度的变化

        //基础坐标系
        tf::Vector3 base_pos = frame_base_pose.getOrigin(); // 基准坐标系的位姿的平面坐标x,y,z == 第一个激光点的位姿
        base_pos.setZ(0);// 将基准坐标系的平面坐标中的Z分量置为0,没有高度的变化



        double mid_angle; // 设置插值计算时中间激光点的航向角(绕Z轴的角度)
        // x, y, z
        tf::Vector3 mid_pos; // 中间激光点的坐标x, y, z?
        tf::Vector3 mid_point;// 中间激光点

        double lidar_angle, lidar_dist; // 激光点的角度,距离
        //插值计算出来每个点对应的位姿
        for(int i = 0; i< beam_number;i++) // 0 ~ (beam_number-1)
        {
            //角度插值
            // Quaternion tf::Quaternion::slerp(const Quaternion & q, const tfScalar &t) const
            // Return the quaternion which is the result of Spherical Linear Interpolation球面线性差值 between this and the other quaternion.两个四元素的球面线性差值
            // Parameters:
            //    q	The other quaternion to interpolate with
            //    t	The ratio between this and q to interpolate. If t = 0 the result is this, if t=1 the result is q. Slerp interpolates assuming constant velocity.
            mid_angle =  tf::getYaw(start_angle_q.slerp(end_angle_q, beam_step * i)); // 起始点~终点的旋转分量的球面线性差值,获得中间点的旋转分量
            // 然后最终获得中间激光点的航向角(绕Z轴的角度)
            // 获得第i个激光点在里程计坐标系下面的旋转分量

            //线性插值
            // Vector3 tf::Vector3::lerp(const Vector3 &v, const tfScalar &t) const;
            // Return the linear interpolation between this and another vector.两个平移向量的线性差值
            // Parameters:
            //    v	The other vector
            //    t	The ration of this to v (t = 0 => return this, t=1 => return other)
            mid_pos = start_pos.lerp(end_pos, beam_step * i); // 起始点~终点的平移分量的线性差值,获得中间点的平移分量x,y,z坐标
            // 获得第i个激光点在里程计坐标系下面的坐标x,y,z

            //得到激光点在odom 坐标系中的坐标 根据
            double tmp_angle;

            //如果激光雷达不等于无穷,则需要进行矫正.
            // typedef double tfScalar;(Scalar.h文件中定义的)
            // bool tfFuzzyZero(tfScalar x) { return tfFabs(x) < TFSIMD_EPSILON; }   (Scalar.h文件中定义的)
            // tfScalar tfFabs(tfScalar x) { return fabs(x); } // extern float fabs(float x); fabs求浮点数的绝对值,返回的是浮点数,abs求int的绝对值
            // startIndex:本分段第一个激光点在激光帧中的下标,
            // 本分段第i个激光点的距离不趋于0,则进行矫正
            if( tfFuzzyZero(ranges[startIndex + i]) == false)
            {
                //计算对应的激光点在odom坐标系中的坐标

                //得到这帧激光束距离和夹角 range, angle
                lidar_dist  =  ranges[startIndex+i];
                lidar_angle =  angles[startIndex+i];

                //激光雷达坐标系下的坐标  range,angle -> px, py
                double laser_x,laser_y;
                laser_x = lidar_dist * cos(lidar_angle); // px = range * cos(angle)
                laser_y = lidar_dist * sin(lidar_angle); // py = range * sin(angle)

                // cos(theta), -sin(theta),    x,
                // sin(theta),  cos(theta),    y,
                //          0,           0,    1;
                //里程计坐标系下的坐标
                double odom_x,odom_y;
                // 当前激光点的x * cos(当前激光点的angle) - 当前激光点的y * sin(当前激光点的angle) + 当前激光点的平移分量的x
                odom_x = laser_x * cos(mid_angle) - laser_y * sin(mid_angle) + mid_pos.x();
                // 当前激光点的x * sin(当前激光点的angle) + 当前激光点的y * cos(当前激光点的angle) + 当前激光点的平移分量的y
                odom_y = laser_x * sin(mid_angle) + laser_y * cos(mid_angle) + mid_pos.y();

                //转换到类型中去
                // tf::Vector3 mid_pos;
                // void	setValue (const tfScalar &x, const tfScalar &y, const tfScalar &z)
                mid_point.setValue(odom_x, odom_y, 0); // 矫正后的当前激光点的坐标x, y, z

                //把在odom坐标系中的激光数据点 转换到 基础坐标系
                double x0,y0,a0,s,c;
                // tf::Vector3 base_pos;
                x0 = base_pos.x(); // 基准(第一个激光点的)x,y,theta
                y0 = base_pos.y();
                // double base_angle_r = tf::getYaw(frame_base_pose.getRotation()); // 基准坐标系的航向角
                a0 = base_angle_r;
                s = sin(a0);
                c = cos(a0);
                /*
                 * 把base转换到odom 为[c -s x0;
                 *                   s c y0;
                 *                   0 0 1]
                 * 把odom转换到base为 [c s -x0*c-y0*s;
                 *                  -s c  x0*s-y0*c;
                 *                  0 0 1]
                 */
                double tmp_x,tmp_y;
                // const tfScalar& tf::Vector3::x( ) const // Return the x value.
                tmp_x =  mid_point.x()*c  + mid_point.y()*s - x0*c - y0*s;
                tmp_y = -mid_point.x()*s  + mid_point.y()*c  + x0*s - y0*c;
                mid_point.setValue(tmp_x,tmp_y,0);

                //然后计算以起始坐标为起点的 dist angle
                double dx,dy;
                dx = (mid_point.x());
                dy = (mid_point.y());
                // range距离 = sqrt(px*px + py*py)
                // angles角度 = arctan( py/px )
                lidar_dist = sqrt(dx*dx + dy*dy);
                // extern float atan2(float y, float x); // 求y/x（弧度表示）的反正切值, 值域为(-π/2，+π/2)
                // extern float atan(float x); // 求x（弧度表示）的反正切值, 值域为(-π/2，+π/2)
                // atan2 比 atan 稳定
                lidar_angle = atan2(dy,dx);

                //激光雷达被矫正
                ranges[startIndex+i] = lidar_dist;
                angles[startIndex+i] = lidar_angle;
            }
            //如果等于无穷,则随便计算一下角度
            else
            {
                //激光角度
                lidar_angle = angles[startIndex+i];

                //里程计坐标系的角度
                tmp_angle = mid_angle + lidar_angle;
                // #define TFSIMD_2_PI         tfScalar(6.283185307179586232)
                // #define TFSIMD_PI           (TFSIMD_2_PI * tfScalar(0.5))
                // tfScalar tfNormalizeAngle(tfScalar angleInRadians) // returns normalized value in range [-TFSIMD_PI, TFSIMD_PI]
                tmp_angle = tfNormalizeAngle(tmp_angle);

                //如果数据非法 则只需要设置角度就可以了。把角度换算成start_pos坐标系内的角度
                lidar_angle = tfNormalizeAngle(tmp_angle - start_angle_r);

                angles[startIndex+i] = lidar_angle;
            }
        }
    }



    //激光雷达数据　分段线性进行插值　分段的周期为10ms
    //这里会调用Lidar_MotionCalibration()
    /**
     * @name Lidar_Calibration()
     * @brief 激光雷达数据　分段线性进行差值　分段的周期为5ms
     * @param ranges 激光束的距离值集合
     * @param angle　激光束的角度值集合
     * @param startTime　第一束激光的时间戳
     * @param endTime　最后一束激光的时间戳
     * @param *tf_
    */
    void Lidar_Calibration(std::vector<double>& ranges,
                           std::vector<double>& angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener * tf_)
    {
        //统计激光束的数量
        int beamNumber = ranges.size();
        if(beamNumber != angles.size())
        {
            ROS_ERROR("Error:ranges not match to the angles");
            return ;
        }

        // 5ms来进行分段
        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        //起始时间 us
        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / beamNumber; // 每束激光数据的时间间隔

        //当前插值的段的起始坐标
        int start_index = 0;

        //起始点的位姿 这里要得到起始点位置的原因是　起始点就是我们的base_pose
        //所有的激光点的基准位姿都会改成我们的base_pose
        // ROS_INFO("get start pose");

        if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }

        if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0),tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }

        int cnt = 0;
        //基准坐标就是第一个位姿的坐标
        frame_base_pose = frame_start_pose;
        for(int i = 0; i < beamNumber; i++)
        {
            //分段线性,时间段的大小为interpolation_time_duration
            double mid_time = start_time + time_inc * (i - start_index);
            if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
            {
                cnt++;

                //得到起点和终点的位姿
                //终点的位姿
                if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
                {
                    ROS_ERROR("Mid %d Pose Error",cnt);
                    return ;
                }

                //对当前的起点和终点进行插值
                //interpolation_time_duration中间有多少个点.
                int interp_count = i - start_index + 1;

                Lidar_MotionCalibration(frame_base_pose,
                                        frame_start_pose,
                                        frame_mid_pose,
                                        ranges,
                                        angles,
                                        start_index,
                                        interp_count);

                //更新时间
                start_time = mid_time;
                start_index = i;
                frame_start_pose = frame_mid_pose;
            }
        }
    }

public:
    tf::TransformListener* tf_;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;

    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
};




int main(int argc,char ** argv)
{
    // 初始化一个ROS节点
    ros::init(argc,argv,"LidarMotionCalib");

    // 初始化一个tf listener,
    tf::TransformListener tf(ros::Duration(10.0));

    // 将tf listener的地址传入,初始化一个类的对象
    LidarMotionCalibrator tmpLidarMotionCalib(&tf);

    ros::spin();
    return 0;
}


