#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
using namespace cv;
using namespace Eigen;

string compare_file = "./compare.txt";

using namespace std;
using namespace Eigen;
using namespace cv;
// path to trajectory file
string trajectory = "./compare.txt";
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> ,vector<Sophus::SE3,
Eigen::aligned_allocator<Sophus::SE3>>);
void pose_estimation_3d3d (
        const vector<Point3f>& pts1,
        const vector<Point3f>& pts2,
        Matrix3d& R, Vector3d& t
);
int main( int argc, char** argv )
{
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e, poses_e2g, poses_g;

    vector<Eigen::Quaterniond> qe, qg;
    vector<Eigen::Vector3d> te, tg;
    vector<Point3f> pts_e, pts_g;
    int i = 0;
    ifstream ifcompare;
    ifcompare.open( compare_file.c_str() );
    if ( !ifcompare.is_open() )
    {
        cout << "Open comapre.txt fail! " << endl;
        return 1;
    }
    else
    {
        string sCompare;
        while( getline(ifcompare, sCompare, '\n') && !sCompare.empty() )
        {
            double timestamp_e, timestamp_g;
            istringstream issCompare( sCompare );
            Eigen::Quaterniond qe_tmp, qg_tmp;
            Eigen::Vector3d te_tmp, tg_tmp;
            // vector存入数据的方式！！！！？？？？？
            // issCompare >> timestamp_e >> te.at(i)[0] >> te.at(i)[1] >> te.at(i)[2] >> qe.at(i).x() >> qe.at(i).y() >> qe.at(i).z() >> qe.at(i).w()
            //            >> timestamp_g >> tg.at(i)[0] >> tg.at(i)[1] >> tg.at(i)[2] >> qg.at(i).x() >> qg.at(i).y() >> qg.at(i).z() >> qg.at(i).w();
            issCompare >> timestamp_e >> te_tmp[0] >> te_tmp[1] >> te_tmp[2] >> qe_tmp.x() >> qe_tmp.y() >> qe_tmp.z() >> qe_tmp.w()
                       >> timestamp_g >> tg_tmp[0] >> tg_tmp[1] >> tg_tmp[2] >> qg_tmp.x() >> qg_tmp.y() >> qg_tmp.z() >> qg_tmp.w();
            te.push_back(te_tmp);
            tg.push_back(tg_tmp);
            qe.push_back(qe_tmp);
            qg.push_back(qg_tmp);
            // cout << te.at(i) << endl << tg.at(i) << endl;
            Sophus::SE3 Te(qe_tmp,te_tmp);
            Sophus::SE3 Tg(qg_tmp,tg_tmp);
            poses_e.push_back(Te);
            poses_g.push_back(Tg);
            pts_e.push_back( Point3f(te_tmp[0], te_tmp[1], te_tmp[2]) );
            pts_g.push_back( Point3f(tg_tmp[0], tg_tmp[1], tg_tmp[2]) );
            i++;
        }
    }
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    // pts1 = R*pts2 +t; 得到的Rt是第二帧到第一帧的变换
    // pts_g = R*pts_e + t;
    pose_estimation_3d3d( pts_g, pts_e, R, t );
    Sophus::SE3 Tge( R, t );
    cout << "R: " << endl << R << endl << "t: " << endl << t << endl << "Tge: " << endl << Tge.matrix() << endl;
    for ( int i =0; i<poses_g.size(); i++ )
    {
        poses_e2g.push_back(Tge * poses_e[i]);
    }

    DrawTrajectory(poses_e2g,poses_g);

    return 0;
}
// pts1 = R*pts2 +t; 得到的Rt是第二帧到第一帧的变换
void pose_estimation_3d3d (
        const vector<Point3f>& pts1,
        const vector<Point3f>& pts2,
        Eigen::Matrix3d& R, Eigen::Vector3d& t
)
{
    Point3f p1, p2;     // center of mass
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f( Vec3f(p1) /  N);
    p2 = Point3f( Vec3f(p2) / N);
    vector<Point3f>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    R = U* ( V.transpose() );
    t = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R * Eigen::Vector3d ( p2.x, p2.y, p2.z );
}

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) {
    if (poses1.empty() || poses2.empty() ) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    //创建一个窗口
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    //启动深度测试
    glEnable(GL_DEPTH_TEST);
    //启动混合
    glEnable(GL_BLEND);
    //混合函数glBlendFunc( GLenum sfactor , GLenum dfactor );sfactor 源混合因子dfactor 目标混合因子
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);


        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            glColor3f(1 - (float) i / poses1.size(), 0.0f, (float) i / poses1.size());
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t j = 0; j < poses2.size() - 1; j++) {
            glColor3f(1 - (float) j / poses2.size(), 0.0f, (float) j / poses2.size());
            glBegin(GL_LINES);
            auto p3 = poses2[j], p4 = poses2[j + 1];
            glVertex3d(p3.translation()[0], p3.translation()[1], p3.translation()[2]);
            glVertex3d(p4.translation()[0], p4.translation()[1], p4.translation()[2]);
            glEnd();

        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
