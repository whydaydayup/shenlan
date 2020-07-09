#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream> // file
#include <pangolin/pangolin.h> // Pangolin

using namespace std;

string estimated_file = "./estimated.txt";
string groundtruth_file = "./groundtruth.txt";

void DrawTrajectory( vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> );

int main( int argc, char* argv[] )
{
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> groundtruth_poses;

    ifstream ifestimated, ifgroundtruth;
    ifestimated.open( estimated_file );
    if( ! ifestimated.is_open() )
    {
        cout << "Open estimated.txt fail!" << endl;
        return -1;
    }
    else
    {
        while( !ifestimated.eof() )
        {
            Eigen::Vector3d t;
            Eigen::Quaterniond q;
            double timestamp;
            string line;
            getline(ifestimated, line, '\n');
            istringstream line_string(line);
            line_string >> timestamp
                        >> t[0] >> t[1] >> t[2]
                        >> q.x() >> q.y() >> q.z() >> q.w();
            Sophus::SE3 T(q, t);
            estimated_poses.push_back(T);
        }
    }

    ifgroundtruth.open( groundtruth_file );
    if( ! ifgroundtruth.is_open() )
    {
        cout << "Open groundtruth.txt fail!" << endl;
        return -1;
    }
    else
    {
        while( !ifgroundtruth.eof() )
        {
            string line;
            getline(ifgroundtruth, line, '\n');
            istringstream line_string(line);
            double tmp, data[8];
            int i = 0;
            while( line_string >> tmp )
            {
                data[i] = tmp;
                i++;
            }
            Eigen::Vector3d t( data[1], data[2], data[3] );
            Eigen::Quaterniond q( data[7], data[4], data[5], data[6] );
            Sophus::SE3 T(q, t);
            groundtruth_poses.push_back(T);
        }
    }
    cout << estimated_poses.size() << endl;
    cout << groundtruth_poses.size() << endl;
    /*
    Eigen::Vector3d e (1 , 2, 3);
    double l = e.norm();
    l = l*l;
    cout << endl << l << endl;
     */

    double sum = 0.0d;
    for(int i = 0; i < estimated_poses.size(); i++)
    {
        Sophus::SE3 Te = estimated_poses[i];
        Sophus::SE3 Tg = groundtruth_poses[i];
        // cout << Te << endl << Tg << endl;
        Eigen::Matrix<double, 6, 1> e = (Tg.inverse() * Te).log();
        double l = e.norm(); // norm二范数
        l = l*l;
        sum += l;
        //cout << e << endl;
    }
    double rmse = sqrt( sum / estimated_poses.size() );
    cout << endl << rmse << endl;

    // draw trajectory in pangolin
    DrawTrajectory(estimated_poses);
    DrawTrajectory(groundtruth_poses);
    return 0;
}

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    // 启动深度测试
    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
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
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}


