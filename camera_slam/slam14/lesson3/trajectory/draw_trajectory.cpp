#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "./trajectory.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

  /*  实际上模板和函数一样，是可以有默认参数的，std::vector的声明是
    template<
            class T,
            class Allocator = std::allocator<T>
    > class vector;
    有两个模板参数，T 是元素类型，而 Allocator 负责提供 vector 需要用到的动态内存。
    其中 Allocator 参数有默认值，一般的使用不需要指定这个参数。但有时对内存有特殊需求，就需要提供自己定义的内存管理类。
    把容器操作和内存管理分开，这是STL的一个亮点，你在设计容器时也可以学习*/
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream file;
    file.open(trajectory_file, ios::in);

    if( !file )
    {
        cout << "Open file failure!" << endl;
        file.close();
        return -1;
    }
    for(int i=0; i < 620; i++)
    {
        double data[8];
        for(auto &d :data )
            file >> d;
        Eigen::Quaterniond q( data[7], data[4], data[5], data[6] );
        Eigen::Vector3d t(data[1], data[2], data[3] );
        Sophus::SE3 T(q, t);
        poses.push_back(T);
    }


/*    // if(file.fail())
    if( !file.is_open() )
    {
        cout << "File lost" << endl;
        file.close();
        return -1;
    }
    else // File exist
    {
        while( !file.eof() )
        {
            string line;
            double tmp, data[8];
            int i = 0;
            // istream& getline ( istream &is , string &str , char delim );
            getline(file, line, '\n');
            cout << line << endl;
            istringstream line_string(line);
*//*            while( line_string >> tmp )
            {
                data[i] = tmp;
                cout << data[i] << " ";
                i++;
            }
            cout << endl;
            Eigen::Vector3d t(data[1], data[2], data[3]);
            Eigen::Quaterniond q( data[7], data[4], data[5], data[6]);*//*
            Eigen::Vector3d t;
            Eigen::Quaterniond q;
            double timestamp;
            line_string >> timestamp >> t[0] >> t[1] >> t[2] >> q.x() >> q.y() >> q.z() >> q.w();
            Sophus::SE3 T(q, t);
            poses.push_back(T);
        }
        file.close();
    }*/
    // end your code here

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
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