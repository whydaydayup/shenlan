#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char* argv[])
{
    cv::Mat image;
    image = cv::imread( argv[1] );
    if( image.data == nullptr )
    {
        cerr << "File " << argv[1] << " is not exist!" << endl;
        return -1;
    }
    cout << "Width: " << image.cols
         << ", Height: " << image.rows
         << ", channels: " << image.channels()
         << endl;
    cv::imshow("image", image);
    cv::waitKey( 0 );
    if( image.type() != CV_8UC1 && image.type() != CV_8UC3 )
    {
        cout << "Input a grayscale or a RGB one" << endl;
        return 0;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for( size_t y = 0; y < image.rows; y++ )
    {
        for( size_t x = 0; x < image.cols; x++ )
        {
            unsigned
        }
    }


}