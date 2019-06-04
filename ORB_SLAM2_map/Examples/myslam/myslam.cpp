// 该文件将打开你电脑的摄像头，并将图像传递给ORB-SLAM2进行定位

// opencv
#include <opencv2/opencv.hpp>

// ORB-SLAM的系统接口
#include "System.h"
#include <string>
#include <chrono>   // for time stamp
#include <iostream>

using namespace std;

// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
//string parameterFile = "../myslam.yaml";
//string vocFile = "../../../Vocabulary/ORBvoc.txt";


const char* usage ="./myslam -c=0 -w=width -h=height -voc=../../Vocabulary/ORBvoc.txt -p=./myslam.yaml\n"
                   "./myslam -c=[opencv camera index] -w=[camera width] -h=[camera height] -voc=[../../Vocabulary/ORBvoc.txt]  -p=./myslam.yaml\n"
                   "./myslam -c=0 -w=1280 -h=720 -voc=../../Vocabulary/ORBvoc.txt  -p=./myslam.yaml\n";

void help()
{
    cout << usage<<endl;
}
int main(int argc, char **argv) {
    cv::CommandLineParser parser(argc, argv, "{help||}{c|0|camera index}{w|1280|width}{h|720|height}{voc|../../Vocabulary/ORBvoc.txt|vocFile}{p|./myslam.yaml|parameterFile}");
    if(argc < 3)
    {
        help();
        return 1;
    }
    int videoIndex = parser.get<int>("c");//摄像机
    int imgW = parser.get<int>("w");
    int imgH = parser.get<int>("h");
    string vocFile = parser.get<string>("voc");// "../../Vocabulary/ORBvoc.txt";
    string parameterFile =parser.get<string>("p");// "./myslam.yaml";
    // 声明 ORB-SLAM2 系统
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);

    // 获取相机图像代码
    cv::VideoCapture cap(videoIndex);    // change to 1 if you want to use USB camera.

    // 分辨率设为640x480
    cap.set(CV_CAP_PROP_FRAME_WIDTH, imgW);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgH);

    // 记录系统时间
    auto start = chrono::system_clock::now();

    while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackMonocular(frame, double(timestamp.count())/1000.0);
    }

    return 0;
}
