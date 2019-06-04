// 该文件将打开给定的视频文件，并将图像传递给ORB-SLAM2进行定位

// 需要opencv
#include <opencv2/opencv.hpp>

// ORB-SLAM的系统接口
#include "System.h"

#include <string>
#include <chrono>   // for time stamp
#include <iostream>

using namespace std;

// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
//string parameterFile = "./myvideo.yaml";
//string vocFile = "../../Vocabulary/ORBvoc.txt";

const char *usage = "./myvideo -v=myvideo -w=width -h=height -voc=../../Vocabulary/ORBvoc.txt -p=./myvideo.yaml -rp=false\n"
                    "./myvideo -v=[load video] -w=[resize width] -h=[resize height] -voc=[../../Vocabulary/ORBvoc.txt]  -p=./myvideo.yaml -rp=false\n"
                    "./myvideo -v=./myvideo.avi -w=1280 -h=720 -voc=../../Vocabulary/ORBvoc.txt  -p=./myvideo.yaml -rp=false\n";

void help() {
    cout << usage << endl;

}


std::time_t getTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
    //std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
    return timestamp;
}

int main(int argc, char **argv) {
    cv::CommandLineParser parser(argc, argv,
                                 "{help||}{v|./myvideo.avi|videoFile}{w|1280|width}{h|720|height}{voc|../../Vocabulary/ORBvoc.txt|vocFile}{p|./myvideo.yaml|parameterFile}{rp|false|bReuseMap}");
    if (argc < 4) {
        help();
        return 1;
    }

    std::string videoFile = parser.get<string>("v");// 视频文件
    int imgW = parser.get<int>("w");
    int imgH = parser.get<int>("h");
    string vocFile = parser.get<string>("voc");// "../../Vocabulary/ORBvoc.txt";
    string parameterFile = parser.get<string>("p");// "./myvideo.yaml";
    bool bReuseMap = parser.get<bool>("rp");// 地图复用

    // 声明 ORB-SLAM2 系统
//    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true, bReuseMap);
    // 获取视频图像
    cv::VideoCapture cap(videoFile);    // change to 1 if you want to use USB camera.

    // 记录系统时间
    auto start = chrono::system_clock::now();
    if (bReuseMap)
        SLAM.LoadMap("Slam_Map.bin");
    int num_image =0;
    while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据
        if (frame.data == nullptr)
            break;
//        cv::imshow("image", frame);
        num_image++;
        // rescale because image is too large
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(imgW, imgH));
//        auto now = chrono::system_clock::now();
//        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);//668877
//        cv::Mat Tcw = SLAM.TrackMonocular(frame_resized, double(timestamp.count()) / 1000.0);//RT 3*4

        time_t myTimeStamp=getTimeStamp();//        clock_t myTimeStamp=getTimeStamp_s();
        cv::Mat Tcw=SLAM.TrackMonocular(frame_resized, double(myTimeStamp)/1000.0);//cv::Mat Tcw=

//        cv::Mat Tcw = SLAM.TrackMonocular(frame_resized, double(num_image));//RT 3*4    sbug
        cout << "Tcw=  " << Tcw << endl;
        if (!bReuseMap && !Tcw.empty()) {
            // Save camera everyframe trajectory
//            SLAM.SaveReLocalFrameTrajectoryTUM("./every_frameTrajectory.txt", double(myTimeStamp)/1000.0, Tcw, 1);
            SLAM.SaveEveryFrameTrajectoryTUMCount("./every_frameTrajectory.txt",num_image++,double(myTimeStamp)/1000.0, Tcw, 1);
        }
        if (bReuseMap && !Tcw.empty()) {
            // scale used to real space
            float scale = 6.789;
//            SLAM.SaveReLocalFrameTrajectoryTUM("./reLocalFrameTrajectory.txt",double(timestamp.count()) / 1000.0,Tcw,scale);
//            SLAM.SaveReLocalFrameTrajectoryTUM("./reLocalFrameTrajectory.txt", double(num_image), Tcw, scale);
            SLAM.SaveReLocalFrameTrajectoryTUM("./reLocalFrameTrajectory.txt", double(myTimeStamp)/1000.0, Tcw, scale);
        }
//        cv::waitKey(30);

    }
    if (!bReuseMap) {
        // Save camera trajectory
        SLAM.SaveKeyFrameTrajectoryTUM("./KeyFrameTrajectory.txt");//
        // Save map
        SLAM.SaveMap("Slam_latest_Map.bin");
    }
    SLAM.Shutdown();

    return 0;
}



//ofstream f;
//f.open(filename.c_str());
//f << fixed;
//
//for(size_t i=0; i<vpKFs.size(); i++)
//{
//KeyFrame* pKF = vpKFs[i];
//
//// pKF->SetPose(pKF->GetPose()*Two);
//
//if(pKF->isBad())
//continue;
//
//cv::Mat R = pKF->GetRotation().t();
//vector<float> q = Converter::toQuaternion(R);
//cv::Mat t = pKF->GetCameraCenter();
//f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
//<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
//
//}
//
//f.close();
//cout << endl << "trajectory saved!" << endl;
