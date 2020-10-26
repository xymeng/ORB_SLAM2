/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings video_file" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;

    cout << argv[3] << "\n";
    cv::VideoCapture vc = cv::VideoCapture(std::string(argv[3]));

    // Check if camera opened successfully
    if(!vc.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, true);

    float _K[3][3] = {
            {1.14673746e+03, 0.00000000e+00, 9.64993175e+02},
            {0.00000000e+00, 1.14714045e+03, 5.44838433e+02},
            {0.00000000e+00, 0.00000000e+00, 1.00000000e+00}
    };
    cv::Mat K(3, 3, CV_32F, _K);

    float _D[5] = {-2.81611807e-01, 1.36990069e-01, -2.63780104e-04, -2.61087345e-04, -4.22056500e-02};
    // cv::Mat D(1, 5, CV_32F, _D);

    std::vector<float> D {-2.81611807e-01, 1.36990069e-01, -2.63780104e-04, -2.61087345e-04, -4.22056500e-02};

    cv::Mat new_K = cv::getOptimalNewCameraMatrix(
            K, D, cv::Size(1920, 1080), 0);

    cout << "new_K: " << new_K << "\n";

    int frame_idx = 0;

    while(1){
        cv::Mat frame;
        // Capture frame-by-frame
        vc >> frame;

        if (frame.empty())
            break;

        cv::Mat frame2;
        cv::undistort(frame, frame2, K, D, new_K);
        // If the frame is empty, break immediately

        cv::Mat frame3;
        cv::resize(frame2, frame3, cv::Size(0, 0), 0.5, 0.5);

//        cv::imshow("", frame3);
//        cv::waitKey(1);

        auto pose = SLAM.TrackMonocular(frame3, frame_idx * 0.1);
        usleep(size_t(0.1 * 1e6));

        frame_idx++;
    }

    return 0;
}
