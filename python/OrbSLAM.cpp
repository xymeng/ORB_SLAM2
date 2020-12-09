#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <cstdint>
#include <iostream>
#include <string>
#include <unistd.h>

#include <System.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace py = pybind11;
using std::string;
using std::vector;

class OrbSLAM2 {
public:
    OrbSLAM2(const string& vocab_file, const string& settings_file):
        SLAM(vocab_file, settings_file, ORB_SLAM2::System::MONOCULAR, false) {
    }

    ~OrbSLAM2() {
        SLAM.Shutdown();
    }

    vector<vector<float>> BuildMap(py::array_t<uint8_t> imgs, float delay) {
        auto imgs_view = imgs.unchecked<4>();  // N x H x W x 3
        auto n = imgs_view.shape(0);
        auto h = imgs_view.shape(1);
        auto w = imgs_view.shape(2);

        SLAM.DeactivateLocalizationMode();
        SLAM.Reset();
        vector<vector<float>> poses;

        for (int i = 0; i < n; i++) {
            cv::Mat img(h, w, CV_8UC3, (void*)imgs.data(i));
            cv::Mat pose = SLAM.TrackMonocular(img, float(i));
            usleep(delay * 1e6);
        }

        auto keyframes = SLAM.GetKeyFrames();

        cout << keyframes.size() << " keyframes\n";
        cout << SLAM.GetNumMapPoints() << " map points\n";

        sort(keyframes.begin(), keyframes.end(), ORB_SLAM2::KeyFrame::lId);
        for (auto kf: keyframes) {
            cv::Mat pose = kf->GetPose();
            cv::Mat posef;
            pose.convertTo(posef, CV_32F);
            vector<float> posev;
            posev.assign((float*)posef.datastart, (float*)posef.dataend);
            poses.push_back(posev);

            // auto center = kf->GetCameraCenter();
            // cout << center << ", ";
        }
        // cout << "\n";

        SLAM.ActivateLocalizationMode();
        should_relocalize = true;

        return poses;
    }

    vector<float> Localize(float timestamp, py::array_t<uint8_t> img) {
        if (should_relocalize) {
            // SLAM.ForceRelocalization();
            should_relocalize = false;
        }

        auto h = img.shape(0);
        auto w = img.shape(1);
        cv::Mat cvimg(h, w, CV_8UC3, (void*)img.data());
        cv::Mat pose = SLAM.TrackMonocular(cvimg, timestamp);

        cv::Mat posef;
        pose.convertTo(posef, CV_32F);
        vector<float> posev;
        posev.assign((float*)posef.datastart, (float*)posef.dataend);

        return posev;
    }

private:
    ORB_SLAM2::System SLAM;
    bool should_relocalize;
};


PYBIND11_MODULE(orbslam2, m) {
    py::class_<OrbSLAM2>(m, "OrbSLAM2")
            .def(py::init<const string&, const string&>())
            .def("BuildMap", &OrbSLAM2::BuildMap)
            .def("Localize", &OrbSLAM2::Localize);
}
