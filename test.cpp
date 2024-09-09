#include <dlfcn.h>
#include <cstdint>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <opencv2/opencv.hpp>  // Include OpenCV for image handling

using namespace std;
using namespace cv;  // For OpenCV functions

namespace
{
    const string SO_FILE_PATH = "/home/ros_ws/src/ros2-detection-python/ros_shared_object/build/libros_shared_object_library.so";
    volatile sig_atomic_t flag = 1;

    void handler(int signum)
    {
        flag = 0;
    }
}

int main(int argc, char* argv[])
{
    signal(SIGINT, handler);

    void* handle = dlopen(SO_FILE_PATH.c_str(), RTLD_NOW);
    if (!handle) {
        cerr << "Error loading shared object: " << dlerror() << endl;
        return 1;
    }

    // Load the function pointers
    intptr_t (*create)() = (intptr_t (*)())dlsym(handle, "create");
    if (!create) {
        cerr << "Error loading symbol create: " << dlerror() << endl;
        return 1;
    }

    void (*spin_some)(intptr_t) = (void (*)(intptr_t))dlsym(handle, "spin_some");
    if (!spin_some) {
        cerr << "Error loading symbol spin_some: " << dlerror() << endl;
        return 1;
    }

    // Function to publish the sensor image (assuming it takes an image and an instance)
    void (*talk)(intptr_t, const cv::Mat&) = (void (*)(intptr_t, const cv::Mat&))dlsym(handle, "talk");
    if (!talk) {
        cerr << "Error loading symbol talk: " << dlerror() << endl;
        return 1;
    }

    void (*destroy)(intptr_t) = (void (*)(intptr_t))dlsym(handle, "destroy");
    if (!destroy) {
        cerr << "Error loading symbol destroy: " << dlerror() << endl;
        return 1;
    }

    // Create the instance of the ROS2 node
    intptr_t instance = (*create)();

    cv::Mat frame = cv::Mat::ones(480, 640, CV_8UC3)*125; 
    int publish_count = 0;
    while (flag != 0)
    {
        if (frame.empty()) {
            cerr << "Empty frame captured!" << endl;
            break;
        }

        // Increment the publish count
        publish_count++;

        // Add the published count as text to the image
        std::string text = "cnt: " + std::to_string(publish_count);
        double fontScale = 1.0;
        int thickness = 2;
          cv::putText(
            frame,
            text,
            cv::Point(25,75),
            cv::FONT_HERSHEY_SIMPLEX,
            2.5,
            cv::Scalar(0,0,0),
            3
        );

        // Publish the image
        (*talk)(instance, frame);
        (*spin_some)(instance);

        this_thread::sleep_for(chrono::milliseconds(30));  // Control the frame rate
    }

    // Clean up
    (*destroy)(instance);
    dlclose(handle);

    return 0;
}
