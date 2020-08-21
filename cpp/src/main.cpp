///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/***********************************************************************************************
 ** This sample demonstrates how to use the ZED SDK with OpenCV. 					  	      **
 ** Depth and images are captured with the ZED SDK, converted to OpenCV format and displayed. **
 ***********************************************************************************************/

 // ZED includes
#include <sl/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Sample includes
#include <SaveDepth.hpp>
#include <thread>
#include <time_tracker.h>
#include <chrono>

using namespace sl;

cv::Mat slMat2cvMat(Mat& input);
void printHelp();

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::VGA;
    init_params.coordinate_units = UNIT::METER;
    init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Coordinate system used in ROS
    init_params.camera_fps = 60;
    init_params.depth_mode = DEPTH_MODE::NONE;
    if (argc > 1) init_params.input.setFromSVOFile(argv[1]);

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Display help in console
    printHelp();

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;
    runtime_parameters.enable_depth = false;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getCameraInformation().camera_resolution;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat imageL_zed(image_size.width, image_size.height, MAT_TYPE::U8_C4);
    cv::Mat imageL_ocv = slMat2cvMat(imageL_zed);
    Mat imageR_zed(image_size.width, image_size.height, MAT_TYPE::U8_C4);
    cv::Mat imageR_ocv = slMat2cvMat(imageR_zed);
    cv::Mat grayL;
    cv::Mat grayR;

    SensorsData sensors_data;
    Timestamp last_imu_ts = 0;

    // Loop until 'q' is pressed
    char key = ' ';
    while (true) {
        timer tWall = {.name = "wall_time",.clock_id = CLOCK_REALTIME,};
        start_timer(&tWall);

        // camera thread
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(imageL_zed, VIEW::LEFT, MEM::CPU, image_size);
            zed.retrieveImage(imageR_zed, VIEW::RIGHT, MEM::CPU, image_size);

            stop_timer(&tWall);
    				print_timer(&tWall);

            // std::cout << "Left cam fx: " << zed.getCameraInformation().calibration_parameters.left_cam.fx << std::endl;
            // std::cout << "Left cam fy: " << zed.getCameraInformation().calibration_parameters.left_cam.fy << std::endl;
            // std::cout << "Left cam cx: " << zed.getCameraInformation().calibration_parameters.left_cam.cx << std::endl;
            // std::cout << "Left cam cy: " << zed.getCameraInformation().calibration_parameters.left_cam.cy << std::endl;
            // std::cout << "Left cam k1: " << zed.getCameraInformation().calibration_parameters.left_cam.disto[0] << std::endl;
            // std::cout << "Left cam k2: " << zed.getCameraInformation().calibration_parameters.left_cam.disto[1] << std::endl;
            // std::cout << "Left cam p1: " << zed.getCameraInformation().calibration_parameters.left_cam.disto[2] << std::endl;
            // std::cout << "Left cam p2: " << zed.getCameraInformation().calibration_parameters.left_cam.disto[3] << std::endl;
            // std::cout << "Left cam k3: " << zed.getCameraInformation().calibration_parameters.left_cam.disto[4] <<"\n" <<std::endl;
            //
            // std::cout << "Right cam fx: " << zed.getCameraInformation().calibration_parameters.right_cam.fx << std::endl;
            // std::cout << "Right cam fy: " << zed.getCameraInformation().calibration_parameters.right_cam.fy << std::endl;
            // std::cout << "Right cam cx: " << zed.getCameraInformation().calibration_parameters.right_cam.cx << std::endl;
            // std::cout << "Right cam cy: " << zed.getCameraInformation().calibration_parameters.right_cam.cy << std::endl;
            // std::cout << "Right cam k1: " << zed.getCameraInformation().calibration_parameters.right_cam.disto[0] << std::endl;
            // std::cout << "Right cam k2: " << zed.getCameraInformation().calibration_parameters.right_cam.disto[1] << std::endl;
            // std::cout << "Right cam p1: " << zed.getCameraInformation().calibration_parameters.right_cam.disto[2] << std::endl;
            // std::cout << "Right cam p2: " << zed.getCameraInformation().calibration_parameters.right_cam.disto[3] << std::endl;
            // std::cout << "Right cam k3: " << zed.getCameraInformation().calibration_parameters.right_cam.disto[4] <<"\n" <<std::endl;
        }

        // // imu thread
        // zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);
        // if (sensors_data.imu.timestamp > last_imu_ts) {
        //   std::this_thread::sleep_for(std::chrono::milliseconds{2});
        //   std::cout << "IMU Rate: " << sensors_data.imu.effective_rate << "\n" << std::endl;
        //   last_imu_ts = sensors_data.imu.timestamp;
        //
        //   stop_timer(&tWall);
        //   print_timer(&tWall);
        // }
    }
    zed.close();
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}
